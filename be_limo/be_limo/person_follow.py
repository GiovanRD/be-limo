"""Mode suivi : suivre la personne devant le robot, repérée au lidar (sans ROS).

À la hauteur du lidar de la LIMO (~15 cm), une personne donne une ou deux
petites taches (les jambes, ou le bas d'un pantalon), de 3 à 45 cm de large.
Un mur ou une table donne une longue tache : ignorée.

1. Taches du scan, on garde les petites et non masquées (un morceau de mur
   vu entre deux jambes a ses deux bords cachés par un point plus proche :
   ignoré) ; deux taches à moins de 45 cm l'une de l'autre = une personne.
2. Accroche : la personne la plus proche devant le robot (cône de ±35°,
   entre 0,3 et 2 m), au démarrage du mode ou sur « choisir la personne ».
3. Suivi : à chaque scan, la candidate la plus proche de la position
   prévue (dernière position corrigée du mouvement du robot), à moins de 0,5 m.
   Perdue pendant `lost_time` : arrêt, et on attend une nouvelle accroche.
4. Commande : v proportionnelle à l'écart à la distance voulue (0,7 m), jamais
   en arrière ; braquage par pure pursuit vers la personne.
"""

import math
from dataclasses import dataclass



@dataclass
class FollowParams:
    distance: float = 0.70          # distance de suivi voulue [m]
    too_close: float = 0.45         # en dessous : arrêt [m]
    gain: float = 0.8               # v = gain * (d - distance)
    wheelbase: float = 0.20
    max_steering: float = math.radians(25.0)
    cluster_gap: float = 0.10       # m
    min_width: float = 0.03         # m : taille d'une jambe
    max_width: float = 0.45         # m : au-delà, mur ou meuble
    pair_distance: float = 0.45     # deux jambes
    acquire_cone: float = math.radians(35.0)
    acquire_min: float = 0.30
    acquire_max: float = 2.00
    gate: float = 0.50              # saut maximal entre deux scans [m]
    laser_x: float = 0.1236         # position du lidar dans base_link [m]
    occlusion_angle: float = math.radians(3.0)   # voisin « juste à côté »
    occlusion_depth: float = 0.10   # voisin plus proche de ça : il masque [m]
    lost_time: float = 1.0          # s


def _polar(pt, laser_x):
    x = pt[0] - laser_x
    return math.hypot(x, pt[1]), math.atan2(pt[1], x)


def _occluded(edge, neighbour, p):
    """Le bord `edge` est-il caché par `neighbour` (voisin dans le scan) ?"""
    if neighbour is None:
        return False
    re, ae = _polar(edge, p.laser_x)
    rn, an = _polar(neighbour, p.laser_x)
    da = abs(math.atan2(math.sin(an - ae), math.cos(an - ae)))
    return da <= p.occlusion_angle and rn < re - p.occlusion_depth


def person_candidates(points, p=None):
    """Positions (x, y) des personnes possibles dans le scan (repère base_link).

    `points` : points du scan dans l'ordre des angles (obstacles.scan_to_points).
    """
    p = p or FollowParams()
    n = len(points)
    # Taches : indices consécutifs à moins de cluster_gap les uns des autres.
    spans, start = [], 0
    for i in range(1, n + 1):
        if i == n or math.hypot(points[i][0] - points[i - 1][0],
                                points[i][1] - points[i - 1][1]) > p.cluster_gap:
            spans.append((start, i - 1))
            start = i
    blobs = []
    for a, b in spans:
        if b - a + 1 < 2:
            continue
        c = points[a:b + 1]
        width = math.hypot(c[0][0] - c[-1][0], c[0][1] - c[-1][1])
        if not p.min_width <= width <= p.max_width:
            continue
        before = points[a - 1] if n > 1 else None          # le scan fait le tour
        after = points[(b + 1) % n] if n > 1 else None
        if _occluded(c[0], before, p) or _occluded(c[-1], after, p):
            continue
        blobs.append((sum(q[0] for q in c) / len(c), sum(q[1] for q in c) / len(c)))
    # Regroupe les jambes : chaque tache rejoint le groupe le plus proche.
    people = []
    for b in blobs:
        for grp in people:
            gx = sum(q[0] for q in grp) / len(grp)
            gy = sum(q[1] for q in grp) / len(grp)
            if math.hypot(b[0] - gx, b[1] - gy) <= p.pair_distance:
                grp.append(b)
                break
        else:
            people.append([b])
    return [(sum(q[0] for q in g) / len(g), sum(q[1] for q in g) / len(g)) for g in people]


def move_point(pt, v, omega, dt):
    """Position d'un point fixe dans le repère du robot après un mouvement (v, omega, dt)."""
    dth = omega * dt
    # déplacement du robot dans son ancien repère
    if abs(omega) < 1e-6:
        dx, dy = v * dt, 0.0
    else:
        r = v / omega
        dx, dy = r * math.sin(dth), r * (1.0 - math.cos(dth))
    x, y = pt[0] - dx, pt[1] - dy
    c, s = math.cos(-dth), math.sin(-dth)
    return c * x - s * y, s * x + c * y


class PersonFollower:

    def __init__(self, params=None):
        self.p = params or FollowParams()
        self.target = None          # (x, y) dans base_link
        self.last_seen = None
        self.last_t = None
        self.lost = False
        self.state = 'pas de cible'

    def reset(self):
        self.target = None
        self.last_seen = None
        self.last_t = None
        self.lost = False
        self.state = 'pas de cible'

    def acquire(self, candidates):
        p = self.p
        best = None
        for x, y in candidates:
            d = math.hypot(x, y)
            if x > 0 and abs(math.atan2(y, x)) <= p.acquire_cone and p.acquire_min <= d <= p.acquire_max:
                if best is None or d < math.hypot(*best):
                    best = (x, y)
        return best

    def update(self, now, points, v_robot, omega_robot, cruise):
        """Nouveau scan. Renvoie (v, delta) ; self.target = cible (ou None)."""
        p = self.p
        dt = 0.0 if self.last_t is None else max(0.0, min(0.5, now - self.last_t))
        self.last_t = now
        candidates = person_candidates(points, p)
        if self.target is None:
            self.target = self.acquire(candidates)
            if self.target is None:
                self.state = ('personne perdue : arrêt, attend une personne devant' if self.lost
                              else 'personne devant ? (cône ±35°, 0,3 à 2 m)')
                return 0.0, 0.0
            self.last_seen = now
            self.lost = False
        else:
            predicted = move_point(self.target, v_robot, omega_robot, dt)
            best, best_d = None, p.gate
            for c in candidates:
                dd = math.hypot(c[0] - predicted[0], c[1] - predicted[1])
                if dd <= best_d:
                    best, best_d = c, dd
            if best is not None:
                self.target, self.last_seen = best, now
            else:
                self.target = predicted
                if now - self.last_seen > p.lost_time:
                    self.reset()
                    self.lost = True
                    self.state = 'personne perdue : arrêt, attend une personne devant'
                    return 0.0, 0.0

        x, y = self.target
        d = math.hypot(x, y)
        kappa = 2.0 * y / max(d * d, 1e-6)
        delta = max(-p.max_steering, min(p.max_steering, math.atan(p.wheelbase * kappa)))
        if d < p.too_close or x <= 0:
            self.state = 'trop près : attend'
            return 0.0, delta
        v = max(0.0, min(cruise, p.gain * (d - p.distance)))
        self.state = 'suit (%.2f m)' % d if v > 0 else 'à distance : attend'
        return v, delta
