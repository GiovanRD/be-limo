"""Arrêt devant les obstacles, pour tous les modes de pilotage (sans ROS).

On regarde le couloir que le robot va balayer avec la commande demandée
(vitesse v, braquage delta) : un arc de courbure tan(delta) / L, de la
largeur du robot plus une marge (obstacles.detect). En marche arrière, on
regarde derrière (points retournés).

    distance < stop         -> vitesse nulle dans ce sens
    distance < slow         -> vitesse réduite (linéaire entre stop et slow)
    on ne repart qu'au-dessus de resume (hystérésis, pas d'à-coups)
"""

import math
from dataclasses import dataclass

from be_limo.obstacles import ObstacleParams, detect


@dataclass
class SafetyParams:
    stop: float = 0.35          # m
    resume: float = 0.45        # m
    slow: float = 0.80          # m
    min_scale: float = 0.3      # vitesse minimale (fraction) juste avant l'arrêt
    wheelbase: float = 0.20


class ObstacleGuard:

    def __init__(self, params=None, obstacle_params=None):
        self.p = params or SafetyParams()
        self.op = obstacle_params or ObstacleParams()
        self.blocked = {1: False, -1: False}     # par sens de marche
        self.distance = float('inf')

    def free_distance(self, points, direction, delta, exclude=None):
        """Distance libre dans le couloir, dans le sens direction (+1 / -1)."""
        curvature = math.tan(delta) / self.p.wheelbase
        pts = points
        if exclude is not None:
            ex, ey, er = exclude
            pts = [q for q in pts if math.hypot(q[0] - ex, q[1] - ey) > er]
        if direction < 0:
            # Derrière : on retourne l'avant et l'arrière. Le centre du virage
            # reste du même côté, donc la courbure est la même.
            pts = [(-x, y) for x, y in pts]
        return detect(pts, curvature, self.op).distance

    def limit(self, points, v, delta, exclude=None):
        """Vitesse autorisée (même signe que v, |.| <= |v|) et distance libre."""
        if abs(v) < 1e-6:
            self.distance = float('inf')
            return 0.0, self.distance
        direction = 1 if v > 0 else -1
        d = self.free_distance(points, direction, delta, exclude)
        self.distance = d
        p = self.p
        if d < p.stop:
            self.blocked[direction] = True
        elif d > p.resume:
            self.blocked[direction] = False
        if self.blocked[direction]:
            return 0.0, d
        if d < p.slow:
            scale = p.min_scale + (1.0 - p.min_scale) * (d - p.stop) / (p.slow - p.stop)
            return v * max(p.min_scale, min(1.0, scale)), d
        return v, d
