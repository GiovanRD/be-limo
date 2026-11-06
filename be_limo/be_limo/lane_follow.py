"""Mode route : suivre la voie vue par la caméra (sans ROS).

vision_node publie /lane_detection = [valide, décalage, cap, confiance] :
le centre de la voie est à `offset` m à gauche du robot, à x_ref m devant,
et la voie fait l'angle `heading` avec l'axe du robot.

Loi de commande (pure pursuit sur le centre de la voie) : on vise le point
de la ligne centrale à `lookahead` m devant, y = offset + tan(heading) (lookahead - x_ref) ;
la courbure qui y mène est kappa = 2 y / (x² + y²), et le braquage
delta = atan(L kappa). Plus le point est proche, plus le robot rejoint vite le
centre (mais plus il réagit au bruit de la détection). On ralentit dans les virages et quand la
détection est peu sûre. Sans voie pendant `lost_time`, on s'arrête.
"""

import math
from dataclasses import dataclass


@dataclass
class LaneFollowParams:
    x_ref: float = 0.70             # doit être celui de vision_node (LaneParams.x_ref)
    wheelbase: float = 0.20
    max_steering: float = math.radians(25.0)
    min_confidence: float = 0.3
    lost_time: float = 0.6          # s sans voie -> arrêt
    curve_slowdown: float = 1.5     # v = v0 / (1 + k |kappa|)
    smoothing: float = 0.5          # filtre du braquage (0 = aucun, 1 = figé)
    lookahead: float = 0.50         # point visé sur la ligne centrale [m]


class LaneFollower:

    def __init__(self, params=None):
        self.p = params or LaneFollowParams()
        self.delta = 0.0
        self.last_seen = None
        self.state = 'pas de voie'
        self.kappa = 0.0

    def reset(self):
        self.delta = 0.0
        self.last_seen = None
        self.state = 'pas de voie'

    def update(self, now, lane, cruise):
        """lane = (valide, décalage, cap, confiance) ou None. Renvoie (v, delta)."""
        p = self.p
        ok = lane is not None and lane[0] > 0.5 and lane[3] >= p.min_confidence
        if ok:
            _, offset, heading, conf = lane
            x = p.lookahead
            y = offset + math.tan(heading) * (x - p.x_ref)
            kappa = 2.0 * y / (x * x + y * y)
            target = math.atan(p.wheelbase * kappa)
            target = max(-p.max_steering, min(p.max_steering, target))
            self.delta = p.smoothing * self.delta + (1.0 - p.smoothing) * target
            self.kappa = kappa
            self.last_seen = now
            scale = min(1.0, 0.5 + conf) / (1.0 + p.curve_slowdown * abs(kappa))
            self.state = 'suit la voie'
            return cruise * scale, self.delta
        if self.last_seen is not None and now - self.last_seen < p.lost_time:
            self.state = 'voie perdue un instant'
            return cruise * 0.5, self.delta
        self.state = 'pas de voie : arrêt'
        return 0.0, self.delta
