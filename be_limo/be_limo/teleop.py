"""Téléopération : cinématique Ackermann et règles de sécurité (sans ROS).

La commande est donnée par l'interface en vitesse v [m/s] et angle de braquage
delta [rad] (angle « central », celui d'une roue avant fictive au milieu de
l'essieu). Elle est envoyée sur /cmd_vel en (v, omega) avec

    omega = v * tan(delta) / L,       L = 0,20 m (empattement du driver limo_base)

Le driver fait l'opération inverse (r = v / omega, delta = atan(L / r)), donc
il retrouve exactement delta. Le même L est utilisé ici pour cette raison.

Sécurité, dans TeleopState :
* rien n'est envoyé tant que l'interface n'est pas « armée » ;
* l'interface doit envoyer une commande au moins toutes les `timeout` s
  (chien de garde) : onglet fermé, réseau coupé, PC planté -> arrêt ;
* vitesse et braquage bornés ; accélération en rampe (pas d'à-coups),
  ralentissement et arrêt immédiats ;
* désarmé : quelques commandes nulles, puis plus rien, pour ne pas gêner
  local_controller s'il tourne.
"""

import math

WHEELBASE = 0.20      # m, comme limo_base (limo_driver.h)
TRACK = 0.172         # m, voie (distance gauche-droite des roues)

# En Ackermann, le driver envoie un braquage nul quand |v| < 1e-6. Pour voir
# les roues braquer à l'arrêt, on envoie une vitesse si petite que le driver
# l'arrondit à 0 mm/s dans la trame série (int16 en mm/s), mais qui garde le
# braquage.
STEER_IN_PLACE_V = 2e-4


def omega_from_steering(v, delta, wheelbase=WHEELBASE):
    """Vitesse angulaire [rad/s] pour rouler à v avec le braquage delta."""
    return v * math.tan(delta) / wheelbase


def steering_from_omega(v, omega, wheelbase=WHEELBASE):
    """Braquage central équivalent (comme le driver). 0 si v = 0."""
    if abs(v) < 1e-6 or abs(omega) < 1e-9:
        return 0.0
    return math.atan(wheelbase * omega / v)


def wheel_angles(delta, wheelbase=WHEELBASE, track=TRACK):
    """(gauche, droite) : angle de chaque roue avant (Ackermann idéal)."""
    if abs(delta) < 1e-6:
        return 0.0, 0.0
    r = wheelbase / math.tan(delta)          # rayon au centre de l'essieu arrière, signé
    left = math.atan(wheelbase / (r - track / 2.0))
    right = math.atan(wheelbase / (r + track / 2.0))
    return left, right


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class TeleopState:
    """Machine d'état de la téléopération. `now` en secondes (monotone)."""

    def __init__(self, max_speed=0.30, max_steering=math.radians(25.0),
                 accel=0.5, timeout=0.5, idle_zero_count=10):
        self.max_speed = max_speed
        self.max_steering = max_steering
        self.accel = accel                   # m/s²
        self.timeout = timeout               # s sans commande -> arrêt
        self.idle_zero_count = idle_zero_count
        self.armed = False
        self.target_v = 0.0
        self.delta = 0.0
        self.steer_in_place = False
        self.v = 0.0                         # vitesse envoyée (après rampe)
        self.last_cmd_time = None
        self.last_step_time = None
        self.zeros_left = 0
        self.reason = 'désarmé'

    # ------------------------------------------------------------ entrées
    def command(self, now, v=None, steering=None, armed=None, steer_in_place=None):
        """Commande reçue de l'interface (sert aussi de battement de cœur)."""
        if armed is not None:
            if armed and not self.armed:
                self.reason = 'armé'
            if not armed and self.armed:
                self.disarm('désarmé par l\'interface')
            self.armed = bool(armed)
        if v is not None:
            self.target_v = clamp(float(v), -self.max_speed, self.max_speed)
        if steering is not None:
            self.delta = clamp(float(steering), -self.max_steering, self.max_steering)
        if steer_in_place is not None:
            self.steer_in_place = bool(steer_in_place)
        self.last_cmd_time = now

    def stop(self, reason='arrêt'):
        """Arrêt immédiat (bouton ARRÊT, touche espace)."""
        self.target_v = 0.0
        self.v = 0.0
        self.disarm(reason)

    def disarm(self, reason):
        self.armed = False
        self.target_v = 0.0
        self.v = 0.0
        self.zeros_left = self.idle_zero_count
        self.reason = reason

    # ------------------------------------------------------------- sortie
    def step(self, now):
        """(v, omega) à publier sur /cmd_vel, ou None pour ne rien publier."""
        dt = 0.0 if self.last_step_time is None else max(0.0, now - self.last_step_time)
        self.last_step_time = now
        if self.armed and (self.last_cmd_time is None or now - self.last_cmd_time > self.timeout):
            self.disarm('plus de nouvelles de l\'interface (chien de garde)')
        if not self.armed:
            if self.zeros_left > 0:
                self.zeros_left -= 1
                return 0.0, 0.0
            return None
        # Ralentir ou s'arrêter : immédiat (touche lâchée = arrêt).
        # Accélérer : en rampe, pour éviter les à-coups. Changement de sens :
        # arrêt, puis rampe dans l'autre sens.
        if self.target_v * self.v < 0:
            self.v = 0.0
        elif abs(self.target_v) <= abs(self.v):
            self.v = self.target_v
        else:
            max_dv = self.accel * dt
            self.v += clamp(self.target_v - self.v, -max_dv, max_dv)
        if abs(self.v) < 1e-4 and abs(self.target_v) < 1e-4:
            self.v = 0.0
        if abs(self.v) < 1e-6:
            if self.steer_in_place and abs(self.delta) > 1e-3:
                return STEER_IN_PLACE_V, omega_from_steering(STEER_IN_PLACE_V, self.delta)
            return 0.0, 0.0
        # En marche arrière, v < 0 donne omega < 0 : le driver (r = v / omega)
        # retrouve le même braquage, comme une voiture.
        return self.v, omega_from_steering(self.v, self.delta)

    def status(self, now):
        age = None if self.last_cmd_time is None else round(now - self.last_cmd_time, 2)
        return {'armed': self.armed, 'target_v': round(self.target_v, 3), 'v': round(self.v, 3),
                'steering': round(self.delta, 4), 'steer_in_place': self.steer_in_place,
                'max_speed': self.max_speed, 'max_steering': round(self.max_steering, 4),
                'reason': self.reason, 'cmd_age': age}
