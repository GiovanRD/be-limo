"""Odométrie corrigée : distance des roues + cap du gyroscope (sans ROS).

Sur la LIMO 201, le châssis annonce le mode Mecanum alors qu'il roule en
Ackermann : limo_base calcule alors une vitesse angulaire nulle et le cap de
/odom ne change jamais (diagnostic du 06/11/2026). La vitesse linéaire de
/odom est bonne. On garde donc la distance parcourue donnée par les roues et
on prend le cap en intégrant le gyroscope (/limo/imu, angular_velocity.z).

Le biais du gyroscope (quelques 0,1 °/s) est estimé quand le robot est
immobile, puis retranché. Roues arrêtées, le cap ne change pas (une voiture ne
tourne pas sur place) : on ne cumule pas le bruit du gyroscope à l'arrêt.
"""

import math


class GyroOdometry:

    def __init__(self, bias_alpha=0.02, still_speed=0.005, still_time=0.5, gyro_scale=1.0):
        self.x = self.y = self.yaw = 0.0
        self.v = 0.0
        self.omega = 0.0
        self.bias = 0.0
        self.bias_alpha = bias_alpha       # filtre du biais (par mesure, à l'arrêt)
        self.still_speed = still_speed     # m/s : en dessous, robot immobile
        self.still_time = still_time       # s d'immobilité avant d'estimer le biais
        self.gyro_scale = gyro_scale
        self.still_since = None
        self.last_t = None
        self.gyro_raw = 0.0

    def update_speed(self, t, v):
        """Vitesse linéaire des roues (/odom twist.linear.x)."""
        self.v = v
        if abs(v) < self.still_speed:
            if self.still_since is None:
                self.still_since = t
        else:
            self.still_since = None

    def is_still(self, t):
        return self.still_since is not None and t - self.still_since >= self.still_time

    def update_gyro(self, t, gyro_z):
        """Intègre une mesure du gyroscope ; renvoie la pose (x, y, yaw)."""
        self.gyro_raw = gyro_z
        if self.still_since is not None:
            # Roues arrêtées : une voiture (Ackermann) ne tourne pas sur place,
            # le cap ne bouge pas. Après still_time, on apprend le biais.
            if self.is_still(t):
                self.bias += self.bias_alpha * (gyro_z - self.bias)
            omega = 0.0
        else:
            omega = self.gyro_scale * (gyro_z - self.bias)
        self.omega = omega
        if self.last_t is not None:
            dt = t - self.last_t
            if 0.0 < dt < 0.5:               # trou de mesures : on n'intègre pas
                mid = self.yaw + 0.5 * omega * dt
                self.x += self.v * math.cos(mid) * dt
                self.y += self.v * math.sin(mid) * dt
                self.yaw = math.atan2(math.sin(self.yaw + omega * dt), math.cos(self.yaw + omega * dt))
        self.last_t = t
        return self.x, self.y, self.yaw
