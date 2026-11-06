"""Téléopération : cinématique et règles de sécurité (sans ROS)."""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.join(HERE, '..'), HERE]

from be_limo.teleop import (STEER_IN_PLACE_V, WHEELBASE, TeleopState, omega_from_steering,  # noqa: E402
                            steering_from_omega, wheel_angles)


def driver_steering(v, omega, L=0.2):
    """Ce que fait limo_base en Ackermann (limo_driver.cpp) : r = v / omega, delta = atan(L / r)."""
    if abs(v) < 1e-6 or abs(omega) < 1e-6:
        return 0.0
    return math.atan(L / (v / omega))


def test_driver_recovers_the_steering_angle():
    for v in (0.15, -0.1, STEER_IN_PLACE_V):
        for d in (-0.4, -0.1, 0.05, 0.3):
            w = omega_from_steering(v, d)
            assert abs(driver_steering(v, w) - d) < 1e-9
            assert abs(steering_from_omega(v, w) - d) < 1e-9
    # Le driver arrondit la vitesse en mm/s (int16) : la vitesse « braquer à l'arrêt » donne 0.
    assert int(STEER_IN_PLACE_V * 1000) == 0
    assert WHEELBASE == 0.2


def test_ackermann_inner_wheel_turns_more():
    left, right = wheel_angles(0.3)          # virage à gauche
    assert left > 0.3 > right > 0
    left, right = wheel_angles(-0.3)
    assert left > -0.3 > right and abs(right) > abs(left)
    assert wheel_angles(0.0) == (0.0, 0.0)


def run_for(t, state, start, duration, dt=0.05):
    out, now = None, start
    while now < start + duration - 1e-9:
        now += dt
        state.command(now)                    # battement de cœur de la page
        out = state.step(now)
    return out, now


def test_nothing_is_sent_before_arming():
    s = TeleopState()
    s.command(0.0, v=0.2)
    assert s.step(0.05) is None


def test_limits_and_ramp():
    s = TeleopState(max_speed=0.15, max_steering=math.radians(25), accel=0.5)
    s.command(0.0, armed=True, v=1.0, steering=1.0)
    assert s.target_v == 0.15 and abs(s.delta - math.radians(25)) < 1e-9
    s.step(0.0)
    v, _ = s.step(0.1)
    assert abs(v - 0.05) < 1e-9               # 0,5 m/s² pendant 0,1 s
    (v, w), _ = run_for(None, s, 0.1, 1.0)
    assert abs(v - 0.15) < 1e-9
    assert abs(w - omega_from_steering(0.15, math.radians(25))) < 1e-9


def test_watchdog_stops_and_disarms():
    s = TeleopState(timeout=0.5)
    s.command(0.0, armed=True, v=0.1)
    _, now = run_for(None, s, 0.0, 1.0)
    assert s.v > 0
    # la page ne répond plus
    out = s.step(now + 0.6)
    assert out == (0.0, 0.0) and not s.armed and 'chien de garde' in s.reason
    # quelques zéros, puis plus rien (pour ne pas gêner local_controller)
    outs = [s.step(now + 0.6 + 0.05 * k) for k in range(1, 30)]
    assert outs[0] == (0.0, 0.0) and outs[-1] is None
    # une commande sans « armed » ne réarme pas
    s.command(now + 2.0, v=0.1)
    assert s.step(now + 2.05) is None or s.step(now + 2.05) == (0.0, 0.0)
    assert not s.armed


def test_stop_is_immediate():
    s = TeleopState()
    s.command(0.0, armed=True, v=0.2)
    run_for(None, s, 0.0, 1.0)
    s.stop('test')
    assert s.step(1.05) == (0.0, 0.0) and s.v == 0.0 and not s.armed


def test_zero_speed_straightens_wheels_unless_steer_in_place():
    s = TeleopState()
    s.command(0.0, armed=True, v=0.0, steering=0.3)
    assert s.step(0.05) == (0.0, 0.0)
    s.command(0.06, steer_in_place=True)
    v, w = s.step(0.1)
    assert v == STEER_IN_PLACE_V and abs(driver_steering(v, w) - 0.3) < 1e-9


def test_releasing_the_key_stops_at_once():
    s = TeleopState(accel=0.5)
    s.command(0.0, armed=True, v=0.15)
    (v, _), now = run_for(None, s, 0.0, 1.0)
    assert abs(v - 0.15) < 1e-9
    s.command(now, v=0.0)                     # touche lâchée
    assert s.step(now + 0.02) == (0.0, 0.0)
    # ralentir aussi est immédiat
    s.command(now + 0.03, v=0.15)
    run_for(None, s, now + 0.03, 1.0)
    s.command(now + 1.1, v=0.05)
    v, _ = s.step(now + 1.12)
    assert abs(v - 0.05) < 1e-9
    # changement de sens : arrêt d'abord, puis rampe
    s.command(now + 1.13, v=-0.15)
    assert s.step(now + 1.15) == (0.0, 0.0)
    v, _ = s.step(now + 1.25)
    assert -0.06 < v < 0


def test_reverse_keeps_the_same_steering():
    s = TeleopState(accel=10.0)
    s.command(0.0, armed=True, v=-0.1, steering=0.2)
    s.step(0.0)
    v, w = s.step(0.1)
    assert v < 0 and w < 0 and abs(driver_steering(v, w) - 0.2) < 1e-9
