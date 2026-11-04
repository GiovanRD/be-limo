"""Draw a traffic light or a stop sign in a synthetic camera image."""
import cv2


def project(cam, x, y, z):
    """Pixel of a point (x forward, y left, z up) relative to base_link, pitch 0."""
    xf = x - cam.cam_x
    return cam.cx + cam.f * (-(y - cam.cam_y)) / xf, cam.cy + cam.f * (cam.cam_height - z) / xf


def draw_traffic_light(img, cam, x, y, state):
    """Housing 0.08 x 0.16 m centred at 0.375 m high, bulbs of 0.034 m."""
    s = cam.f / (x - cam.cam_x)
    u0, v0 = project(cam, x, y, 0.375)
    cv2.rectangle(img, (int(u0 - 0.04 * s), int(v0 - 0.08 * s)), (int(u0 + 0.04 * s), int(v0 + 0.08 * s)),
                  (20, 20, 22), -1)
    for z, colour, on in ((0.42, (13, 13, 255), state == 'red'), (0.375, (5, 50, 75), False),
                          (0.33, (13, 255, 13), state == 'green')):
        u, v = project(cam, x, y, z)
        c = colour if on else tuple(int(k * 0.25) for k in colour)
        cv2.circle(img, (int(round(u)), int(round(v))), max(1, int(round(0.017 * s))), c, -1)
    pu, pv = project(cam, x, y, 0.0)
    cv2.line(img, (int(pu), int(v0 + 0.08 * s)), (int(pu), int(pv)), (20, 20, 22), max(1, int(0.024 * s)))


def draw_stop_sign(img, cam, x, y, z=0.30):
    s = cam.f / (x - cam.cam_x)
    u, v = project(cam, x, y, z)
    cv2.circle(img, (int(round(u)), int(round(v))), max(2, int(round(0.08 * s))), (235, 235, 235), -1)
    cv2.circle(img, (int(round(u)), int(round(v))), max(1, int(round(0.05 * s))), (0, 0, 190), -1)
