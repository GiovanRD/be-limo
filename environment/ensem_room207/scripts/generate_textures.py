#!/usr/bin/env python3
"""Generate the textures of the Room 207 model.

Run once after changing the patterns (needs python3-pil and the Liberation
and DejaVu fonts):

    python3 scripts/generate_textures.py [--gazebo-view RENDER.jpg]

- bricks and floor tiles: each texture covers a square of BRICK_TEXTURE_SIZE /
  FLOOR_TEXTURE_SIZE metres (see generate_room.py) and is repeated;
- whiteboards: bicycle model drawing and state-space equations, "handwritten";
- computer screens: Simulink, terminal and Gazebo windows. The Gazebo screen
  shows a render of the room given with --gazebo-view (kept as is otherwise).
"""

import argparse
import math
import random
from pathlib import Path

from PIL import Image, ImageDraw, ImageFilter, ImageFont

PACKAGE_DIR = Path(__file__).resolve().parent.parent
TEXTURE_DIR = PACKAGE_DIR / 'models' / 'room207_room' / 'materials' / 'textures'
PIXELS = 512

HAND_FONT = '/usr/share/fonts/truetype/liberation/LiberationSans-Italic.ttf'
UI_FONT = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
UI_BOLD = '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'
MONO_FONT = '/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf'
MONO_BOLD = '/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf'

# Marker colours.
BLACK_MARKER = (35, 38, 48)
BLUE_MARKER = (25, 70, 160)
RED_MARKER = (190, 35, 40)
GREEN_MARKER = (20, 120, 60)
BOARD_WHITE = (238, 239, 236)


def text_width(font, text):
    """Width of the text in pixels (Pillow 7 on Ubuntu 20.04 has no getlength)."""
    return font.getsize(text)[0] if text else 0


def bricks():
    """0.90 m x 0.90 m of orange/brown bricks in running bond.

    4 bricks per course (22.5 cm with joint), 12 courses (7.5 cm).
    """
    rng = random.Random(207)
    image = Image.new('RGB', (PIXELS, PIXELS), (178, 168, 152))  # Mortar
    draw = ImageDraw.Draw(image)
    columns, courses = 4, 12
    w, h = PIXELS / columns, PIXELS / courses
    joint = 4
    for row in range(courses):
        offset = w / 2 if row % 2 else 0
        for col in range(-1, columns + 1):
            x0 = col * w + offset
            shade = rng.uniform(-1, 1)
            base = (
                int(165 + 35 * shade + rng.uniform(-8, 8)),
                int(82 + 20 * shade + rng.uniform(-6, 6)),
                int(52 + 12 * shade + rng.uniform(-5, 5)),
            )
            draw.rectangle(
                [x0 + joint / 2, row * h + joint / 2, x0 + w - joint / 2, (row + 1) * h - joint / 2],
                fill=base,
            )
    # Surface grain.
    pixels = image.load()
    for _ in range(PIXELS * PIXELS // 6):
        x, y = rng.randrange(PIXELS), rng.randrange(PIXELS)
        r, g, b = pixels[x, y]
        d = rng.randint(-14, 14)
        pixels[x, y] = (max(0, r + d), max(0, g + d), max(0, b + d))
    return image.filter(ImageFilter.SMOOTH)


def floor_tiles():
    """0.80 m x 0.80 m: 2 x 2 off-white (blanc casse) 40 cm tiles."""
    rng = random.Random(40)
    image = Image.new('RGB', (PIXELS, PIXELS))
    pixels = image.load()
    tile = PIXELS // 2
    tones = [(222, 217, 204), (216, 211, 198), (225, 220, 208), (219, 214, 201)]
    for y in range(PIXELS):
        for x in range(PIXELS):
            r, g, b = tones[(y // tile) * 2 + x // tile]
            d = rng.randint(-10, 10)
            pixels[x, y] = (r + d, g + d, b + d)
    # Speckles of the mottled tiles.
    draw = ImageDraw.Draw(image)
    for _ in range(900):
        x, y = rng.randrange(PIXELS), rng.randrange(PIXELS)
        c = rng.randint(150, 190)
        draw.ellipse([x, y, x + rng.randint(1, 3), y + rng.randint(1, 3)], fill=(c, c - 4, c - 12))
    image = image.filter(ImageFilter.SMOOTH)
    # Grout lines on the tile borders (half on each side, so the texture tiles).
    draw = ImageDraw.Draw(image)
    grout = (168, 163, 152)
    for k in (0, tile, PIXELS):
        draw.rectangle([k - 2, 0, k + 1, PIXELS], fill=grout)
        draw.rectangle([0, k - 2, PIXELS, k + 1], fill=grout)
    return image


# ============================================================
# Whiteboards
# ============================================================

class Hand:
    """Marker handwriting: every glyph is slightly rotated and shifted.

    Markup: _{...} subscript, ^{...} superscript, \\.c puts a dot over c.
    """

    def __init__(self, image, seed):
        self.image = image
        self.draw = ImageDraw.Draw(image)
        self.rng = random.Random(seed)
        self.fonts = {}

    def font(self, size):
        if size not in self.fonts:
            self.fonts[size] = ImageFont.truetype(HAND_FONT, size)
        return self.fonts[size]

    def glyph(self, x, y, char, size, color):
        font = self.font(size)
        w = int(text_width(font, char)) + size
        tile = Image.new('L', (w, int(size * 1.6)), 0)
        ImageDraw.Draw(tile).text((size // 2, size // 4), char, font=font, fill=255,
                                  stroke_width=max(1, size // 28), stroke_fill=255)
        tile = tile.rotate(self.rng.uniform(-5, 5), resample=Image.BICUBIC)
        jy = self.rng.uniform(-0.05, 0.05) * size
        self.image.paste(color, (int(x - size // 2), int(y - size // 4 + jy)), tile)
        return text_width(font, char) * self.rng.uniform(0.98, 1.06)

    def text(self, x, y, text, size, color):
        """Write text with its top-left corner at (x, y); return the end x."""
        i = 0
        while i < len(text):
            c = text[i]
            if c in '_^' and i + 1 < len(text) and text[i + 1] == '{':
                j = text.index('}', i)
                dy = size * 0.45 if c == '_' else -size * 0.2
                x = self.text(x, y + dy, text[i + 2:j], int(size * 0.62), color)
                i = j + 1
                continue
            if c == '\\' and text[i + 1] == '.':
                char = text[i + 2]
                cx = x + text_width(self.font(size), char) / 2
                r = max(2, size // 14)
                self.draw.ellipse([cx - r, y - r, cx + r, y + r], fill=color)
                x += self.glyph(x, y, char, size, color)
                i += 3
                continue
            x += self.glyph(x, y, c, size, color)
            i += 1
        return x

    def width(self, text, size):
        plain = text.replace('_{', '').replace('^{', '').replace('}', '').replace('\\.', '')
        return text_width(self.font(size), plain) * 1.02

    def line(self, points, color, width=5):
        """Slightly wobbly marker line through the points."""
        wobbly = []
        for (x0, y0), (x1, y1) in zip(points, points[1:]):
            n = max(2, int(math.hypot(x1 - x0, y1 - y0) / 40))
            for k in range(n):
                t = k / n
                wobbly.append((x0 + (x1 - x0) * t + self.rng.uniform(-1.2, 1.2),
                               y0 + (y1 - y0) * t + self.rng.uniform(-1.2, 1.2)))
        wobbly.append(points[-1])
        self.draw.line(wobbly, fill=color, width=width, joint='curve')

    def arrow(self, p0, p1, color, width=5, head=22):
        self.line([p0, p1], color, width)
        a = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
        for da in (2.7, -2.7):
            self.line([p1, (p1[0] + head * math.cos(a + da), p1[1] + head * math.sin(a + da))], color, width)

    def arc(self, center, radius, a0, a1, color, width=4, arrow=True):
        """Arc from angle a0 to a1 (radians, image frame: y down)."""
        n = 24
        pts = [(center[0] + radius * math.cos(a0 + (a1 - a0) * k / n),
                center[1] + radius * math.sin(a0 + (a1 - a0) * k / n)) for k in range(n + 1)]
        self.line(pts, color, width)
        if arrow:
            self.arrow(pts[-3], pts[-1], color, width, head=14)

    def dashed(self, p0, p1, color, width=3, dash=22):
        length = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
        n = int(length / dash)
        for k in range(0, n, 2):
            t0, t1 = k / n, min(1, (k + 1) / n)
            self.draw.line([(p0[0] + (p1[0] - p0[0]) * t0, p0[1] + (p1[1] - p0[1]) * t0),
                            (p0[0] + (p1[0] - p0[0]) * t1, p0[1] + (p1[1] - p0[1]) * t1)],
                           fill=color, width=width)

    def fraction(self, x, y, num, den, size, color):
        """Stacked fraction centred on the text line starting at y; return the end x."""
        w = max(self.width(num, size), self.width(den, size)) + 10
        self.text(x + (w - self.width(num, size)) / 2, y - size * 0.62, num, size, color)
        self.line([(x, y + size * 0.48), (x + w, y + size * 0.48)], color, 3)
        self.text(x + (w - self.width(den, size)) / 2, y + size * 0.62, den, size, color)
        return x + w + 8

    def bracket(self, x, y0, y1, color, left=True):
        d = 14 if left else -14
        self.line([(x + d, y0), (x, y0), (x, y1), (x + d, y1)], color, 4)


def board_base(width, height):
    image = Image.new('RGB', (width, height), BOARD_WHITE)
    draw = ImageDraw.Draw(image)
    rng = random.Random(width)
    # Faint traces of erased marker.
    for _ in range(14):
        x, y = rng.randrange(width), rng.randrange(height)
        draw.line([(x, y), (x + rng.randint(80, 300), y + rng.randint(-30, 30))],
                  fill=(226, 228, 226), width=rng.randint(10, 30))
    return image.filter(ImageFilter.GaussianBlur(4))


def rot(origin, point, angle):
    """Rotate point around origin by angle (image frame, y down, angle CCW on screen)."""
    c, s = math.cos(angle), math.sin(angle)
    dx, dy = point[0] - origin[0], point[1] - origin[1]
    return (origin[0] + c * dx + s * dy, origin[1] - s * dx + c * dy)


def wheel(hand, center, angle, color):
    """Wheel drawn as a thick segment of a rectangle along the given heading."""
    L, W = 95, 26
    corners = [(-L / 2, -W / 2), (L / 2, -W / 2), (L / 2, W / 2), (-L / 2, W / 2), (-L / 2, -W / 2)]
    pts = [rot((0, 0), c, angle) for c in corners]
    hand.line([(center[0] + x, center[1] + y) for x, y in pts], color, 5)


def whiteboard_bicycle():
    """Track whiteboard (2.40 m x 1.10 m): kinematic bicycle model."""
    image = board_base(1920, 880)
    hand = Hand(image, 1)

    hand.text(70, 40, 'Kinematic bicycle model', 54, BLUE_MARKER)
    hand.line([(70, 112), (690, 108)], BLUE_MARKER, 4)

    # Ground frame.
    o = (120, 800)
    hand.arrow(o, (390, 800), BLACK_MARKER)
    hand.arrow(o, (120, 560), BLACK_MARKER)
    hand.text(400, 772, 'X', 40, BLACK_MARKER)
    hand.text(92, 505, 'Y', 40, BLACK_MARKER)
    hand.text(80, 812, 'O', 36, BLACK_MARKER)

    # Vehicle: rear wheel R, centre of gravity G, front wheel F, heading psi.
    psi = math.radians(24)
    delta = math.radians(28)
    beta = math.radians(11)
    R = (250, 660)
    lr, lf = 260, 330
    G = rot(R, (R[0] + lr, R[1]), psi)
    F = rot(R, (R[0] + lr + lf, R[1]), psi)
    hand.line([R, F], BLACK_MARKER, 7)
    wheel(hand, R, psi, BLACK_MARKER)
    wheel(hand, F, psi + delta, BLACK_MARKER)
    hand.draw.ellipse([G[0] - 13, G[1] - 13, G[0] + 13, G[1] + 13], outline=BLACK_MARKER, width=5)
    hand.text(G[0] - 10, G[1] + 22, 'G', 38, BLACK_MARKER)

    # Heading angle psi.
    hand.dashed(R, (R[0] + 300, R[1]), BLACK_MARKER)
    hand.arc(R, 110, 0, -psi, BLUE_MARKER)
    hand.text(R[0] + 122, R[1] - 48, 'ψ', 40, BLUE_MARKER)

    # Steering angle delta.
    ahead = rot(F, (F[0] + 150, F[1]), psi)
    hand.dashed(F, ahead, BLACK_MARKER)
    hand.arc(F, 105, -psi, -psi - delta, BLUE_MARKER)
    hand.text(F[0] + 100, F[1] - 105, 'δ', 42, BLUE_MARKER)

    # Velocity at G and side slip beta.
    V = rot(G, (G[0] + 260, G[1]), psi + beta)
    hand.arrow(G, V, RED_MARKER, 6)
    hand.text(V[0] + 10, V[1] - 40, 'v', 44, RED_MARKER)
    hand.arc(G, 190, -psi, -psi - beta, RED_MARKER, arrow=False)
    hand.text(G[0] + 195, G[1] - 125, 'β', 40, RED_MARKER)

    # Lengths l_r and l_f, offset under the body.
    n = rot((0, 0), (0, 95), psi)
    a, b, c = [(p[0] + n[0], p[1] + n[1]) for p in (R, G, F)]
    hand.line([a, b], GREEN_MARKER, 3)
    hand.line([b, c], GREEN_MARKER, 3)
    for p in (a, b, c):
        hand.line([(p[0] - n[0] * 0.2, p[1] - n[1] * 0.2), (p[0] + n[0] * 0.2, p[1] + n[1] * 0.2)], GREEN_MARKER, 3)
    hand.text((a[0] + b[0]) / 2 + 10, (a[1] + b[1]) / 2 + 12, 'l_{r}', 38, GREEN_MARKER)
    hand.text((b[0] + c[0]) / 2 + 10, (b[1] + c[1]) / 2 + 12, 'l_{f}', 38, GREEN_MARKER)

    # Equations.
    x0, size, y = 1060, 50, 170
    hand.text(x0, y, '\\.x = v cos(ψ + β)', size, BLACK_MARKER)
    hand.text(x0, y + 95, '\\.y = v sin(ψ + β)', size, BLACK_MARKER)
    end = hand.text(x0, y + 210, '\\.ψ = ', size, BLACK_MARKER)
    end = hand.fraction(end, y + 210, 'v', 'l_{r}', size, BLACK_MARKER)
    hand.text(end, y + 210, 'sin β', size, BLACK_MARKER)
    end = hand.text(x0, y + 345, 'β = arctan(', size, BLACK_MARKER)
    end = hand.fraction(end, y + 345, 'l_{r}', 'l_{f} + l_{r}', size, BLACK_MARKER)
    hand.text(end, y + 345, 'tan δ )', size, BLACK_MARKER)
    hand.text(x0, y + 490, 'state  q = [x  y  ψ]^{T}', 44, RED_MARKER)
    hand.text(x0, y + 560, 'input  u = [v  δ]^{T}', 44, RED_MARKER)
    hand.line([(x0 - 40, 150), (x0 - 40, 820)], BLUE_MARKER, 3)
    return image


def whiteboard_state_space():
    """Classroom whiteboard (1.60 m x 1.10 m): linear lateral dynamics."""
    image = board_base(1408, 968)
    hand = Hand(image, 2)
    s = 40

    hand.text(50, 30, 'Lateral dynamics (linear tyres)', 48, BLUE_MARKER)
    hand.line([(50, 95), (720, 92)], BLUE_MARKER, 4)
    hand.text(50, 130, 'x = [v_{y}  r]^{T}     u = δ     y = r', 42, RED_MARKER)

    y = 230
    end = hand.text(50, y, '\\.x = A x + B u', 48, BLACK_MARKER)

    # Matrix A.
    y = 400
    end = hand.text(50, y, 'A = ', s, BLACK_MARKER)
    left = end + 5
    hand.bracket(left, y - 75, y + 205, BLACK_MARKER)
    x = left + 25
    e1 = hand.text(x, y, '−', s, BLACK_MARKER)
    e1 = hand.fraction(e1, y, 'C_{f} + C_{r}', 'm v_{x}', s, BLACK_MARKER)
    e2 = hand.text(e1 + 40, y, '−v_{x} −', s, BLACK_MARKER)
    e2 = hand.fraction(e2, y, 'C_{f} l_{f} − C_{r} l_{r}', 'm v_{x}', s, BLACK_MARKER)
    y2 = y + 150
    f1 = hand.text(x, y2, '−', s, BLACK_MARKER)
    f1 = hand.fraction(f1, y2, 'C_{f} l_{f} − C_{r} l_{r}', 'I_{z} v_{x}', s, BLACK_MARKER)
    f2 = hand.text(max(e1, f1) + 40, y2, '−', s, BLACK_MARKER)
    f2 = hand.fraction(f2, y2, 'C_{f} l_{f}^{2} + C_{r} l_{r}^{2}', 'I_{z} v_{x}', s, BLACK_MARKER)
    hand.bracket(max(e2, f2) + 15, y - 75, y + 205, BLACK_MARKER, left=False)

    # Matrix B and C.
    y = 760
    end = hand.text(50, y, 'B = ', s, BLACK_MARKER)
    hand.bracket(end + 5, y - 70, y + 150, BLACK_MARKER)
    b1 = hand.fraction(end + 30, y - 20, 'C_{f}', 'm', s, BLACK_MARKER)
    b2 = hand.fraction(end + 30, y + 90, 'C_{f} l_{f}', 'I_{z}', s, BLACK_MARKER)
    hand.bracket(max(b1, b2) + 10, y - 70, y + 150, BLACK_MARKER, left=False)
    end = hand.text(max(b1, b2) + 110, y + 30, 'C = [ 0   1 ]', s, BLACK_MARKER)

    # Sketch of the closed-loop yaw rate step response.
    o = (960, 600)
    hand.arrow(o, (1360, 600), BLACK_MARKER, 4)
    hand.arrow(o, (960, 260), BLACK_MARKER, 4)
    hand.text(1330, 612, 't', 36, BLACK_MARKER)
    hand.text(905, 250, 'r', 36, BLACK_MARKER)
    hand.dashed((960, 380), (1340, 380), RED_MARKER)
    hand.text(1270, 330, 'r_{ref}', 34, RED_MARKER)
    curve = []
    for k in range(60):
        t = k / 59 * 6
        curve.append((960 + t * 62, 600 - 220 * (1 - math.exp(-0.9 * t) * (math.cos(1.6 * t) + 0.56 * math.sin(1.6 * t)))))
    hand.line(curve, BLUE_MARKER, 4)

    hand.text(940, 760, 'K = place(A, B, p)', 38, GREEN_MARKER)
    hand.text(940, 830, 'u = −K x', 38, GREEN_MARKER)
    return image


# ============================================================
# Computer screens (1024 x 640, 16:10)
# ============================================================

SCREEN = (1024, 640)


def font(path, size):
    return ImageFont.truetype(path, size)


def screen_simulink():
    image = Image.new('RGB', SCREEN, (255, 255, 255))
    d = ImageDraw.Draw(image)
    # Title bar, ribbon tabs and toolstrip.
    d.rectangle([0, 0, SCREEN[0], 30], fill=(232, 232, 232))
    d.text((12, 7), 'lateral_control - Simulink', font=font(UI_FONT, 14), fill=(30, 30, 30))
    for i, c in enumerate(((236, 106, 94), (245, 191, 79), (98, 197, 84))):
        d.ellipse([SCREEN[0] - 80 + i * 24, 9, SCREEN[0] - 66 + i * 24, 23], fill=c)
    d.rectangle([0, 30, SCREEN[0], 58], fill=(0, 92, 151))
    x = 14
    for tab in ('SIMULATION', 'DEBUG', 'MODELING', 'FORMAT', 'APPS'):
        d.text((x, 37), tab, font=font(UI_BOLD, 12), fill=(255, 255, 255))
        x += text_width(font(UI_BOLD, 12), tab) + 26
    d.rectangle([0, 58, SCREEN[0], 132], fill=(245, 245, 245))
    x = 18
    for label, colour in (('New', (90, 90, 90)), ('Save', (40, 100, 170)), ('Library', (200, 120, 30)),
                          ('Step Back', (90, 90, 90)), ('Run', (40, 150, 60)), ('Step Fwd', (90, 90, 90)),
                          ('Stop', (190, 50, 50)), ('Data Insp.', (120, 60, 160))):
        d.rectangle([x + 10, 66, x + 46, 100], fill=colour)
        d.text((x + 28 - text_width(font(UI_FONT, 11), label) / 2, 108), label, font=font(UI_FONT, 11), fill=(40, 40, 40))
        x += 72
    d.text((640, 74), 'Stop Time  20', font=font(UI_FONT, 13), fill=(40, 40, 40))
    d.text((640, 100), 'Normal', font=font(UI_FONT, 13), fill=(40, 40, 40))
    d.line([(0, 132), (SCREEN[0], 132)], fill=(200, 200, 200))
    # Canvas with blocks.
    blk = font(UI_FONT, 13)

    def block(x0, y0, x1, y1, lines):
        d.rectangle([x0, y0, x1, y1], outline=(0, 0, 0), width=2, fill=(255, 255, 255))
        h = len(lines) * 17
        for i, line in enumerate(lines):
            d.text(((x0 + x1) / 2 - text_width(blk, line) / 2, (y0 + y1) / 2 - h / 2 + i * 17), line, font=blk, fill=(0, 0, 0))

    def arrow(points):
        d.line(points, fill=(0, 0, 0), width=2)
        (x0, y0), (x1, y1) = points[-2], points[-1]
        a = math.atan2(y1 - y0, x1 - x0)
        d.polygon([(x1, y1), (x1 - 10 * math.cos(a - 0.4), y1 - 10 * math.sin(a - 0.4)),
                   (x1 - 10 * math.cos(a + 0.4), y1 - 10 * math.sin(a + 0.4))], fill=(0, 0, 0))

    cy = 300
    block(40, cy - 30, 120, cy + 30, [])
    d.line([(55, cy + 15), (80, cy + 15), (80, cy - 15), (105, cy - 15)], fill=(0, 0, 0), width=2)
    d.text((58, cy + 36), 'r_ref', font=blk, fill=(0, 0, 0))
    d.ellipse([175, cy - 18, 211, cy + 18], outline=(0, 0, 0), width=2)
    d.text((181, cy - 14), '+', font=blk, fill=(0, 0, 0))
    d.text((190, cy + 2), '−', font=blk, fill=(0, 0, 0))
    block(270, cy - 35, 380, cy + 35, ['PID(s)'])
    block(450, cy - 55, 640, cy + 55, ["x' = Ax+Bu", 'y = Cx+Du', '', 'Bicycle model'])
    block(730, cy - 35, 820, cy + 35, [])
    d.rectangle([745, cy - 22, 805, cy + 12], fill=(30, 30, 30))
    d.line([(748, cy + 5), (765, cy - 5), (785, cy - 15), (802, cy - 16)], fill=(240, 220, 40), width=2)
    d.text((750, cy + 40), 'Scope', font=blk, fill=(0, 0, 0))
    arrow([(120, cy), (175, cy)])
    arrow([(211, cy), (270, cy)])
    d.text((222, cy - 22), 'e', font=blk, fill=(0, 0, 0))
    arrow([(380, cy), (450, cy)])
    d.text((405, cy - 22), 'δ', font=blk, fill=(0, 0, 0))
    arrow([(640, cy), (730, cy)])
    d.text((672, cy - 22), 'r', font=blk, fill=(0, 0, 0))
    arrow([(690, cy), (690, cy + 130), (193, cy + 130), (193, cy + 18)])
    block(450, cy + 150, 640, cy + 210, ['To Workspace', 'simout'])
    arrow([(690, cy + 130), (690, cy + 180), (640, cy + 180)])
    # Status bar.
    d.rectangle([0, SCREEN[1] - 26, SCREEN[0], SCREEN[1]], fill=(240, 240, 240))
    d.text((10, SCREEN[1] - 21), 'Ready', font=font(UI_FONT, 12), fill=(50, 50, 50))
    d.text((440, SCREEN[1] - 21), '100%', font=font(UI_FONT, 12), fill=(50, 50, 50))
    d.text((820, SCREEN[1] - 21), 'VariableStepAuto', font=font(UI_FONT, 12), fill=(50, 50, 50))
    return image


def screen_terminal():
    image = Image.new('RGB', SCREEN, (48, 10, 36))
    d = ImageDraw.Draw(image)
    d.rectangle([0, 0, SCREEN[0], 30], fill=(44, 44, 44))
    title = 'jkreiss@room207: ~/ros2_ws'
    d.text((SCREEN[0] / 2 - text_width(font(UI_BOLD, 13), title) / 2, 8), title, font=font(UI_BOLD, 13), fill=(220, 220, 220))
    for i, c in enumerate(((120, 120, 120), (120, 120, 120), (233, 84, 32))):
        d.ellipse([SCREEN[0] - 80 + i * 24, 8, SCREEN[0] - 64 + i * 24, 24], fill=c)
    mono, bold = font(MONO_FONT, 15), font(MONO_BOLD, 15)
    white, green, blue, grey, yellow = (238, 238, 236), (138, 226, 52), (114, 159, 207), (170, 170, 170), (252, 233, 79)
    lines = [
        [('jkreiss@room207', green, bold), (':', white, mono), ('~/ros2_ws', blue, bold), ('$ ', white, mono),
         ('colcon build --symlink-install', white, mono)],
        [('Starting >>> ensem_room207', white, mono)],
        [('Finished <<< ensem_room207 [0.41s]', white, mono)],
        [('Summary: 9 packages finished [12.3s]', white, mono)],
        [('jkreiss@room207', green, bold), (':', white, mono), ('~/ros2_ws', blue, bold), ('$ ', white, mono),
         ('ros2 launch room207_limo demo_simu.launch.py', white, mono)],
        [('[INFO] [launch]: All log files can be found below ~/.ros/log', white, mono)],
        [('[INFO] [gzserver-1]: process started with pid [4211]', white, mono)],
        [('[INFO] [spawn_entity.py-4]: Spawn status: Successfully spawned entity [limo]', white, mono)],
        [('[robot_state_publisher-3] Got segment base_link', grey, mono)],
        [('[WARN] [rviz2]: Fixed frame [map] does not exist', yellow, mono)],
        [('jkreiss@room207', green, bold), (':', white, mono), ('~/ros2_ws', blue, bold), ('$ ', white, mono),
         ('ros2 topic hz /odom', white, mono)],
        [('average rate: 50.012', white, mono)],
        [('    min: 0.019s max: 0.021s std dev: 0.00041s window: 52', white, mono)],
        [('jkreiss@room207', green, bold), (':', white, mono), ('~/ros2_ws', blue, bold), ('$ ', white, mono),
         ('ros2 topic echo /cmd_vel --once', white, mono)],
        [('linear:', white, mono)],
        [('  x: 0.3', white, mono)],
        [('angular:', white, mono)],
        [('  z: 0.25', white, mono)],
        [('---', white, mono)],
        [('jkreiss@room207', green, bold), (':', white, mono), ('~/ros2_ws', blue, bold), ('$ ', white, mono),
         ('█', white, mono)],
    ]
    y = 42
    for line in lines:
        x = 10
        for text, colour, f in line:
            d.text((x, y), text, font=f, fill=colour)
            x += text_width(f, text)
        y += 28
    return image


def screen_gazebo(view):
    image = Image.new('RGB', SCREEN, (60, 60, 60))
    d = ImageDraw.Draw(image)
    ui = font(UI_FONT, 12)
    d.rectangle([0, 0, SCREEN[0], 26], fill=(66, 66, 66))
    x = 10
    for item in ('File', 'Edit', 'Camera', 'View', 'Window', 'Help'):
        d.text((x, 6), item, font=ui, fill=(230, 230, 230))
        x += text_width(ui, item) + 20
    d.rectangle([0, 26, SCREEN[0], 58], fill=(80, 80, 80))
    for i in range(14):
        d.rectangle([230 + i * 34, 31, 254 + i * 34, 53], fill=(120, 120, 120) if i % 4 else (200, 160, 60))
    # Left panel.
    d.rectangle([0, 58, 210, SCREEN[1] - 30], fill=(70, 70, 70))
    for i, tab in enumerate(('World', 'Insert', 'Layers')):
        d.rectangle([2 + i * 69, 60, 68 + i * 69, 82], fill=(95, 95, 95) if i == 0 else (75, 75, 75))
        d.text((12 + i * 69, 64), tab, font=ui, fill=(240, 240, 240))
    for i, item in enumerate(('GUI', 'Scene', 'Spherical Coordinates', 'Physics', 'Atmosphere', 'Wind',
                              'Models', '  ground_plane', '  room207_room', '  room207_track', '  limo',
                              'Lights', '  daylight')):
        d.text((14, 94 + i * 22), ('▸ ' if not item.startswith(' ') else '   ') + item.strip(), font=ui, fill=(225, 225, 225))
    # 3D view.
    view = view.convert('RGB').resize((SCREEN[0] - 212, SCREEN[1] - 30 - 58))
    image.paste(view, (212, 58))
    # Bottom bar.
    d.rectangle([0, SCREEN[1] - 30, SCREEN[0], SCREEN[1]], fill=(66, 66, 66))
    d.text((12, SCREEN[1] - 22), '▮▮   Real Time Factor: 0.98   Sim Time: 00 00:02:13.420   '
           'Real Time: 00 00:02:15.871   Iterations: 133420', font=ui, fill=(230, 230, 230))
    return image


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--gazebo-view', help='render of the room shown on the Gazebo screens')
    args = parser.parse_args()

    TEXTURE_DIR.mkdir(parents=True, exist_ok=True)
    bricks().save(TEXTURE_DIR / 'room207_bricks.png')
    floor_tiles().save(TEXTURE_DIR / 'room207_floor_tiles.png')
    whiteboard_bicycle().save(TEXTURE_DIR / 'room207_whiteboard_bicycle.png')
    whiteboard_state_space().save(TEXTURE_DIR / 'room207_whiteboard_state_space.png')
    screen_simulink().save(TEXTURE_DIR / 'room207_screen_simulink.png')
    screen_terminal().save(TEXTURE_DIR / 'room207_screen_terminal.png')
    if args.gazebo_view:
        screen_gazebo(Image.open(args.gazebo_view)).save(TEXTURE_DIR / 'room207_screen_gazebo.png')
    print(f'Textures written to {TEXTURE_DIR}')


if __name__ == '__main__':
    main()
