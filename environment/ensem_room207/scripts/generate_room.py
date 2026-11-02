#!/usr/bin/env python3
"""Generate the ENSEM Room 207 Gazebo model and world.

All room dimensions are defined below. Edit them after measuring the room,
then run this script again:

    python3 scripts/generate_room.py

Frame convention (same as the qcar_large_track world):
    - the QCar track is centred on the origin, 4.88 m along x, 6.20 m along y;
    - on the room plan (track at the top, classroom at the bottom):
        plan up    = world -x  (wall with the blinds, behind the track)
        plan down  = world +x  (classroom)
        plan left  = world -y  (whiteboard wall, door)
        plan right = world +y  (windows, banner walkway)

The values below come from a hand-drawn plan and photographs, scaled with
the track size. They are estimates and must be measured.
"""

import math
from pathlib import Path

PACKAGE_DIR = Path(__file__).resolve().parent.parent

# ============================================================
# Room dimensions (metres) - ESTIMATES, to be measured
# ============================================================

TRACK_X = 4.88
TRACK_Y = 6.20
TRACK_HEIGHT = 0.20  # The track mat lies on a raised platform

ROOM_X_MIN = -2.74   # North wall, behind the track (windows with blinds)
ROOM_X_MAX = 9.66    # South wall, end of the classroom
ROOM_Y_MIN = -3.12   # West wall (whiteboards, door)
ROOM_Y_MAX = 3.92    # East wall (windows)
CLASSROOM_X = 3.14   # Limit between the white track area and the yellow classroom

# U-shaped wall protruding into the room from the west wall, with the door
# at its end (the inside of the U is the entrance from the corridor).
NOTCH_X_MIN = 3.14
NOTCH_X_MAX = 5.52
NOTCH_Y_MAX = -1.07
DOOR_X = 4.33
DOOR_WIDTH = 1.60   # Double door (both leaves)
DOOR_HEIGHT = 2.10

WALL_HEIGHT = 2.90
WALL_THICKNESS = 0.20
BRICK_SKIN = 0.02   # Brick facing on the outside of the walls with windows
PAINT = 0.005       # Yellow paint layer on white wall cores

# Real size covered by one repetition of the textures (see generate_textures.py).
BRICK_TEXTURE_SIZE = 0.90
FLOOR_TEXTURE_SIZE = 0.80   # 2 x 2 tiles of 40 cm

# UrbanLoop banner along the track border, on the sides facing the two
# white walls with windows (north and east sides of the track).
BANNER_HEIGHT = 0.90
BANNER_THICKNESS = 0.03

# Back-to-back PC table pairs: (centre x, name). The pairs span
# y = PAIR_Y_MIN .. PAIR_Y_MAX and 1.40 m along x.
PC_PAIRS = [(6.00, 'center'), (8.86, 'south')]
PAIR_Y_MIN = -0.08
PAIR_Y_MAX = 1.32

# Pillars touching the north-east corner of each PC table pair
# (track and window side).
PILLAR_RADIUS = 0.08
PILLARS = [
    (x - 0.70 - PILLAR_RADIUS, PAIR_Y_MAX + PILLAR_RADIUS) for x, _ in PC_PAIRS
]

# Red traffic lights on the four island corners of the central crossing.
TRAFFIC_LIGHTS = [
    ('traffic_light_nw', -0.51, -0.26, 0.0),
    ('traffic_light_ne', 0.54, -0.26, math.pi / 2),
    ('traffic_light_sw', -0.51, -1.21, -math.pi / 2),
    ('traffic_light_se', 0.54, -1.21, math.pi),
]

# ============================================================
# Colours (ambient/diffuse RGBA)
# ============================================================

WHITE_WALL = '0.93 0.93 0.91 1'
YELLOW_WALL = '0.93 0.80 0.32 1'
WINDOW = '0.62 0.78 0.90 1'
BLIND = '0.45 0.47 0.48 1'
BANNER = '0.20 0.45 0.70 1'
CABINET = '0.88 0.87 0.83 1'
DOOR = '0.96 0.96 0.95 1'
DOOR_GLOW = '0.35 0.35 0.35 1'  # Keeps the white door white in the shade
DOOR_FRAME = '0.55 0.56 0.58 1'
METAL = '0.70 0.71 0.72 1'
CABINET_DOOR = '0.84 0.83 0.78 1'
PLINTH = '0.30 0.30 0.31 1'
ESTOP_BOX = '0.92 0.92 0.92 1'
PROJECTOR = '0.55 0.56 0.58 1'
WINDOW_FRAME = '0.96 0.96 0.95 1'
GLASS = '0.75 0.85 0.92 1'
GLASS_TRANSPARENCY = 0.70
RADIATOR = '0.95 0.95 0.94 1'
OUTSIDE_GROUND = '0.45 0.46 0.45 1'

# ============================================================
# Windows: (centre along the wall, width) - ESTIMATES
# ============================================================

WINDOW_SILL_Z = 1.00
WINDOW_TOP_Z = 2.50
# Positions read from the photos: the windows of the long east wall follow
# a regular ~2 m spacing on both sides of the beam at CLASSROOM_X, and the two
# windows of the south wall are on its west half (whiteboard side).
WINDOWS_NORTH = [(-1.8, 1.10), (0.1, 1.10), (2.0, 1.10)]      # with grey blinds
WINDOWS_EAST_TRACK = [(-1.6, 1.10), (0.4, 1.10), (2.4, 1.10)]
WINDOWS_EAST_CLASS = [(4.4, 1.10), (6.4, 1.10), (8.4, 1.10)]  # radiators below
WINDOWS_SOUTH = [(-2.2, 1.10), (-0.6, 1.10)]                  # radiators below
BLIND_LOWERED = 0.70  # Fraction of the north windows covered by the blinds



# Textured materials written to materials/scripts/room207_room.material:
# name -> (texture, (scale_u, scale_v), (scroll_u, scroll_v), lit).
# Unlit materials (screens, whiteboards) show the exact texture colours.
TEXTURED = {}


def textured(name, texture, scale=(1.0, 1.0), scroll=(0.0, 0.0), lit=True):
    TEXTURED[name] = (texture, scale, scroll, lit)
    return name


def material(rgba, emissive=None):
    if rgba in TEXTURED:
        return (
            '<material><script><uri>model://room207_room/materials/scripts</uri>'
            '<uri>model://room207_room/materials/textures</uri>'
            f'<name>{rgba}</name></script></material>'
        )
    emissive_sdf = f'<emissive>{emissive}</emissive>' if emissive else ''
    return (
        f'<material><ambient>{rgba}</ambient>'
        f'<diffuse>{rgba}</diffuse>{emissive_sdf}</material>'
    )


def parts_link(name, parts):
    """Link made of several boxes.

    Each part is (center, size, rgba, collision, transparency, emissive).
    """
    sdf = f'    <link name="{name}">\n'
    for i, (center, size, rgba, collision, transparency, emissive) in enumerate(parts):
        pose = f'<pose>{center[0]:.3f} {center[1]:.3f} {center[2]:.3f} 0 0 0</pose>'
        geometry = (
            f'<geometry><box><size>{size[0]:.3f} {size[1]:.3f} {size[2]:.3f}'
            '</size></box></geometry>'
        )
        if collision:
            sdf += f'      <collision name="collision_{i}">{pose}{geometry}</collision>\n'
        transparency_sdf = (
            f'<transparency>{transparency}</transparency>' if transparency else ''
        )
        sdf += (
            f'      <visual name="visual_{i}">{pose}{geometry}'
            f'{material(rgba, emissive)}{transparency_sdf}</visual>\n'
        )
    return sdf + '    </link>\n'


class Wall:
    """Straight wall with window openings.

    along: 'x' for a wall of constant y, 'y' for a wall of constant x.
    inner, outer: coordinates of the inner and outer faces.
    start, end: extent along the wall.
    """

    def __init__(self, name, along, inner, outer, start, end, rgba, windows=(),
                 bricks=False, skin_extend=(0.0, 0.0)):
        self.name = name
        self.bricks = bricks
        self.skin_extend = skin_extend
        self.along = along
        self.inner = inner
        self.outer = outer
        self.start = start
        self.end = end
        self.rgba = rgba
        self.windows = sorted(windows)

    def xy(self, u, v):
        """Wall coordinates (u along the wall, v across) to world x, y."""
        return (u, v) if self.along == 'x' else (v, u)

    def part(self, u0, u1, v0, v1, z0, z1, rgba, collision=True,
             transparency=0.0, emissive=None):
        x0, y0 = self.xy(min(u0, u1), min(v0, v1))
        x1, y1 = self.xy(max(u0, u1), max(v0, v1))
        center = ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2)
        size = (x1 - x0, y1 - y0, z1 - z0)
        return (center, size, rgba, collision, transparency, emissive)

    def links(self, blinds=False, radiators=False):
        h = WALL_HEIGHT
        v_in, v_out = self.inner, self.outer
        inward = 1.0 if v_in > v_out else -1.0
        z0, z1 = WINDOW_SILL_Z, WINDOW_TOP_Z
        pieces = []
        u = self.start
        for center, width in self.windows:
            a, b = center - width / 2, center + width / 2
            pieces += [(u, a, 0, h), (a, b, 0, z0), (a, b, z1, h)]
            u = b
        pieces.append((u, self.end, 0, h))

        parts = []
        for k, (u0, u1, pz0, pz1) in enumerate(pieces):
            v_core = v_out
            if self.bricks:
                # Skin slightly outside the wall so that it covers the corners.
                s0 = u0 - self.skin_extend[0] if u0 == self.start else u0
                s1 = u1 + self.skin_extend[1] if u1 == self.end else u1
                v_core = v_out + inward * BRICK_SKIN
                parts.append(self.part(s0, s1, v_out - inward * 0.005, v_core, pz0, pz1,
                                       self.brick_material(k, s0, s1, pz0, pz1, -inward)))
            if self.rgba == WHITE_WALL:
                parts.append(self.part(u0, u1, v_core, v_in, pz0, pz1, WHITE_WALL))
            else:
                parts.append(self.part(u0, u1, v_core, v_in - inward * PAINT, pz0, pz1, WHITE_WALL))
                parts.append(self.part(u0, u1, v_in - inward * PAINT, v_in, pz0, pz1, self.rgba))
        links = [parts_link(self.name, parts)]

        for i, (center, width) in enumerate(self.windows):
            links.append(self.window_link(f'{self.name}_window_{i}', center, width, inward, blinds, radiators))
        return links

    def brick_material(self, k, u0, u1, z0, z1, outward):
        """Brick material of one skin piece, scaled and shifted so that the
        courses of all the pieces of the facade line up."""
        # Direction of the texture u axis on the outer face of a Gazebo box.
        sign = outward if self.along == 'y' else -outward
        size = BRICK_TEXTURE_SIZE
        length, height = u1 - u0, z1 - z0
        # Ogre scales texture coordinates about 0.5, then adds the scroll.
        scroll_u = min(sign * u0, sign * u1) / size + 0.5 * length / size - 0.5
        scroll_v = -z1 / size + 0.5 * height / size - 0.5
        return textured(
            f'Room207/Bricks/{self.name}_{k}', 'room207_bricks.png',
            (size / length, size / height), (scroll_u % 1.0, scroll_v % 1.0),
        )

    def radiator_parts(self, center, inward, width=0.90):
        """Column radiator under a window: vertical sections between two
        headers, wall brackets, thermostatic valve and supply pipes."""
        v = self.inner + inward * 0.05          # Back of the radiator
        z0, z1 = 0.16, 0.66
        a, b = center - width / 2, center + width / 2
        parts = [
            # Top and bottom headers.
            self.part(a, b, v + inward * 0.02, v + inward * 0.06, z1 - 0.04, z1, RADIATOR),
            self.part(a, b, v + inward * 0.02, v + inward * 0.06, z0, z0 + 0.04, RADIATOR),
        ]
        pitch = 0.06
        for k in range(int(width / pitch)):
            u = a + pitch / 2 + k * pitch
            parts.append(self.part(u - 0.022, u + 0.022, v, v + inward * 0.08, z0 + 0.01, z1 - 0.01,
                                   RADIATOR, False))
        # Wall brackets.
        for u in (a + 0.12, b - 0.12):
            parts.append(self.part(u - 0.02, u + 0.02, self.inner, v, z1 - 0.10, z1 - 0.06, METAL, False))
        # Thermostatic valve on the top header and pipes down to the floor.
        parts.append(self.part(b, b + 0.05, v + inward * 0.025, v + inward * 0.055, z1 - 0.035, z1 - 0.005,
                               METAL, False))
        parts.append(self.part(b + 0.05, b + 0.10, v + inward * 0.015, v + inward * 0.065, z1 - 0.05, z1 + 0.06,
                               RADIATOR, False))
        for u, z_top in ((b + 0.06, z1 - 0.05), (a - 0.04, z0 + 0.02)):
            parts.append(self.part(u, u + 0.022, v + inward * 0.03, v + inward * 0.052, 0.0, z_top,
                                   METAL, False))
        parts.append(self.part(a - 0.04, a, v + inward * 0.03, v + inward * 0.052, z0 + 0.0, z0 + 0.025,
                               METAL, False))
        return parts

    def window_link(self, name, center, width, inward, blinds, radiators):
        a, b = center - width / 2, center + width / 2
        z0, z1 = WINDOW_SILL_Z, WINDOW_TOP_Z
        mid = (self.inner + self.outer) / 2
        f = 0.06       # Frame bar width
        d = 0.035      # Half depth of the frame
        parts = [
            # Frame: sides, bottom, top and central mullion (two casements).
            self.part(a, a + f, mid - d, mid + d, z0, z1, WINDOW_FRAME, False),
            self.part(b - f, b, mid - d, mid + d, z0, z1, WINDOW_FRAME, False),
            self.part(a, b, mid - d, mid + d, z0, z0 + f, WINDOW_FRAME, False),
            self.part(a, b, mid - d, mid + d, z1 - f, z1, WINDOW_FRAME, False),
            self.part(center - f / 2, center + f / 2, mid - d, mid + d, z0, z1, WINDOW_FRAME, False),
            # Handle.
            self.part(center + f / 2, center + f / 2 + 0.02, self.inner, self.inner + inward * 0.04,
                      (z0 + z1) / 2 - 0.08, (z0 + z1) / 2 + 0.08, WINDOW_FRAME, False),
            # Glass.
            self.part(a + f, b - f, mid - 0.005, mid + 0.005, z0 + f, z1 - f, GLASS, False, GLASS_TRANSPARENCY),
            # Inner sill.
            self.part(a - 0.05, b + 0.05, self.inner, self.inner + inward * 0.15, z0 - 0.03, z0, WINDOW_FRAME, False),
        ]
        if blinds:
            blind_bottom = z1 - BLIND_LOWERED * (z1 - z0)
            parts.append(self.part(a, b, self.inner + inward * 0.02, self.inner + inward * 0.03,
                                   blind_bottom, z1, BLIND, False))
            parts.append(self.part(a - 0.03, b + 0.03, self.inner, self.inner + inward * 0.10,
                                   z1, z1 + 0.10, BLIND, False))
        if radiators:
            parts += self.radiator_parts(center, inward)
        return parts_link(name, parts)


def box(name, center, size, rgba, collision=True):
    """Return an SDF link with a static box."""
    x, y, z = center
    sx, sy, sz = size
    geometry = f'<geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>'
    collision_sdf = (
        f'<collision name="collision">{geometry}</collision>' if collision else ''
    )
    return (
        f'    <link name="{name}">\n'
        f'      <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose>\n'
        f'      {collision_sdf}\n'
        f'      <visual name="visual">{geometry}{material(rgba)}</visual>\n'
        f'    </link>\n'
    )


def box_between(name, x_min, x_max, y_min, y_max, z_min, z_max, rgba, collision=True):
    return box(
        name,
        ((x_min + x_max) / 2, (y_min + y_max) / 2, (z_min + z_max) / 2),
        (x_max - x_min, y_max - y_min, z_max - z_min),
        rgba,
        collision,
    )


def cylinder(name, center, radius, length, rgba):
    x, y, z = center
    geometry = (
        f'<geometry><cylinder><radius>{radius:.3f}</radius>'
        f'<length>{length:.3f}</length></cylinder></geometry>'
    )
    return (
        f'    <link name="{name}">\n'
        f'      <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose>\n'
        f'      <collision name="collision">{geometry}</collision>\n'
        f'      <visual name="visual">{geometry}{material(rgba)}</visual>\n'
        f'    </link>\n'
    )


def panel_on_y_wall(name, y_wall, x, z, width, height, rgba):
    """Thin visual panel (window, board...) on a wall of constant y."""
    offset = -0.01 if y_wall > 0 else 0.01
    return box(name, (x, y_wall + offset, z), (width, 0.02, height), rgba, False)


def panel_on_x_wall(name, x_wall, y, z, width, height, rgba):
    """Thin visual panel on a wall of constant x."""
    offset = -0.01 if x_wall > 0 else 0.01
    return box(name, (x_wall + offset, y, z), (0.02, width, height), rgba, False)


def entrance_links():
    """U-shaped entrance: two side walls and an end wall with a double door."""
    t = WALL_THICKNESS
    h = WALL_HEIGHT
    f = 0.06                      # Door frame width
    y_out, y_in = NOTCH_Y_MAX - t, NOTCH_Y_MAX
    a, b = DOOR_X - DOOR_WIDTH / 2, DOOR_X + DOOR_WIDTH / 2
    top = DOOR_HEIGHT

    def part(x0, x1, y0, y1, z0, z1, rgba, collision=True, transparency=0.0, emissive=None):
        return (((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2),
                (x1 - x0, y1 - y0, z1 - z0), rgba, collision, transparency, emissive)

    walls = [
        part(NOTCH_X_MIN, NOTCH_X_MIN + t, ROOM_Y_MIN - t, y_in, 0, h, WHITE_WALL),
        part(NOTCH_X_MAX - t, NOTCH_X_MAX, ROOM_Y_MIN - t, y_in, 0, h, WHITE_WALL),
        part(NOTCH_X_MIN + t, a - f, y_out, y_in, 0, h, WHITE_WALL),
        part(b + f, NOTCH_X_MAX - t, y_out, y_in, 0, h, WHITE_WALL),
        part(a - f, b + f, y_out, y_in, top + f, h, WHITE_WALL),
    ]

    # Yellow paint of the classroom continues on the side of the entrance
    # facing the classroom and on the door wall, up to the corner after the door.
    p = PAINT
    walls += [
        part(NOTCH_X_MAX, NOTCH_X_MAX + p, ROOM_Y_MIN, y_in + p, 0, h, YELLOW_WALL, False),
        part(NOTCH_X_MIN, a - f, y_in, y_in + p, 0, h, YELLOW_WALL, False),
        part(b + f, NOTCH_X_MAX, y_in, y_in + p, 0, h, YELLOW_WALL, False),
        part(a - f, b + f, y_in, y_in + p, top + f, h, YELLOW_WALL, False),
    ]

    # Frame (slightly proud of both wall faces).
    e = 0.015
    door = [
        part(a - f, a, y_out - e, y_in + e, 0, top + f, DOOR_FRAME),
        part(b, b + f, y_out - e, y_in + e, 0, top + f, DOOR_FRAME),
        part(a, b, y_out - e, y_in + e, top, top + f, DOOR_FRAME),
    ]
    # Two leaves, closed, hinged on the frame; handles near the central gap.
    gap = 0.006
    d = 0.022                     # Half thickness of a leaf
    mid = (y_out + y_in) / 2
    for x0, x1, inner in ((a, DOOR_X - gap / 2, DOOR_X - gap / 2),
                          (DOOR_X + gap / 2, b, DOOR_X + gap / 2)):
        side = 1 if inner == x0 else -1          # From the gap towards the hinge
        door.append(part(x0, x1, mid - d, mid + d, 0.005, top - 0.005, DOOR, emissive=DOOR_GLOW))
        # Vision panel and kick plate on both faces.
        g0 = inner + side * 0.18
        g1 = inner + side * 0.33
        for face in (-1, 1):
            y_face = mid + face * d
            y_proud = y_face + face * 0.003
            door.append(part(min(g0, g1), max(g0, g1), min(y_face, y_proud), max(y_face, y_proud),
                             1.05, 1.90, GLASS, False, 0.3))
            door.append(part(x0 + 0.03, x1 - 0.03, min(y_face, y_proud), max(y_face, y_proud),
                             0.03, 0.28, METAL, False))
            # Lever handle: back plate and lever pointing to the hinge.
            hx = inner + side * 0.07
            y_lever = y_face + face * 0.055
            door.append(part(hx - 0.02, hx + 0.02, min(y_face, y_proud + face * 0.007),
                             max(y_face, y_proud + face * 0.007), 0.95, 1.15, METAL, False))
            door.append(part(min(hx, hx + side * 0.13), max(hx, hx + side * 0.13),
                             min(y_lever, y_lever - face * 0.02), max(y_lever, y_lever - face * 0.02),
                             1.04, 1.06, METAL, False))
            door.append(part(hx - 0.01, hx + 0.01, min(y_face, y_lever), max(y_face, y_lever),
                             1.04, 1.06, METAL, False))
    return [parts_link('entrance_walls', walls), parts_link('entrance_door', door)]


def drawer_cabinet_link(name, x_back, y_min, y_max, depth, height, drawers):
    """Tall drawer cabinet against a wall of constant x, front facing -x."""
    x_front = x_back - depth
    plinth = 0.08
    top = 0.02
    gap = 0.006
    y_mid = (y_min + y_max) / 2
    parts = [
        # Recessed plinth, body and top.
        (((x_front + 0.03 + x_back) / 2, y_mid, plinth / 2),
         (depth - 0.03, y_max - y_min - 0.02, plinth), PLINTH, True, 0.0, None),
        (((x_front + 0.02 + x_back) / 2, y_mid, (plinth + height - top) / 2),
         (depth - 0.02, y_max - y_min, height - top - plinth), CABINET, True, 0.0, None),
        (((x_front - 0.01 + x_back) / 2, y_mid, height - top / 2),
         (depth + 0.01, y_max - y_min + 0.02, top), CABINET, True, 0.0, None),
    ]
    pitch = (height - top - plinth) / drawers
    for i in range(drawers):
        z0 = plinth + i * pitch + gap / 2
        z1 = plinth + (i + 1) * pitch - gap / 2
        zc = (z0 + z1) / 2
        # Drawer front, label holder and handle.
        parts.append(((x_front + 0.01, y_mid, zc),
                      (0.02, y_max - y_min - 0.02, z1 - z0), CABINET_DOOR, False, 0.0, None))
        parts.append(((x_front - 0.002, y_mid, zc + pitch * 0.18),
                      (0.004, 0.10, pitch * 0.22), METAL, False, 0.0, None))
        parts.append(((x_front - 0.012, y_mid, zc - pitch * 0.12),
                      (0.02, 0.12, 0.015), METAL, False, 0.0, None))
    return parts_link(name, parts)


def whiteboard_link(name, x, z, width, height, texture):
    """Whiteboard on the west wall (facing +y) with an aluminium frame and a
    marker tray. The drawing is the texture of the board face."""
    y = ROOM_Y_MIN
    w, h = width / 2, height / 2
    f = 0.025   # Frame width

    def part(x0, x1, y0, y1, z0, z1, rgba):
        return (((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2),
                (x1 - x0, y1 - y0, z1 - z0), rgba, False, 0.0, None)

    board = textured(f'Room207/{name}', texture, lit=False)
    parts = [
        part(x - w, x + w, y, y + 0.02, z - h, z + h, board),
        part(x - w - f, x + w + f, y, y + 0.03, z + h, z + h + f, METAL),
        part(x - w - f, x + w + f, y, y + 0.03, z - h - f, z - h, METAL),
        part(x - w - f, x - w, y, y + 0.03, z - h, z + h, METAL),
        part(x + w, x + w + f, y, y + 0.03, z - h, z + h, METAL),
        # Marker tray.
        part(x - w * 0.6, x + w * 0.6, y, y + 0.08, z - h - f - 0.03, z - h - f, METAL),
    ]
    return parts_link(name, parts)


# Image shown by each workstation screen, in turn.
SCREEN_IMAGES = ['simulink', 'terminal', 'gazebo']


def screen_links():
    """Images on the workstation screens (Simulink, terminal, Gazebo).

    They are part of the room model: the workstation model is shared, so
    each screen image is placed in front of the screen of its workstation.
    """
    links = []
    workstations = [item for item in furniture() if item[0] == 'room207_workstation']
    for i, (_, name, x, y, yaw) in enumerate(workstations):
        image = SCREEN_IMAGES[i % len(SCREEN_IMAGES)]
        material_name = textured(f'Room207/Screen_{image}', f'room207_screen_{image}.png', lit=False)
        # Front face of the screen of room207_workstation: local x = -0.205, z = 1.05.
        sx, sy = local_to_world(x, y, yaw, -0.2035, 0.0)
        geometry = '<geometry><box><size>0.002 0.530 0.330</size></box></geometry>'
        links.append(
            f'    <link name="screen_{name}">\n'
            f'      <pose>{sx:.4f} {sy:.4f} 1.050 0 0 {yaw:.4f}</pose>\n'
            f'      <visual name="visual">{geometry}{material(material_name)}</visual>\n'
            '    </link>\n'
        )
    return links


def outside_links():
    """Ground seen through the windows."""
    return [box_between('outside_ground', -14, 22, -16, 16, -0.06, -0.03, OUTSIDE_GROUND, False)]


def room_model():
    t = WALL_THICKNESS
    h = WALL_HEIGHT
    links = []

    # Floor (slightly below the track to avoid visual flickering).
    # The top face of a box maps the texture u axis on y and v on x.
    floor_tiles = textured(
        'Room207/FloorTiles', 'room207_floor_tiles.png',
        (FLOOR_TEXTURE_SIZE / (ROOM_Y_MAX - ROOM_Y_MIN), FLOOR_TEXTURE_SIZE / (ROOM_X_MAX - ROOM_X_MIN)),
    )
    links.append(box_between(
        'floor', ROOM_X_MIN, ROOM_X_MAX, ROOM_Y_MIN, ROOM_Y_MAX, -0.025, -0.005, floor_tiles
    ))

    # North wall (windows with blinds) and south wall (two windows).
    links += Wall('wall_north', 'y', ROOM_X_MIN, ROOM_X_MIN - t, ROOM_Y_MIN - t, ROOM_Y_MAX + t,
                  WHITE_WALL, WINDOWS_NORTH, bricks=True).links(blinds=True)
    links += Wall('wall_south', 'y', ROOM_X_MAX, ROOM_X_MAX + t, ROOM_Y_MIN - t, ROOM_Y_MAX + t,
                  YELLOW_WALL, WINDOWS_SOUTH, bricks=True).links(radiators=True)

    # East wall with windows: white around the track, yellow in the classroom.
    # The three walls with windows are made of bricks on the outside.
    links += Wall('wall_east_track', 'x', ROOM_Y_MAX, ROOM_Y_MAX + t, ROOM_X_MIN, CLASSROOM_X,
                  WHITE_WALL, WINDOWS_EAST_TRACK, bricks=True, skin_extend=(t, 0)).links()
    links += Wall('wall_east_class', 'x', ROOM_Y_MAX, ROOM_Y_MAX + t, CLASSROOM_X, ROOM_X_MAX,
                  YELLOW_WALL, WINDOWS_EAST_CLASS, bricks=True, skin_extend=(0, t)).links(radiators=True)

    # West wall, interrupted by the U-shaped entrance with the door.
    links.append(box_between('wall_west_track', ROOM_X_MIN, NOTCH_X_MIN, ROOM_Y_MIN - t, ROOM_Y_MIN, 0, h, WHITE_WALL))
    # White outside (corridor), yellow paint inside the classroom.
    links.append(box_between('wall_west_class', NOTCH_X_MAX, ROOM_X_MAX, ROOM_Y_MIN - t, ROOM_Y_MIN - PAINT, 0, h, WHITE_WALL))
    links.append(box_between('wall_west_class_paint', NOTCH_X_MAX, ROOM_X_MAX, ROOM_Y_MIN - PAINT, ROOM_Y_MIN, 0, h, YELLOW_WALL))
    links += entrance_links()

    # Whiteboards on the west wall (track area and classroom).
    links.append(whiteboard_link('whiteboard_track', -0.10, 1.50, 2.40, 1.10, 'room207_whiteboard_bicycle.png'))
    links.append(whiteboard_link('whiteboard_class', 7.70, 1.50, 1.60, 1.10, 'room207_whiteboard_state_space.png'))
    links.append(panel_on_y_wall('estop_box', ROOM_Y_MIN, 8.75, 1.40, 0.45, 0.70, ESTOP_BOX))

    # Tall drawer cabinet against the entrance wall, facing the track.
    links.append(drawer_cabinet_link('drawer_cabinet', NOTCH_X_MIN, NOTCH_Y_MAX - 0.55, NOTCH_Y_MAX - 0.05,
                                     0.50, 1.90, 10))

    # UrbanLoop banner attached to the track border, on the two sides facing
    # the white walls with windows (north and east sides of the track).
    b = BANNER_THICKNESS
    tx, ty = TRACK_X / 2, TRACK_Y / 2
    links.append(box_between('banner_north', -tx - b, -tx, -ty, ty + b, 0, BANNER_HEIGHT, BANNER))
    links.append(box_between('banner_east', -tx - b, tx, ty, ty + b, 0, BANNER_HEIGHT, BANNER))

    # Pillars of the classroom.
    for i, (x, y) in enumerate(PILLARS):
        links.append(cylinder(f'pillar_{i}', (x, y, h / 2), PILLAR_RADIUS, h, WHITE_WALL))

    # Ceiling projector in front of the classroom whiteboard (hanging rod and body).
    links.append(box_between('projector_rod', 7.68, 7.72, -1.42, -1.38, 2.45, h, PROJECTOR, False))
    links.append(box_between('projector', 7.55, 7.85, -1.55, -1.25, 2.33, 2.45, PROJECTOR, False))

    # Ceiling: drawn only from below (see materials/scripts/room207_room.material)
    # so that the Gazebo GUI camera still sees the room from the top.
    links.append(
        '    <link name="ceiling">\n'
        f'      <pose>{(ROOM_X_MIN + ROOM_X_MAX) / 2:.3f} {(ROOM_Y_MIN + ROOM_Y_MAX) / 2:.3f} {h:.3f} 0 0 0</pose>\n'
        '      <visual name="visual"><geometry><plane><normal>0 0 1</normal>'
        f'<size>{ROOM_X_MAX - ROOM_X_MIN:.3f} {ROOM_Y_MAX - ROOM_Y_MIN:.3f}</size></plane></geometry>'
        '<material><script><uri>model://room207_room/materials/scripts</uri>'
        '<name>Room207/Ceiling</name></script></material></visual>\n'
        '    </link>\n'
    )

    links += screen_links()
    links += outside_links()

    return (
        '<?xml version="1.0"?>\n'
        '<!-- Generated by scripts/generate_room.py: edit the script, not this file. -->\n'
        '<sdf version="1.7">\n'
        '  <model name="room207_room">\n'
        '    <static>true</static>\n'
        + ''.join(links)
        + '  </model>\n'
        '</sdf>\n'
    )


# ============================================================
# Furniture placement: (model, name, x, y, yaw[, z])
# ============================================================

def local_to_world(x, y, yaw, dx, dy):
    c, s = math.cos(yaw), math.sin(yaw)
    return x + c * dx - s * dy, y + s * dx + c * dy


def workstation(name, x, y, yaw, chairs=1):
    """Workstation (screen on its local -x side) with chairs pushed under
    the table on the user side, as if the room were empty."""
    items = [('room207_workstation', name, x, y, yaw)]
    offsets = [0.0] if chairs == 1 else [-0.40, 0.40]
    for i, dy in enumerate(offsets):
        cx, cy = local_to_world(x, y, yaw, 0.30, dy)
        items.append(('room207_chair', f'{name}_chair_{i}', cx, cy, yaw + math.pi))
    return items


def furniture():
    items = []
    # Workstation just behind the banner, between the track and the classroom.
    items += workstation('workstation_track', 2.97, 2.40, 0.0)
    # Two pairs of back-to-back workstations in the middle of the classroom.
    for x, name in PC_PAIRS:
        items += workstation(f'workstation_{name}_w', x, PAIR_Y_MIN + 0.35, -math.pi / 2, chairs=2)
        items += workstation(f'workstation_{name}_e', x, PAIR_Y_MAX - 0.35, math.pi / 2, chairs=2)
    # Workstations against the east wall.
    items += workstation('workstation_east_center', 6.00, ROOM_Y_MAX - 0.36, -math.pi / 2)
    items += workstation('workstation_east_south', 8.86, ROOM_Y_MAX - 0.36, -math.pi / 2)
    # Two LIMO transport boxes on the floor against the entrance wall.
    for i, y in enumerate((ROOM_Y_MIN + 0.33, ROOM_Y_MIN + 0.96)):
        items.append(('room207_limo_box', f'limo_box_{i}', NOTCH_X_MIN - 0.22, y, 0.0))
    # Small teacher desk (laptop for the projector) in the corner between the
    # classroom whiteboard wall and the entrance.
    items.append(('room207_teacher_desk', 'teacher_desk', NOTCH_X_MAX + 0.53, ROOM_Y_MIN + 0.33, math.pi))
    # Red traffic lights at the central crossing, on top of the track platform.
    for name, x, y, yaw in TRAFFIC_LIGHTS:
        items.append(('room207_traffic_light', name, x, y, yaw, TRACK_HEIGHT))
    return items


def daylight():
    """Soft daylight (replaces the default sun, which is too bright indoors).

    The room has no other light source: only the reflection-only lights at
    the windows create highlights on the track and the floor.
    """
    return (
        '    <light name="daylight" type="directional">\n'
        '      <cast_shadows>false</cast_shadows>\n'
        '      <diffuse>0.55 0.55 0.55 1</diffuse>\n'
        '      <specular>0 0 0 1</specular>\n'
        '      <direction>-0.4 0.3 -0.9</direction>\n'
        '    </light>\n'
    )


def window_reflections():
    """Reflection-only lights at the windows around the track.

    They have no diffuse component, so they do not light the room: they only
    create the specular reflection of the windows on the glossy track mat.
    Two lights per window approximate its vertical shape.
    """
    sources = []
    blind_bottom = WINDOW_TOP_Z - BLIND_LOWERED * (WINDOW_TOP_Z - WINDOW_SILL_Z)
    for y, _ in WINDOWS_NORTH:
        # Only the part below the blinds lets daylight in.
        for z in (WINDOW_SILL_Z + 0.12, (WINDOW_SILL_Z + blind_bottom) / 2 + 0.1):
            sources.append((ROOM_X_MIN + 0.05, y, z))
    for x, _ in WINDOWS_EAST_TRACK:
        for z in (WINDOW_SILL_Z + 0.35, WINDOW_TOP_Z - 0.35):
            sources.append((x, ROOM_Y_MAX - 0.05, z))
    sdf = ''
    for i, (x, y, z) in enumerate(sources):
        sdf += (
            f'    <light name="window_reflection_{i}" type="point">\n'
            f'      <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose>\n'
            '      <cast_shadows>false</cast_shadows>\n'
            '      <diffuse>0 0 0 1</diffuse>\n'
            '      <specular>0.30 0.31 0.32 1</specular>\n'
            '      <attenuation><range>10</range><constant>0.5</constant>'
            '<linear>0.05</linear><quadratic>0.0</quadratic></attenuation>\n'
            '    </light>\n'
        )
    return sdf


def world():
    includes = []
    for model, name, x, y, yaw, *z in furniture():
        z = z[0] if z else 0.0
        includes.append(
            '    <include>\n'
            f'      <uri>model://{model}</uri>\n'
            f'      <name>{name}</name>\n'
            f'      <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 {yaw:.4f}</pose>\n'
            '    </include>\n'
        )
    return (
        '<?xml version="1.0"?>\n'
        '<!-- Generated by scripts/generate_room.py: edit the script, not this file. -->\n'
        '<sdf version="1.7">\n'
        '  <world name="ensem_room207_world">\n'
        '\n'
        + daylight()
        + window_reflections()
        + '\n'
        '    <scene>\n'
        '      <ambient>0.62 0.62 0.62 1</ambient>\n'
        '      <background>0.8 0.85 0.9 1</background>\n'
        '      <shadows>false</shadows>\n'
        '    </scene>\n'
        '\n'
        '    <!-- QCar track on its raised platform, centred on the origin\n'
        '         (same x/y frame as qcar_large_track). -->\n'
        '    <include>\n'
        '      <uri>model://room207_track</uri>\n'
        '      <pose>0 0 0 0 0 0</pose>\n'
        '    </include>\n'
        '\n'
        '    <include>\n'
        '      <uri>model://room207_room</uri>\n'
        '      <pose>0 0 0 0 0 0</pose>\n'
        '    </include>\n'
        '\n'
        + ''.join(includes)
        + '\n'
        '    <!-- Initial view from the classroom, oriented like the room plan. -->\n'
        '    <gui fullscreen="0">\n'
        '      <!-- Hide the markers of the light sources in the Gazebo window. -->\n'
        '      <plugin name="hide_light_markers" filename="libensem_room207_hide_light_markers.so"/>\n'
        '      <camera name="user_camera">\n'
        '        <pose>14.0 0.4 9.5 0 0.68 3.1416</pose>\n'
        '      </camera>\n'
        '    </gui>\n'
        '\n'
        '  </world>\n'
        '</sdf>\n'
    )


def track_model():
    """Raised platform with the QCar track texture on top."""
    top = (
        '<geometry><plane><normal>0 0 1</normal>'
        f'<size>{TRACK_X} {TRACK_Y}</size></plane></geometry>'
    )
    return (
        '<?xml version="1.0"?>\n'
        '<!-- Generated by scripts/generate_room.py: edit the script, not this file. -->\n'
        '<sdf version="1.7">\n'
        '  <model name="room207_track">\n'
        '    <static>true</static>\n'
        + box('platform', (0, 0, TRACK_HEIGHT / 2), (TRACK_X, TRACK_Y, TRACK_HEIGHT), '0.82 0.82 0.80 1')
        + '    <link name="track">\n'
        f'      <pose>0 0 {TRACK_HEIGHT + 0.001:.3f} 0 0 0</pose>\n'
        f'      <visual name="visual">{top}\n'
        '        <material><script>\n'
        '          <uri>model://room207_track/materials/scripts</uri>\n'
        '          <uri>model://qcar_large_track/materials/textures</uri>\n'
        '          <name>Room207/TrackMap</name>\n'
        '        </script></material>\n'
        '      </visual>\n'
        '    </link>\n'
        '  </model>\n'
        '</sdf>\n'
    )


def room_materials():
    sdf = (
        '// Generated by scripts/generate_room.py: edit the script, not this file.\n'
        '\n'
        '// Ceiling seen only from below: the plane faces up and the culling is\n'
        '// inverted, so the Gazebo GUI camera above the room still sees inside.\n'
        '// The ceiling is lit by the lamps below it, approximated with emissive.\n'
        'material Room207/Ceiling\n'
        '{\n'
        '  technique\n  {\n    pass\n    {\n'
        '      cull_hardware anticlockwise\n'
        '      cull_software none\n'
        '      ambient 0.5 0.5 0.5\n'
        '      diffuse 0 0 0\n'
        '      emissive 0.55 0.55 0.54\n'
        '    }\n  }\n}\n'
    )
    for name, (texture, scale, scroll, lit) in TEXTURED.items():
        specular = '0.15 0.15 0.15 1 60' if name == 'Room207/FloorTiles' else '0 0 0 1 1'
        lighting = (
            '      ambient 1 1 1 1\n'
            '      diffuse 1 1 1 1\n'
            f'      specular {specular}\n'
        ) if lit else '      lighting off\n'
        sdf += (
            f'\nmaterial {name}\n'
            '{\n  technique\n  {\n    pass\n    {\n'
            + lighting +
            '      texture_unit\n      {\n'
            f'        texture {texture}\n'
            '        filtering anisotropic\n'
            '        max_anisotropy 8\n'
            f'        scale {scale[0]:.5f} {scale[1]:.5f}\n'
            f'        scroll {scroll[0]:.5f} {scroll[1]:.5f}\n'
            '      }\n    }\n  }\n}\n'
        )
    return sdf


def main():
    (PACKAGE_DIR / 'models' / 'room207_room' / 'model.sdf').write_text(room_model())
    (PACKAGE_DIR / 'models' / 'room207_room' / 'materials' / 'scripts' / 'room207_room.material').write_text(
        room_materials())
    (PACKAGE_DIR / 'models' / 'room207_track' / 'model.sdf').write_text(track_model())
    (PACKAGE_DIR / 'worlds' / 'ensem_room207.world').write_text(world())
    print('Room 207 model and world generated.')


if __name__ == '__main__':
    main()
