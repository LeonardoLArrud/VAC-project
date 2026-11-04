"""Builds the 5x5 grid world of "FSM única e grid 5x5" (step 0): 5 x 5 nodes 0.50 m apart, 1.9 cm white tape.

Run from this folder:  uv run --with numpy --with pillow make_grid_5x5.py
Writes the floor textures and, for each world below, <world>.wbt (controllers fsm and grid_pose, launched
by Webots) and <world>_extern.wbt (both <extern>; from the VM: ./run_extern.sh fsm Cherokey_4W_v3 and
./run_extern.sh grid_pose grid_pose):
- grid_5x5: the clean grid;
- grid_5x5_box: plus a 10 x 10 x 15 cm box in the middle of edge (2, 3)-(3, 3), on the route of MISSION((0, 0), E, (4, 3));
- grid_5x5_reflexos: plus floor reflections (white spots) near the lines and beyond the border T's.
Node (i, j), i east and j north, 0..4, is at world ((i - 2) G, (j - 2) G): the grid is centred on the origin.
The lines stop at the border nodes (T on the sides, L in the corners), like grid_track() of the offline test.
The robot (DEF ST5) starts on node (0, 0) facing east. The grid is symmetric, so the image orientation
on the floor does not matter.
"""
import numpy as np
from PIL import Image

N, G, TAPE = 5, 0.50, 0.019      # nodes per side, m between nodes, m tape width
SIZE, PX = 3.0, 1000             # m floor side, texture px per m (1 mm per px)
FLOOR, LINE = 40, 235            # grey levels (the follower's THRESHOLD is 150)


def texture():
    n = int(SIZE * PX)
    img = np.full((n, n), FLOOR, np.uint8)
    half, ends = TAPE / 2, (N - 1) / 2 * G           # lines from -ends to +ends, tape included
    lo, hi = round((SIZE / 2 - ends - half) * PX), round((SIZE / 2 + ends + half) * PX)
    for k in range(N):
        c = SIZE / 2 + (k - (N - 1) / 2) * G         # line position, m from the image border
        a, b = round((c - half) * PX), round((c + half) * PX)
        img[a:b, lo:hi] = LINE                       # horizontal line
        img[lo:hi, a:b] = LINE                       # vertical line
    return img


def reflections(img, seed=1, n=3000):
    """White spots of 2 to 8 mm, 1.5 to 6 cm from the lines, and 8 mm ones on the axis 5 to 14 cm beyond
    every border T (where a false S would be)."""
    rng, img, ends = np.random.default_rng(seed), img.copy(), (N - 1) / 2 * G
    lines = np.arange(N) * G - ends
    spots = []
    for _ in range(n):
        along, off = rng.uniform(-ends, ends), rng.choice(lines) + rng.choice([-1, 1]) * rng.uniform(0.015, 0.06)
        spots.append(((along, off) if rng.random() < 0.5 else (off, along), rng.uniform(0.002, 0.008)))
    for c in lines[1:-1]:
        for b in (0.05, 0.08, 0.11, 0.14):
            spots += [(p, 0.008) for p in ((ends + b, c), (-ends - b, c), (c, ends + b), (c, -ends - b))]
    for (x, y), size in spots:
        u, v, k = round((x + SIZE / 2) * PX), round((SIZE / 2 - y) * PX), max(1, round(size * PX))
        img[v : v + k, u : u + k] = LINE
    return img


BOX = """DEF BOX Solid {{
  translation {x} {y} 0.025
  children [ Shape {{ appearance PBRAppearance {{ baseColor 0.85 0.35 0.1 roughness 1 metalness 0 }}
                     geometry DEF BOX_GEOMETRY Box {{ size 0.1 0.1 0.15 }} }} ]
  name "box"
  boundingObject USE BOX_GEOMETRY
}}
"""
WORLDS = {  # name: (texture, extra nodes)
    "grid_5x5": ("grid_5x5.png", ""),
    "grid_5x5_box": ("grid_5x5.png", BOX.format(x=0.5 * G, y=G)),   # middle of edge (2, 3)-(3, 3)
    "grid_5x5_reflexos": ("grid_5x5_reflexos.png", ""),
}

WORLD = """#VRML_SIM R2025a utf8
# 5 x 5 grid, 0.50 m between nodes, 1.9 cm tape (step 0 of "FSM única e grid 5x5"). Made by make_grid_5x5.py.
# Node (i, j) is at ((i - 2) * 0.5, (j - 2) * 0.5) m, i east, j north. Robot on node (0, 0) facing east.

EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2023b/projects/objects/backgrounds/protos/TexturedBackground.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2023b/projects/objects/backgrounds/protos/TexturedBackgroundLight.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2023b/projects/objects/floors/protos/RectangleArena.proto"
EXTERNPROTO "../protos/Cherokey_4W_v3.proto"
EXTERNPROTO "../protos/PiCamera.proto"

WorldInfo {{
}}
Viewpoint {{
  orientation 0.25 -0.25 -0.93 4.65
  position 0 -3.6 3.2
}}
TexturedBackground {{
}}
TexturedBackgroundLight {{
}}
RectangleArena {{
  translation 0 0 -0.05
  floorSize {size} {size}
  floorTileSize {size} {size}
  floorAppearance Appearance {{
    texture ImageTexture {{
      url [
        "{texture}"
      ]
    }}
  }}
}}
DEF ST5 Cherokey_4W_v3 {{
  translation {x0} {y0} 0.02
  rotation 0 0 1 0
  controller "{robot}"
  camera [
    PiCamera {{
      rotation 0 1 0 0.5235996938995747
    }}
  ]
}}
Robot {{
  name "grid_pose"
  supervisor TRUE
  controller "{pose}"
}}
{extra}"""

if __name__ == "__main__":
    clean = texture()
    Image.fromarray(clean).save("grid_5x5.png", optimize=True)
    Image.fromarray(reflections(clean)).save("grid_5x5_reflexos.png", optimize=True)
    start = -(N - 1) / 2 * G                         # node (0, 0)
    for world, (tex, extra) in WORLDS.items():
        for name, robot, pose in ((world, "fsm", "grid_pose"), (f"{world}_extern", "<extern>", "<extern>")):
            with open(f"{name}.wbt", "w") as f:
                f.write(WORLD.format(size=SIZE, x0=start, y0=start, robot=robot, pose=pose, texture=tex, extra=extra))
            print("wrote", f"{name}.wbt")
