"""Interactive visual test of the graph logic: track, Dijkstra, obstacles,
cruising without a goal, sharp turns on the 8 and maneuvers.

Run from the project root (needs a Python with tkinter, e.g. the team's .venv):
    ../.venv/bin/python visual_test.py
"""

import os
import sys
import tkinter as tk

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "Robo_FSM"))   # graph/ lives in Robo_FSM/

from graph.maneuver import maneuver
from graph.simulation import Simulation, edges, nearest_edge, nearest_node
from graph.track import figure_eight, grid

TRACKS = {
    "Figure 8": lambda: Simulation(figure_eight(), "L0"),
    "5x5 grid": lambda: Simulation(grid(5), (1, 1)),
}
CLICK_MODES = [
    ("goal", "set the goal (node)"),
    ("static", "static obstacle on/off (edge)"),
    ("dynamic", "dynamic obstacle on/off (edge)"),
    ("car", "move the car = new start (node)"),
]
COLORS = {
    "free": "#c8c8c8",
    "static": "#d63031",
    "timed": "#f39c12",
    "hidden": "#8e44ad",
    "path": "#2f6fde",
    "trail": "#7bc67b",
    "node": "#555555",
    "to_visit": "#74b9ff",
    "goal": "#f1c40f",
    "car": "#27ae60",
}
LEGEND = [
    ("free", "free edge"),
    ("static", "static obstacle (known)"),
    ("timed", "blocked after the sensor saw it (countdown)"),
    ("hidden", "dynamic obstacle, not seen yet"),
    ("path", "planned path (to the goal or next node to visit)"),
    ("to_visit", "node still to visit (after an obstacle, no goal)"),
    ("trail", "where the car has been"),
    ("goal", "goal"),
    ("car", "car + arrow to the next node"),
]

W, H, MARGIN = 640, 540, 60
TICK_MS = 250  # screen refresh
STEP_TICKS = 3  # when playing: one car step every 3 refreshes
CLICK_PX = 25  # how close a click must be to a node / edge
MONO = ("Menlo", 11)
BOLD = ("Helvetica", 13, "bold")


class App:
    def __init__(self, root):
        self.root = root
        root.title("Graph visual test")
        self.canvas = tk.Canvas(root, width=W, height=H, bg="white", highlightthickness=0)
        self.canvas.grid(row=0, column=0)
        self.canvas.bind("<Button-1>", self.on_click)

        panel = tk.Frame(root, padx=12, pady=12)
        panel.grid(row=0, column=1, sticky="n")

        tk.Label(panel, text="Track", font=BOLD).pack(anchor="w")
        self.track = tk.StringVar(value="Figure 8")
        for name in TRACKS:
            tk.Radiobutton(panel, text=name, value=name, variable=self.track,
                           command=self.reset).pack(anchor="w")

        tk.Label(panel, text="Click on the track to", font=BOLD).pack(anchor="w", pady=(10, 0))
        self.mode = tk.StringVar(value="goal")
        for value, text in CLICK_MODES:
            tk.Radiobutton(panel, text=text, value=value, variable=self.mode).pack(anchor="w")

        buttons = tk.Frame(panel)
        buttons.pack(anchor="w", pady=10)
        tk.Button(buttons, text="Step", command=self.step).pack(side="left")
        self.play_button = tk.Button(buttons, text="Play", width=5, command=self.toggle_play)
        self.play_button.pack(side="left")
        tk.Button(buttons, text="Clear goal", command=self.clear_goal).pack(side="left")
        tk.Button(buttons, text="Reset", command=self.reset).pack(side="left")

        self.info = tk.Label(panel, justify="left", anchor="w", font=MONO)
        self.info.pack(anchor="w")

        tk.Label(panel, text="Log", font=BOLD).pack(anchor="w", pady=(10, 0))
        self.log = tk.Text(panel, width=48, height=10, font=MONO, state="disabled")
        self.log.pack(anchor="w")

        legend = tk.Frame(panel)
        legend.pack(anchor="w", pady=(10, 0))
        for key, text in LEGEND:
            row = tk.Frame(legend)
            row.pack(anchor="w")
            tk.Label(row, text="■", fg=COLORS[key], font=BOLD).pack(side="left")
            tk.Label(row, text=text).pack(side="left")

        self.playing = False
        self.ticks = 0
        self.reset()
        self.tick()

    # ---------- actions ----------

    def reset(self):
        self.sim = TRACKS[self.track.get()]()
        self.shown_log = -1
        self.fit_to_canvas()
        self.playing = False
        self.play_button.config(text="Play")
        self.redraw()

    def step(self):
        self.sim.step()
        self.redraw()

    def toggle_play(self):
        self.playing = not self.playing
        self.play_button.config(text="Pause" if self.playing else "Play")

    def clear_goal(self):
        self.sim.set_goal(None)
        self.redraw()

    def on_click(self, event):
        x, y = self.to_world(event.x, event.y)
        g, mode = self.sim.graph, self.mode.get()
        limit = CLICK_PX / self.scale
        if mode in ("goal", "car"):
            node, dist = nearest_node(g, x, y)
            if dist <= limit:
                (self.sim.set_goal if mode == "goal" else self.sim.move_car)(node)
        else:
            (u, v), dist = nearest_edge(g, x, y)
            if dist <= limit:
                (self.sim.toggle_static if mode == "static" else self.sim.toggle_dynamic)(u, v)
        self.redraw()

    def tick(self):
        if self.playing:
            self.ticks += 1
            if self.ticks % STEP_TICKS == 0:
                self.sim.step()
        self.redraw()
        self.root.after(TICK_MS, self.tick)

    # ---------- coordinates ----------

    def fit_to_canvas(self):
        xs = [x for x, _ in self.sim.graph.pos.values()]
        ys = [y for _, y in self.sim.graph.pos.values()]
        self.minx, self.miny = min(xs), min(ys)
        width, height = max(xs) - self.minx, max(ys) - self.miny
        self.scale = min((W - 2 * MARGIN) / width, (H - 2 * MARGIN) / height)
        self.ox = (W - width * self.scale) / 2  # centers the drawing
        self.oy = (H - height * self.scale) / 2

    def to_screen(self, p):
        x, y = p  # y goes up in the graph, down on the screen
        return self.ox + (x - self.minx) * self.scale, H - self.oy - (y - self.miny) * self.scale

    def to_world(self, sx, sy):
        return self.minx + (sx - self.ox) / self.scale, self.miny + (H - self.oy - sy) / self.scale

    # ---------- drawing ----------

    def redraw(self):
        self.canvas.delete("all")
        self.draw_edges()
        self.draw_line(self.sim.nav.history[-12:], COLORS["trail"], 3)
        if len(self.sim.nav.path) > 1:
            self.draw_line(self.sim.nav.path, COLORS["path"], 4)
        self.draw_nodes()
        self.draw_car()
        self.update_info()
        self.update_log()

    def draw_edges(self):
        c, g = self.canvas, self.sim.graph
        for u, v in edges(g):
            e = frozenset((u, v))
            (x1, y1), (x2, y2) = self.to_screen(g.pos[u]), self.to_screen(g.pos[v])
            mx, my = (x1 + x2) / 2, (y1 + y2) / 2
            timed = g.is_blocked(u, v) and e not in self.sim.static
            color = COLORS["static"] if e in self.sim.static else COLORS["timed"] if timed else COLORS["free"]
            c.create_line(x1, y1, x2, y2, fill=color, width=7, capstyle="round")
            if timed:
                left = g.blocked_until[e] - g.clock()
                c.create_text(mx, my - 16, text=f"{left:.0f}s", fill=COLORS["timed"], font=BOLD)
            if e in self.sim.hidden:
                c.create_rectangle(mx - 9, my - 9, mx + 9, my + 9, fill=COLORS["hidden"], outline="")

    def draw_line(self, nodes, color, width):
        points = [xy for n in nodes for xy in self.to_screen(self.sim.graph.pos[n])]
        if len(points) >= 4:
            self.canvas.create_line(*points, fill=color, width=width, capstyle="round",
                                    joinstyle="round")

    def draw_nodes(self):
        c, g, nav = self.canvas, self.sim.graph, self.sim.nav
        names = {}  # position -> names (C1 and C2 share one spot)
        for n, p in g.pos.items():
            names.setdefault(p, []).append(n if isinstance(n, str) else f"{n[0]},{n[1]}")
        to_visit = {g.pos[n] for n in nav.to_visit}
        for p, labels in names.items():
            x, y = self.to_screen(p)
            fill = COLORS["to_visit"] if p in to_visit else COLORS["node"]
            c.create_oval(x - 8, y - 8, x + 8, y + 8, fill=fill, outline="")
            c.create_text(x + 18, y - 18, text="/".join(labels), anchor="w", font=MONO)
        if nav.goal is not None:
            x, y = self.to_screen(g.pos[nav.goal])
            c.create_oval(x - 16, y - 16, x + 16, y + 16, outline=COLORS["goal"], width=5)

    def draw_car(self):
        c, g, nav = self.canvas, self.sim.graph, self.sim.nav
        x, y = self.to_screen(self.sim.car_position())
        nxt = nav.next_node()
        if nxt is not None:
            nx, ny = self.to_screen(g.pos[nxt])
            if (nx, ny) != (x, y):
                c.create_line(x, y, x + 0.45 * (nx - x), y + 0.45 * (ny - y),
                              fill=COLORS["car"], width=5, arrow="last", arrowshape=(14, 16, 6))
        c.create_oval(x - 12, y - 12, x + 12, y + 12, fill=COLORS["car"], outline="white", width=2)

    def update_info(self):
        g, nav = self.sim.graph, self.sim.nav
        nxt = nav.next_node()
        if nav.returning_from is not None:
            move = f"turned around, driving back to {nav.current}"
        else:
            move = maneuver(g, nav.previous, nav.current, nxt) if nxt is not None else "-"
        lines = [
            f"goal        : {nav.goal if nav.goal is not None else 'none (cruising)'}",
            f"car at      : {nav.current}   (from {nav.previous})",
            f"next node   : {nxt if nxt is not None else 'STUCK'}",
            f"maneuver    : {move}",
            f"distance    : {nav.distance if nav.goal is not None else '-'}",
            f"sharp turns : {'allowed' if nav.allow_turns else 'not allowed'}",
            f"visit-all   : {f'{len(nav.to_visit)} nodes left' if nav.to_visit else 'off'}",
            f"history     : {' '.join(map(str, nav.history[-6:]))}",
        ]
        self.info.config(text="\n".join(lines))

    def update_log(self):
        if len(self.sim.log) == self.shown_log:
            return  # nothing new: don't reset the user's scroll position
        self.shown_log = len(self.sim.log)
        self.log.config(state="normal")
        self.log.delete("1.0", "end")
        self.log.insert("end", "\n".join(self.sim.log[-100:]))
        self.log.see("end")
        self.log.config(state="disabled")


if __name__ == "__main__":
    root = tk.Tk()
    App(root)
    root.mainloop()
