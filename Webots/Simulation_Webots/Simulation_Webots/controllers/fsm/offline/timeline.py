"""Timeline of an FSM log (fsm_log.csv from Webots or results/<scenario>.csv).

Usage:
  python timeline.py ../fsm_log.csv
  python timeline.py results/obstacle_leaves.csv --event 5 "obstacle appears" --event 10 "obstacle leaves"
Writes <log>.png next to the log.
"""
import argparse
import csv

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

COLOUR = {"IDLE": "#93c5fd", "FOLLOW": "#3b82f6", "WAIT": "#ef4444", "EMERGENCY": "#b91c1c",
          "SEARCH": "#fb923c", "NODE": "#f97316", "TURN": "#ea580c", "AVOID": "#c2410c"}
D_STOP, D_CLEAR, D_SLOW = 0.25, 0.35, 0.60


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log")
    ap.add_argument("--event", nargs=2, action="append", default=[], metavar=("T", "TEXT"))
    ap.add_argument("--title", default=None)
    a = ap.parse_args()
    with open(a.log) as f:
        r = list(csv.DictReader(f))
    t = [float(x["t"]) for x in r]
    st = [x["state"] for x in r]
    v = [float(x["v"]) for x in r]
    w = [float(x["w"]) for x in r]
    d = [min(float(x["dist"]), 1.0) for x in r]

    plt.rcParams["font.family"] = ["Inter", "DejaVu Sans"]
    fig, (a0, a1, a2) = plt.subplots(3, 1, figsize=(11, 6.6), sharex=True,
                                     gridspec_kw=dict(height_ratios=[0.55, 1.4, 1.2], hspace=0.12))
    fig.subplots_adjust(left=0.09, right=0.91, top=0.86, bottom=0.09)

    # state band
    start = 0
    for i in range(1, len(st) + 1):
        if i == len(st) or st[i] != st[start]:
            t0, t1 = t[start], t[min(i, len(t) - 1)]
            a0.axvspan(t0, t1, color=COLOUR.get(st[start], "#9ca3af"), alpha=0.9, lw=0)
            if t1 - t0 > 0.07 * (t[-1] - t[0]):
                a0.text((t0 + t1) / 2, 0.5, st[start], ha="center", va="center", fontsize=11.5, weight="bold",
                        color="white" if st[start] in ("FOLLOW", "WAIT", "EMERGENCY") else "#111827")
            start = i
    a0.set_yticks([])
    a0.set_ylabel("state", rotation=0, ha="right", va="center", fontsize=11.5)
    for s in a0.spines.values():
        s.set_visible(False)

    a1.plot(t, d, color="#111827", lw=1.8)
    for y, name, colour in [(D_SLOW, "D_SLOW 0.60", "#6b7280"), (D_CLEAR, "D_CLEAR 0.35", "#6b7280"),
                            (D_STOP, "D_STOP 0.25", "#b91c1c")]:
        a1.axhline(y, color=colour, lw=1.1, ls=(0, (4, 3)))
        a1.text(t[-1], y + 0.015, name, ha="right", va="bottom", fontsize=10, color=colour)
    a1.set_ylim(0, 1.08)
    a1.set_ylabel("rangefinder, median (m)", fontsize=11.5)

    a2.plot(t, v, color="#1d4ed8", lw=1.8)
    a2.set_ylabel("v command (m/s)", fontsize=11.5, color="#1d4ed8")
    a2b = a2.twinx()
    if max(abs(x) for x in w) > 0.05:   # only draw w if the robot turned
        a2b.plot(t, w, color="#f97316", lw=1.2, alpha=0.8)
        a2b.set_ylabel("w command (rad/s)", fontsize=11.5, color="#c2410c")
    else:
        a2b.set_yticks([])
    a2.set_ylim(-0.01, 0.16)
    a2.set_xlabel("time (s)", fontsize=11.5)

    for ax in (a1, a2):
        ax.grid(axis="x", color="#e5e7eb")
        for s in ("top", "right"):
            ax.spines[s].set_visible(False)
    for s in ("top", "right"):
        a2b.spines[s].set_visible(False)

    # transitions
    for i in range(1, len(st)):
        if st[i] != st[i - 1]:
            for ax in (a0, a1, a2):
                ax.axvline(t[i], color="#374151", lw=0.9, ls=":")
            a0.text(t[i], 1.08, f"{t[i]:.1f} s", fontsize=10, color="#374151",
                    va="bottom", ha="center", transform=a0.get_xaxis_transform(), clip_on=False)
    for te, text in a.event:
        te = float(te)
        for ax in (a1, a2):
            ax.axvline(te, color="#7c3aed", lw=1.3)
        a2.text(te + 0.08, 0.15, text, fontsize=10.5, color="#6d28d9", va="top", weight="bold")
    fig.text(0.09, 0.95, a.title or f"Timeline: {a.log}", fontsize=14.5, weight="bold", va="center")
    out = a.log.rsplit(".", 1)[0] + ".png"
    fig.savefig(out, dpi=170)
    print(out)


if __name__ == "__main__":
    main()
