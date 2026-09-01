"""Visual vehicle builder. Students place thrusters; JSON is the only hardware file.

The pygame window is the primary UI. Headless tests import VehicleDraft and never
open a display.
"""

from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

from airbearing.spec import (
    SatelliteSpec,
    assert_vehicle_save_path,
    controllability_report,
    plus_frame_radial_warning,
    repo_root,
    save_vehicle,
    spec_from_dict,
    spec_to_dict,
    vehicles_dir,
)

TYPE_ALIASES = {
    "solenoid": "binary_solenoid",
    "binary_solenoid": "binary_solenoid",
    "pwm_fan": "pwm_fan",
    "fan": "pwm_fan",
    "continuous": "continuous",
}
TYPES = ("binary_solenoid", "pwm_fan", "continuous")


def _new_thruster(tid: str, pos, direc, F: float, typ: str, bidir: bool) -> dict[str, Any]:
    typ = TYPE_ALIASES.get(typ, typ)
    t: dict[str, Any] = {
        "id": tid,
        "position": [float(pos[0]), float(pos[1])],
        "force_direction": [float(direc[0]), float(direc[1])],
        "F_max": float(F),
        "type": typ,
    }
    if typ == "binary_solenoid":
        t["min_pulse_ms"] = 30
        t["deadman_ms"] = 100
    elif typ == "pwm_fan":
        t["tau"] = 0.18
        t["bidirectional"] = bool(bidir)
        t["duty_min"] = 0.0
        t["duty_max"] = 1.0
        t["deadman_ms"] = 100
    else:
        t["bidirectional"] = True
    return t


@dataclass
class VehicleDraft:
    """In-memory vehicle. Mutate here; save writes vehicles/<name>.json only."""

    data: dict[str, Any]
    selected: int | None = None
    path: Path | None = None

    @classmethod
    def blank(cls, name: str = "mine", table: float = 2.0, mass: float = 5.0) -> VehicleDraft:
        radius = 0.18
        data = {
            "name": name,
            "mass": mass,
            "Iz": mass * (radius ** 2) * 0.6,
            "com": [0.0, 0.0],
            "table_size": table,
            "hull_radius": radius * 1.15,
            "linear_damping": 0.05,
            "rotational_damping": 0.01,
            "control_dt": 0.05,
            "sim_dt": 0.01,
            "notes": "Drawn in python -m airbearing edit-vehicle. Calibrate F_max (docs/LAB.md).",
            "thrusters": [],
        }
        return cls(data=data)

    @classmethod
    def from_json(cls, path: str | Path) -> VehicleDraft:
        p = Path(path)
        data = json.loads(p.read_text())
        spec_from_dict(data)  # validate
        return cls(data=data, path=p)

    @classmethod
    def from_spec(cls, spec: SatelliteSpec) -> VehicleDraft:
        return cls(data=spec_to_dict(spec), path=spec.source_path)

    def spec(self) -> SatelliteSpec:
        return spec_from_dict(self.data, source=self.path)

    def next_id(self) -> str:
        used = {t["id"] for t in self.data["thrusters"]}
        i = 1
        while f"T{i}" in used:
            i += 1
        return f"T{i}"

    def add_thruster(
        self,
        pos=(0.18, 0.0),
        direc=(0.0, 1.0),
        F_max: float = 0.3,
        typ: str = "pwm_fan",
        bidirectional: bool = True,
        tid: str | None = None,
    ) -> int:
        tid = tid or self.next_id()
        self.data["thrusters"].append(_new_thruster(tid, pos, direc, F_max, typ, bidirectional))
        self.selected = len(self.data["thrusters"]) - 1
        return self.selected

    def move_thruster(self, index: int, pos) -> None:
        self.data["thrusters"][index]["position"] = [float(pos[0]), float(pos[1])]

    def delete_thruster(self, index: int | None = None) -> None:
        i = self.selected if index is None else index
        if i is None:
            return
        self.data["thrusters"].pop(i)
        self.selected = None if not self.data["thrusters"] else min(i, len(self.data["thrusters"]) - 1)

    def set_type(self, typ: str, index: int | None = None) -> None:
        i = self.selected if index is None else index
        if i is None:
            return
        old = self.data["thrusters"][i]
        bidir = bool(old.get("bidirectional", typ != "binary_solenoid"))
        self.data["thrusters"][i] = _new_thruster(
            old["id"], old["position"], old["force_direction"], old["F_max"], typ, bidir
        )

    def set_fmax(self, F: float, index: int | None = None) -> None:
        i = self.selected if index is None else index
        if i is None:
            return
        self.data["thrusters"][i]["F_max"] = float(max(1e-6, F))

    def set_bidirectional(self, flag: bool, index: int | None = None) -> None:
        i = self.selected if index is None else index
        if i is None:
            return
        t = self.data["thrusters"][i]
        if t["type"] == "binary_solenoid":
            return
        t["bidirectional"] = bool(flag)

    def rotate_direction(self, radians: float, index: int | None = None) -> None:
        i = self.selected if index is None else index
        if i is None:
            return
        d = self.data["thrusters"][i]["force_direction"]
        c, s = math.cos(radians), math.sin(radians)
        self.data["thrusters"][i]["force_direction"] = [c * d[0] - s * d[1], s * d[0] + c * d[1]]

    def set_mass(self, mass: float) -> None:
        self.data["mass"] = float(max(1e-6, mass))

    def set_Iz(self, Iz: float) -> None:
        self.data["Iz"] = float(max(1e-9, Iz))

    def set_com(self, xy) -> None:
        self.data["com"] = [float(xy[0]), float(xy[1])]

    def set_table_size(self, size: float) -> None:
        self.data["table_size"] = float(max(0.2, size))

    def hit_test(self, xy, radius: float = 0.04) -> int | None:
        best, best_d = None, radius
        for i, t in enumerate(self.data["thrusters"]):
            p = t["position"]
            d = math.hypot(p[0] - xy[0], p[1] - xy[1])
            if d < best_d:
                best, best_d = i, d
        return best

    def report(self) -> dict[str, Any]:
        if not self.data["thrusters"]:
            return {
                "full": False,
                "rank_B": 0,
                "limits": {},
                "warning": "No thrusters. Click the table to add one.",
                "n_thrusters": 0,
                "radial_plus": False,
            }
        spec = self.spec()
        r = controllability_report(spec)
        r["radial_warn"] = plus_frame_radial_warning(spec)
        return r

    def save(self, path: str | Path | None = None) -> Path:
        dest = Path(path) if path is not None else self.path
        if dest is None:
            dest = vehicles_dir() / f"{self.data['name']}.json"
        dest = assert_vehicle_save_path(dest, allow_any=False)
        out = save_vehicle(self.data, dest, allow_any=False)
        self.path = out
        return out


def one_screen_report(spec: SatelliteSpec) -> str:
    r = controllability_report(spec)
    lim = r.get("limits", {})
    lines = [
        f"vehicle  {spec.name}",
        f"file     {spec.source_path or '(memory)'}",
        f"mass     {spec.mass:.4g} kg     Iz {spec.Iz:.4g} kg m^2     COM {spec.com.tolist()}",
        f"table    {spec.table_size:.3g} m     hull {spec.hull_radius:.3g} m     n={spec.n_thrusters}",
        f"B rank   {r['rank_B']}/3     both-signs Fx/Fy/Mz: {'YES' if r['full'] else 'NO'}",
        (
            "wrench   "
            f"+Fx={lim.get('+Fx', 0):.3g}  -Fx={lim.get('-Fx', 0):.3g}  "
            f"+Fy={lim.get('+Fy', 0):.3g}  -Fy={lim.get('-Fy', 0):.3g}  "
            f"+Mz={lim.get('+Mz', 0):.3g}  -Mz={lim.get('-Mz', 0):.3g}"
        ),
    ]
    for t in spec.thrusters:
        lines.append(
            f"  {t.id:6s}  {t.type:16s}  pos=({t.position[0]:+.3f},{t.position[1]:+.3f})  "
            f"dir=({t.force_direction[0]:+.2f},{t.force_direction[1]:+.2f})  "
            f"F_max={t.F_max:.3g} N  bidir={t.bidirectional}"
        )
    if r.get("warning"):
        lines.append(f"WARNING  {r['warning']}")
    else:
        lines.append("ok       layout can produce ±Fx ±Fy ±Mz")
    return "\n".join(lines) + "\n"


def _try_pygame():
    try:
        import pygame
        return pygame
    except Exception:
        return None


def run_editor(
    path: str | Path | None = None,
    *,
    headless: bool = False,
    save_png: str | Path | None = None,
    frames: int = 1,
) -> VehicleDraft:
    """Open the pygame table (or a dummy surface). Tests pass headless=True."""
    if path:
        draft = VehicleDraft.from_json(path)
    else:
        draft = VehicleDraft.blank("mine")
        # start with an empty table so students click-to-add; wizard still exists
    if headless:
        os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
    pygame = _try_pygame()
    if pygame is None:
        if save_png:
            _matplotlib_layout(draft.spec() if draft.data["thrusters"] else None, draft, Path(save_png))
        return draft
    pygame.init()
    W, H = 980, 640
    screen = pygame.display.set_mode((W, H))
    pygame.display.set_caption("airbearing vehicle editor — JSON only under vehicles/")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("dejavusansmono,consolas,monospace", 16)
    small = pygame.font.SysFont("dejavusansmono,consolas,monospace", 13)
    dragging = False
    table_px = min(H - 40, 560)
    origin = (40 + table_px // 2, H // 2)

    def world_to_px(xy):
        table = float(draft.data["table_size"])
        s = table_px / table
        return int(origin[0] + xy[0] * s), int(origin[1] - xy[1] * s)

    def px_to_world(px, py):
        table = float(draft.data["table_size"])
        s = table_px / table
        return (px - origin[0]) / s, (origin[1] - py) / s

    running = True
    frame = 0
    while running:
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                running = False
            elif ev.type == pygame.KEYDOWN:
                if ev.key in (pygame.K_ESCAPE, pygame.K_q):
                    running = False
                elif ev.key == pygame.K_s:
                    try:
                        out = draft.save()
                        draft.data["notes"] = f"Saved {out.name}"
                    except Exception as exc:
                        draft.data["notes"] = f"SAVE REFUSED: {exc}"
                elif ev.key == pygame.K_DELETE or ev.key == pygame.K_BACKSPACE:
                    draft.delete_thruster()
                elif ev.key == pygame.K_1:
                    draft.set_type("solenoid")
                elif ev.key == pygame.K_2:
                    draft.set_type("pwm_fan")
                elif ev.key == pygame.K_3:
                    draft.set_type("continuous")
                elif ev.key == pygame.K_b:
                    if draft.selected is not None:
                        cur = bool(draft.data["thrusters"][draft.selected].get("bidirectional", False))
                        draft.set_bidirectional(not cur)
                elif ev.key == pygame.K_LEFTBRACKET:
                    if draft.selected is not None:
                        draft.set_fmax(draft.data["thrusters"][draft.selected]["F_max"] * 0.9)
                elif ev.key == pygame.K_RIGHTBRACKET:
                    if draft.selected is not None:
                        draft.set_fmax(draft.data["thrusters"][draft.selected]["F_max"] * 1.1)
                elif ev.key == pygame.K_r:
                    mods = pygame.key.get_mods()
                    draft.rotate_direction(-0.26 if mods & pygame.KMOD_SHIFT else 0.26)
                elif ev.key == pygame.K_COMMA:
                    draft.set_mass(draft.data["mass"] * 0.95)
                elif ev.key == pygame.K_PERIOD:
                    draft.set_mass(draft.data["mass"] * 1.05)
                elif ev.key == pygame.K_MINUS:
                    draft.set_Iz(draft.data["Iz"] * 0.95)
                elif ev.key == pygame.K_EQUALS:
                    draft.set_Iz(draft.data["Iz"] * 1.05)
                elif ev.key == pygame.K_LEFT:
                    draft.set_table_size(draft.data["table_size"] * 0.95)
                elif ev.key == pygame.K_RIGHT:
                    draft.set_table_size(draft.data["table_size"] * 1.05)
            elif ev.type == pygame.MOUSEBUTTONDOWN and ev.button == 1:
                wx, wy = px_to_world(*ev.pos)
                hit = draft.hit_test((wx, wy), radius=0.05)
                if hit is not None:
                    draft.selected = hit
                    dragging = True
                else:
                    # click on table adds a thruster pointing +y (tangent-friendly default)
                    if abs(wx) < draft.data["table_size"] / 2 and abs(wy) < draft.data["table_size"] / 2:
                        draft.add_thruster(pos=(wx, wy), direc=(0.0, 1.0))
                        dragging = True
            elif ev.type == pygame.MOUSEBUTTONUP and ev.button == 1:
                dragging = False
            elif ev.type == pygame.MOUSEMOTION and dragging and draft.selected is not None:
                wx, wy = px_to_world(*ev.pos)
                draft.move_thruster(draft.selected, (wx, wy))

        screen.fill((18, 22, 28))
        half = table_px // 2
        rect = pygame.Rect(origin[0] - half, origin[1] - half, table_px, table_px)
        pygame.draw.rect(screen, (40, 48, 58), rect)
        pygame.draw.rect(screen, (90, 110, 130), rect, 2)
        pygame.draw.line(screen, (50, 60, 72), (origin[0] - half, origin[1]), (origin[0] + half, origin[1]), 1)
        pygame.draw.line(screen, (50, 60, 72), (origin[0], origin[1] - half), (origin[0], origin[1] + half), 1)
        com_px = world_to_px(draft.data["com"])
        pygame.draw.circle(screen, (240, 200, 80), com_px, 6, 2)
        for i, t in enumerate(draft.data["thrusters"]):
            p = world_to_px(t["position"])
            col = (80, 200, 255) if i == draft.selected else (120, 170, 210)
            pygame.draw.circle(screen, col, p, 10)
            d = t["force_direction"]
            n = math.hypot(d[0], d[1]) or 1.0
            tip = world_to_px((t["position"][0] + 0.08 * d[0] / n, t["position"][1] + 0.08 * d[1] / n))
            pygame.draw.line(screen, (255, 120, 100), p, tip, 3)
        report = draft.report()
        lim = report.get("limits") or {}
        hud = [
            f"name {draft.data['name']}   mass {draft.data['mass']:.3g} kg   Iz {draft.data['Iz']:.3g}   table {draft.data['table_size']:.3g} m",
            f"±Fx {lim.get('+Fx', 0):+.3g}/{lim.get('-Fx', 0):+.3g}   "
            f"±Fy {lim.get('+Fy', 0):+.3g}/{lim.get('-Fy', 0):+.3g}   "
            f"±Mz {lim.get('+Mz', 0):+.3g}/{lim.get('-Mz', 0):+.3g}   full={'yes' if report.get('full') else 'NO'}",
            "click add/move   del remove   1 solenoid  2 pwm_fan  3 continuous   b bidirectional",
            "[ ] F_max   r rotate   ,/. mass   -/= Iz   arrows table   s save vehicles/   esc quit",
        ]
        if report.get("warning"):
            hud.append("WARNING: " + str(report["warning"]))
        if draft.selected is not None:
            t = draft.data["thrusters"][draft.selected]
            hud.append(
                f"selected {t['id']} type={t['type']} F_max={t['F_max']:.3g} bidir={t.get('bidirectional', False)} "
                f"dir={t['force_direction']}"
            )
        y = 8
        for line in hud:
            img = (small if len(line) > 90 else font).render(line, True, (230, 230, 230))
            screen.blit(img, (table_px + 56, y))
            y += 22
        pygame.display.flip()
        if save_png and frame == 0:
            pygame.image.save(screen, str(save_png))
        frame += 1
        if headless and frame >= frames:
            running = False
        clock.tick(60)
    pygame.quit()
    return draft


def _matplotlib_layout(spec: SatelliteSpec | None, draft: VehicleDraft, path: Path) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, FancyArrow, Rectangle

    table = float(draft.data["table_size"])
    half = table / 2
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.add_patch(Rectangle((-half, -half), table, table, fill=False, lw=2, color="0.4"))
    ax.scatter([draft.data["com"][0]], [draft.data["com"][1]], marker="+", c="#e3b341", s=80, zorder=5)
    for t in draft.data["thrusters"]:
        p = t["position"]
        d = t["force_direction"]
        ax.plot(p[0], p[1], "o", color="#4cc2ff", ms=8)
        ax.annotate("", xy=(p[0] + 0.08 * d[0], p[1] + 0.08 * d[1]), xytext=p,
                    arrowprops=dict(arrowstyle="->", color="#ff7b72", lw=2))
        ax.text(p[0] + 0.02, p[1] + 0.02, t["id"], fontsize=8, color="0.2")
    ax.set_aspect("equal")
    ax.set_xlim(-half, half)
    ax.set_ylim(-half, half)
    ax.set_title(draft.data["name"])
    ax.set_xlabel("body x (m)")
    ax.set_ylabel("body y (m)")
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=140)
    plt.close(fig)


def plot_layout(spec: SatelliteSpec, path: Path) -> Path:
    draft = VehicleDraft.from_spec(spec)
    _matplotlib_layout(spec, draft, Path(path))
    return Path(path)
