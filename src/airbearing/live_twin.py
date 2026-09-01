"""Live top-down twin: same Runtime, pygame table, plumes, trail, HUD, teleop → MPC."""

from __future__ import annotations

import os
from pathlib import Path

import numpy as np

from airbearing.control.mpc import LinearMPC
from airbearing.missions import point_to_point
from airbearing.runtime import Runtime
from airbearing.spec import SatelliteSpec


def _render_mpl(spec: SatelliteSpec, rt: Runtime, path: Path) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle, RegularPolygon

    logs = rt.result.logs
    st = np.array([r.state for r in logs]) if logs else rt.plant.state.reshape(1, 6)
    cmd = logs[-1].cmd if logs else np.zeros(spec.n_thrusters)
    pose = st[-1]
    half = spec.table_size / 2
    fig, ax = plt.subplots(figsize=(6.4, 6.4))
    ax.add_patch(Rectangle((-half, -half), spec.table_size, spec.table_size, fill=False, lw=2, color="0.35"))
    ax.plot(st[:, 0], st[:, 1], color="#1f6feb", lw=1.6, alpha=0.85)
    x, y, th = pose[0], pose[1], pose[2]
    body = RegularPolygon((x, y), 8, radius=spec.hull_radius, orientation=th, fc="#79c0ff", ec="#0969da", lw=1.5)
    ax.add_patch(body)
    c, s = np.cos(th), np.sin(th)
    for k, thr in enumerate(spec.thrusters):
        pb = thr.position
        pi = np.array([x + c * pb[0] - s * pb[1], y + s * pb[0] + c * pb[1]])
        mag = abs(float(cmd[k])) if k < len(cmd) else 0.0
        fb = -thr.force_direction * (0.06 + 0.22 * mag)
        fi = np.array([c * fb[0] - s * fb[1], s * fb[0] + c * fb[1]])
        ax.plot([pi[0], pi[0] + fi[0]], [pi[1], pi[1] + fi[1]], color="#ff7b72", lw=3, alpha=0.2 + 0.8 * min(1.0, mag))
    ax.set_xlim(-half, half)
    ax.set_ylim(-half, half)
    ax.set_aspect("equal")
    ax.set_title(f"{spec.name}  t={rt._t:.2f}s  {rt.mode}  estop={rt.estop}")
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=140)
    plt.close(fig)


def _try_pygame():
    try:
        import pygame
        return pygame
    except Exception:
        return None


def view(
    spec: SatelliteSpec,
    *,
    record: bool = False,
    out_png: Path | None = None,
    out_gif: Path | None = None,
    duration: float | None = None,
    replay: str | Path | None = None,
    dashboard=None,
    controller=None,
    runs_root: Path | None = None,
) -> Path | None:
    mission = point_to_point(spec.table_size)
    if duration is not None:
        mission.duration = duration
    elif record:
        mission.duration = 4.0
    ctrl = controller or LinearMPC(spec, horizon=8)
    rt = Runtime(spec, ctrl, mission, replay=replay, runs_root=runs_root or Path("runs") / "view")
    rt.begin()
    png = Path(out_png) if out_png else Path("docs/assets/live_twin.png")
    gif = Path(out_gif) if out_gif else Path("docs/assets/live_twin.gif")

    if record:
        os.environ.setdefault("SDL_VIDEODRIVER", "dummy")

    pygame = _try_pygame()
    frames = []
    if pygame is None or (record and os.environ.get("AIRBEARING_TWIN") == "mpl"):
        while rt.tick():
            pass
        rt.finish()
        _render_mpl(spec, rt, png)
        if record:
            try:
                from airbearing.viz import animate
                animate(spec, rt.result, gif, stride=3)
            except Exception:
                pass
        return png

    pygame.init()
    size = 720
    screen = pygame.display.set_mode((size, 760))
    pygame.display.set_caption("airbearing live twin")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("dejavusansmono,consolas,monospace", 16)
    running = True
    half_tbl = spec.table_size / 2
    pad = 40
    table_px = size - 2 * pad

    def w2p(x, y):
        s = table_px / spec.table_size
        return int(pad + (x + half_tbl) * s), int(pad + (half_tbl - y) * s)

    while running:
        wrench = np.zeros(3)
        keys = pygame.key.get_pressed()
        if keys[pygame.K_LEFT] or keys[pygame.K_a]:
            wrench[0] -= 0.4 * max(t.F_max for t in spec.thrusters) * spec.n_thrusters / 4
        if keys[pygame.K_RIGHT] or keys[pygame.K_d]:
            wrench[0] += 0.4 * max(t.F_max for t in spec.thrusters) * spec.n_thrusters / 4
        if keys[pygame.K_UP] or keys[pygame.K_w]:
            wrench[1] += 0.4 * max(t.F_max for t in spec.thrusters) * spec.n_thrusters / 4
        if keys[pygame.K_DOWN] or keys[pygame.K_s]:
            wrench[1] -= 0.4 * max(t.F_max for t in spec.thrusters) * spec.n_thrusters / 4
        if keys[pygame.K_q]:
            wrench[2] += 0.15
        if keys[pygame.K_e]:
            wrench[2] -= 0.15
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                running = False
            elif ev.type == pygame.KEYDOWN:
                if ev.key in (pygame.K_ESCAPE,):
                    running = False
                elif ev.key == pygame.K_SPACE:
                    rt.estop = True
                    rt.mode = "teleop"
                elif ev.key == pygame.K_m:
                    rt.estop = False
                    rt.mode = "mpc"
                elif ev.key == pygame.K_t:
                    rt.estop = False
                    rt.mode = "teleop"
        if rt.mode == "teleop" and not rt.estop:
            rt.teleop_wrench = wrench
        alive = rt.tick()
        if dashboard is not None:
            pass  # status_fn closes over rt
        if not alive and record:
            running = False
        elif not alive and not record:
            # keep drawing last frame until quit
            pass

        screen.fill((16, 18, 24))
        pygame.draw.rect(screen, (48, 56, 68), pygame.Rect(pad, pad, table_px, table_px), 2)
        logs = rt.result.logs
        if logs:
            pts = [w2p(r.state[0], r.state[1]) for r in logs]
            if len(pts) > 1:
                pygame.draw.lines(screen, (70, 140, 255), False, pts, 2)
            pose = logs[-1].state
            cmd = logs[-1].cmd
        else:
            pose = rt.plant.state
            cmd = rt._last_cmd
        x, y, th = float(pose[0]), float(pose[1]), float(pose[2])
        c, s = np.cos(th), np.sin(th)
        hull = []
        for k in range(8):
            a = th + k * np.pi / 4
            hull.append(w2p(x + spec.hull_radius * np.cos(a), y + spec.hull_radius * np.sin(a)))
        if len(hull) >= 3:
            pygame.draw.polygon(screen, (80, 160, 220), hull)
        for k, thr in enumerate(spec.thrusters):
            pb = thr.position
            pi = (x + c * pb[0] - s * pb[1], y + s * pb[0] + c * pb[1])
            mag = abs(float(cmd[k])) if k < len(cmd) else 0.0
            fb = -thr.force_direction * (0.05 + 0.20 * mag)
            fi = (c * fb[0] - s * fb[1], s * fb[0] + c * fb[1])
            p0 = w2p(*pi)
            p1 = w2p(pi[0] + fi[0], pi[1] + fi[1])
            pygame.draw.line(screen, (255, 120, 100), p0, p1, 3)
        hud = (
            f"{spec.name}  t={rt._t:.2f}s  {rt.mode}  estop={rt.estop}  "
            f"x={x:+.2f} y={y:+.2f} yaw={th:+.2f}   arrows teleop  T teleop  M MPC  space e-stop"
        )
        screen.blit(font.render(hud[:120], True, (230, 230, 230)), (12, size + 8))
        pygame.display.flip()
        if record:
            frames.append(pygame.surfarray.array3d(screen).copy())
            if not alive:
                running = False
        clock.tick(30 if not record else 0)

    if record:
        pygame.image.save(screen, str(png))
        if frames:
            try:
                from PIL import Image

                imgs = [Image.fromarray(np.transpose(f, (1, 0, 2))) for f in frames[::2]]
                if imgs:
                    gif.parent.mkdir(parents=True, exist_ok=True)
                    imgs[0].save(gif, save_all=True, append_images=imgs[1:], duration=80, loop=0)
            except Exception:
                pass
        png.parent.mkdir(parents=True, exist_ok=True)
    pygame.quit()
    if not rt.result.logs:
        pass
    else:
        try:
            rt.finish()
        except Exception:
            pass
    if record and not png.exists():
        _render_mpl(spec, rt, png)
    return png if record else None
