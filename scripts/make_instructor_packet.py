#!/usr/bin/env python3
"""Render docs/INSTRUCTOR.md as a one-page PDF (and PNG) for TAs/PIs."""
from __future__ import annotations

from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
OUT_PDF = REPO / "docs" / "instructor.pdf"
OUT_PNG = REPO / "docs" / "assets" / "instructor_onepager.png"


def _story():
    from reportlab.lib import colors
    from reportlab.lib.enums import TA_LEFT
    from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
    from reportlab.lib.units import inch
    from reportlab.platypus import (
        ListFlowable,
        ListItem,
        Paragraph,
        Preformatted,
        Spacer,
        Table,
        TableStyle,
    )

    ss = getSampleStyleSheet()
    title = ParagraphStyle(
        "T",
        parent=ss["Title"],
        fontName="Helvetica-Bold",
        fontSize=13,
        leading=15,
        spaceAfter=4,
        textColor=colors.HexColor("#1a1a1a"),
        alignment=TA_LEFT,
    )
    h = ParagraphStyle(
        "H",
        parent=ss["Heading2"],
        fontName="Helvetica-Bold",
        fontSize=9.5,
        leading=12,
        spaceBefore=6,
        spaceAfter=2,
        textColor=colors.HexColor("#1a1a1a"),
    )
    body = ParagraphStyle(
        "B",
        parent=ss["BodyText"],
        fontName="Helvetica",
        fontSize=8,
        leading=10.2,
        spaceAfter=2,
        textColor=colors.HexColor("#222"),
    )
    code = ParagraphStyle(
        "C",
        parent=ss["Code"],
        fontName="Courier",
        fontSize=6.7,
        leading=8.6,
        backColor=colors.HexColor("#f4f4f4"),
        leftIndent=4,
        rightIndent=4,
        spaceBefore=2,
        spaceAfter=3,
    )
    cell = ParagraphStyle(
        "Cell",
        parent=body,
        fontSize=7.2,
        leading=9.2,
        spaceAfter=0,
    )
    cellb = ParagraphStyle("CellB", parent=cell, fontName="Helvetica-Bold")
    tiny = ParagraphStyle("Tiny", parent=body, fontSize=7, leading=9, textColor=colors.HexColor("#444"))

    def P(text, style=body):
        return Paragraph(text, style)

    def codeblock(text: str):
        return Preformatted(text, code)

    lab_header = [P(x, cellb) for x in ("Lab", "Hours", "Topic", "Done looks like")]
    labs = [
        lab_header,
        [P("1", cell), P("1–2 h", cell), P("Editor + check", cell),
         P("<font face='Courier' size='6.5'>airbearing check vehicles/&lt;name&gt;.json</font> prints <b>both-signs YES</b>; JSON in <font face='Courier' size='6.5'>vehicles/*.json</font>", cell)],
        [P("2", cell), P("1–2 h", cell), P("PD vs LQR vs MPC", cell),
         P("Methods table from <font face='Courier' size='6.5'>airbearing report</font> / <font face='Courier' size='6.5'>methods.txt</font> for all three", cell)],
        [P("3", cell), P("1–2 h", cell), P("Identify F_max", cell),
         P("Identified JSON <b>beats the guess on RMSE</b>; residual plot written", cell)],
        [P("4", cell), P("1–2 h", cell), P("Binary vs PWM vs mismatch", cell),
         P("Binary vs PWM vs F_max mismatch compared (<font face='Courier' size='6.5'>make lab4</font>)", cell)],
    ]
    lab_table = Table(labs, colWidths=[0.42 * inch, 0.55 * inch, 1.55 * inch, 4.88 * inch])
    lab_table.setStyle(
        TableStyle(
            [
                ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#ececec")),
                ("GRID", (0, 0), (-1, -1), 0.3, colors.HexColor("#bbbbbb")),
                ("VALIGN", (0, 0), (-1, -1), "TOP"),
                ("LEFTPADDING", (0, 0), (-1, -1), 3),
                ("RIGHTPADDING", (0, 0), (-1, -1), 3),
                ("TOPPADDING", (0, 0), (-1, -1), 2),
                ("BOTTOMPADDING", (0, 0), (-1, -1), 2),
            ]
        )
    )

    files = [
        [P("Path", cellb), P("What", cellb)],
        [P("<font face='Courier' size='6.5'>vehicles/</font>", cell),
         P("<b>Their</b> satellite JSON. Editor / new-vehicle will not save anywhere else.", cell)],
        [P("<font face='Courier' size='6.5'>runs/</font>", cell),
         P("Each run writes log.csv, summary.json, methods.txt, plots.", cell)],
        [P("<font face='Courier' size='6.5'>examples/vehicles/</font>", cell),
         P("Shipped starting guesses — copy, then calibrate. Do not edit src/ to add a satellite.", cell)],
    ]
    file_table = Table(files, colWidths=[1.7 * inch, 5.7 * inch])
    file_table.setStyle(
        TableStyle(
            [
                ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#ececec")),
                ("GRID", (0, 0), (-1, -1), 0.3, colors.HexColor("#bbbbbb")),
                ("VALIGN", (0, 0), (-1, -1), "TOP"),
                ("LEFTPADDING", (0, 0), (-1, -1), 3),
                ("RIGHTPADDING", (0, 0), (-1, -1), 3),
                ("TOPPADDING", (0, 0), (-1, -1), 2),
                ("BOTTOMPADDING", (0, 0), (-1, -1), 2),
            ]
        )
    )

    grade_items = [
        ListItem(P("<b>Lab 1:</b> <font face='Courier' size='6.5'>vehicles/*.json</font> plus check output showing both-signs <b>YES</b>.", tiny), leftIndent=8),
        ListItem(P("<b>Lab 2:</b> three <font face='Courier' size='6.5'>runs/&lt;id&gt;/summary.json</font> (or methods.txt) — PD, LQR, MPC on the same hop.", tiny), leftIndent=8),
        ListItem(P("<b>Lab 3:</b> identified JSON and residual plot; identified RMSE smaller than the uncalibrated guess.", tiny), leftIndent=8),
        ListItem(P("<b>Lab 4:</b> binary / PWM / mismatch numbers from <font face='Courier' size='6.5'>make lab4</font> (or equivalent summary.json).", tiny), leftIndent=8),
    ]

    story = [
        P("Instructor packet — airbearing (20 minutes)", title),
        P(
            "Planar (x, y, yaw) GNC package for cubesats on air-bearing tables. "
            "Students describe <b>their</b> vehicle in one JSON file. "
            "<b>Not flight software.</b> SI units. Python <b>3.11+</b>. "
            "Solvers are OSQP / Clarabel via cvxpy — <b>no Gurobi</b>.",
            body,
        ),
        P(
            "Clone: <font face='Courier' size='7.5'>https://github.com/AevarOfjord/airbearing-testbed-mpc</font> "
            "· source of truth: <font face='Courier' size='7.5'>docs/INSTRUCTOR.md</font>",
            tiny,
        ),
        P("Install (3 lines)", h),
        codeblock(
            "git clone https://github.com/AevarOfjord/airbearing-testbed-mpc.git\n"
            "cd airbearing-testbed-mpc && python3 -m venv .venv && source .venv/bin/activate\n"
            "pip install -e \".[dev,viz]\""
        ),
        P("Then <font face='Courier' size='7.5'>airbearing --help</font> and <font face='Courier' size='7.5'>make test</font>. Windows: <font face='Courier' size='7.5'>.venv\\Scripts\\activate</font>.", tiny),
        P("Labs 1–4 (software; 1–2 h each)", h),
        P("Hardware is <b>not</b> required. <font face='Courier' size='7.5'>make lab1</font> … <font face='Courier' size='7.5'>make lab4</font> run the staff demos. Write-ups: <font face='Courier' size='7.5'>labs/</font>. Staff notes: <font face='Courier' size='7.5'>labs/staff/</font>.", body),
        lab_table,
        P("Where student files go", h),
        file_table,
        Spacer(1, 3),
        P("Offline sim-vs-log demo (fresh clone, no table):", tiny),
        codeblock("airbearing compare --sim examples/logs/sim.csv --real examples/logs/hardware.csv"),
        P("Hardware (optional)", h),
        P(
            "Labs 1–4 are software. A real table is extra / a later session. "
            "<font face='Courier' size='7.5'>--armed</font> is required. "
            "The runtime <b>refuses null mocap</b> (zeros commands and aborts). "
            "Gateways implement a <b>~100 ms deadman</b>. See <font face='Courier' size='7.5'>docs/HARDWARE.md</font>.",
            body,
        ),
        codeblock("airbearing run --vehicle vehicles/mine.json --armed --port /dev/ttyUSB0 --dashboard"),
        P("Grading — artifacts to collect", h),
        ListFlowable(grade_items, bulletType="bullet", start="•", leftIndent=12, bulletFontSize=7, spaceBefore=0, spaceAfter=1),
        P("Hashes in methods.txt (vehicle_sha256, optional git hash) make it obvious which JSON produced the run.", tiny),
        P("Safety", h),
        P(
            "This kit is laboratory / teaching software, <b>not flight software</b>. "
            "The Python supervisor never <i>adds</i> thrust — it only zeros commands. "
            "Serial gateways force every actuator off if no host packet arrives within ~100 ms. "
            "<font face='Courier' size='7.5'>--armed</font> is required for hardware; null telemetry is refused rather than flown open-loop. "
            "Keep the table clear. A dashboard e-stop is not a substitute for a physical kill switch. "
            "If you bypass <font face='Courier' size='7.5'>--armed</font> or the deadman, this package will not save a runaway vehicle.",
            body,
        ),
    ]
    return story


def write_pdf(path: Path = OUT_PDF) -> Path:
    from reportlab.lib.pagesizes import letter
    from reportlab.lib.units import inch
    from reportlab.platypus import SimpleDocTemplate

    path.parent.mkdir(parents=True, exist_ok=True)
    doc = SimpleDocTemplate(
        str(path),
        pagesize=letter,
        leftMargin=0.5 * inch,
        rightMargin=0.5 * inch,
        topMargin=0.42 * inch,
        bottomMargin=0.38 * inch,
        title="Instructor packet — airbearing",
        author="airbearing",
    )
    doc.build(_story())
    return path


def write_png(pdf: Path, png: Path = OUT_PNG) -> Path | None:
    """Rasterize page 1 with pdftoppm if available."""
    import shutil
    import subprocess

    png.parent.mkdir(parents=True, exist_ok=True)
    pdftoppm = shutil.which("pdftoppm")
    if not pdftoppm:
        return None
    stem = png.with_suffix("")
    subprocess.check_call([pdftoppm, "-png", "-r", "140", "-singlefile", str(pdf), str(stem)])
    return png if png.is_file() else None


def main() -> int:
    pdf = write_pdf()
    print(f"wrote {pdf}")
    png = write_png(pdf)
    if png:
        print(f"wrote {png}")
    else:
        print("pdftoppm not found; PDF only")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
