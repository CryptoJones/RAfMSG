import html
from pathlib import Path
from xml.etree import ElementTree as ET

from PIL import Image, ImageDraw, ImageFont

SCALE = 2
FONT_REG = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
FONT_BOLD = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"

BG = "#07090f"
TEXT = "#d6f2ff"
LINE = "#a8bdd0"
MUTED = "#8190a4"

TONES = {
    "cyan": ("#0a1726", "#27d4ff"),
    "green": ("#0a1a14", "#55ff99"),
    "amber": ("#1a1408", "#ffb454"),
    "purple": ("#140d24", "#b48cff"),
    "red": ("#1a0b10", "#ff6b81"),
    "muted": ("#0e1117", "#8190a4"),
}


def font(size, bold=False):
    return ImageFont.truetype(FONT_BOLD if bold else FONT_REG, size * SCALE)


def pt(x, y):
    return (int(x * SCALE), int(y * SCALE))


class Diagram:
    def __init__(self, name, title, subtitle, width=1680, height=1180):
        self.name = name
        self.title = title
        self.subtitle = subtitle
        self.width = width
        self.height = height
        self.bands = []
        self.nodes = {}
        self.texts = []
        self.edges = []

    def band(self, x, y, w, h, label, tone="muted"):
        self.bands.append(dict(x=x, y=y, w=w, h=h, label=label, tone=tone))

    def node(self, key, x, y, w, h, title, lines=(), tone="cyan", size=15):
        self.nodes[key] = dict(
            x=x, y=y, w=w, h=h, title=title, lines=list(lines), tone=tone, size=size
        )

    def text(self, x, y, value, size=16, tone="muted", bold=False, align="left"):
        self.texts.append(
            dict(x=x, y=y, value=value, size=size, tone=tone, bold=bold, align=align)
        )

    def edge(self, src=None, dst=None, points=None, tone="cyan", dashed=False, label=None, label_pos=None):
        self.edges.append(
            dict(
                src=src,
                dst=dst,
                points=list(points) if points else None,
                tone=tone,
                dashed=dashed,
                label=label,
                label_pos=label_pos,
            )
        )

    def edge_points(self, edge):
        if edge["points"]:
            return edge["points"]
        a = self.nodes[edge["src"]]
        b = self.nodes[edge["dst"]]
        acx, acy = a["x"] + a["w"] / 2, a["y"] + a["h"] / 2
        bcx, bcy = b["x"] + b["w"] / 2, b["y"] + b["h"] / 2
        dx, dy = bcx - acx, bcy - acy
        if abs(dx) >= abs(dy):
            sign = 1 if dx > 0 else -1
            return [
                (acx + sign * a["w"] / 2, acy),
                (bcx - sign * b["w"] / 2, bcy),
            ]
        sign = 1 if dy > 0 else -1
        return [
            (acx, acy + sign * a["h"] / 2),
            (bcx, bcy - sign * b["h"] / 2),
        ]

    def _draw_band(self, draw, band):
        fill, stroke = TONES[band["tone"]]
        draw.rounded_rectangle(
            [pt(band["x"], band["y"]), pt(band["x"] + band["w"], band["y"] + band["h"])],
            radius=14 * SCALE,
            fill=fill,
            outline=stroke,
            width=2 * SCALE,
        )
        draw.text(pt(band["x"] + 22, band["y"] + 20), band["label"], font=font(15, True), fill=stroke, anchor="lm")

    def _draw_node(self, draw, node):
        fill, stroke = TONES[node["tone"]]
        x, y, w, h = node["x"], node["y"], node["w"], node["h"]
        draw.rounded_rectangle(
            [pt(x, y), pt(x + w, y + h)],
            radius=12 * SCALE,
            fill=fill,
            outline=stroke,
            width=2 * SCALE,
        )
        draw.rounded_rectangle(
            [pt(x, y + 6), pt(x + 6, y + h - 6)],
            radius=3 * SCALE,
            fill=stroke,
        )
        title_font = font(node["size"], True)
        line_font = font(max(11, node["size"] - 2))
        line_h = (node["size"] - 2 + 7) * SCALE
        total = (node["size"] + 6) * SCALE + len(node["lines"]) * line_h
        start = y * SCALE + (h * SCALE - total) / 2
        cx = (x + w / 2) * SCALE
        draw.text((cx, start + (node["size"] + 6) * SCALE / 2), node["title"], font=title_font, fill=TEXT, anchor="mm")
        cursor = start + (node["size"] + 6) * SCALE
        for line in node["lines"]:
            draw.text((cx, cursor + line_h / 2), line, font=line_font, fill=LINE, anchor="mm")
            cursor += line_h

    def _draw_edge(self, draw, edge):
        stroke = TONES[edge["tone"]][1]
        points = [pt(x, y) for x, y in self.edge_points(edge)]
        width = max(2, 2) * SCALE
        if edge["dashed"]:
            for a, b in zip(points, points[1:]):
                length = ((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2) ** 0.5
                if length == 0:
                    continue
                ux, uy = (b[0] - a[0]) / length, (b[1] - a[1]) / length
                pos = 0.0
                while pos < length:
                    end = min(pos + 10 * SCALE, length)
                    draw.line(
                        [(a[0] + ux * pos, a[1] + uy * pos), (a[0] + ux * end, a[1] + uy * end)],
                        fill=stroke,
                        width=width,
                    )
                    pos += 17 * SCALE
        else:
            draw.line(points, fill=stroke, width=width, joint="curve")
        tail, head = points[-2], points[-1]
        dx, dy = head[0] - tail[0], head[1] - tail[1]
        length = max(1.0, (dx * dx + dy * dy) ** 0.5)
        ux, uy = dx / length, dy / length
        px, py = -uy, ux
        size = 12 * SCALE
        wing = 6 * SCALE
        draw.polygon(
            [
                head,
                (head[0] - ux * size + px * wing, head[1] - uy * size + py * wing),
                (head[0] - ux * size - px * wing, head[1] - uy * size - py * wing),
            ],
            fill=stroke,
        )
        if edge["label"] and edge["label_pos"]:
            tx, ty = edge["label_pos"]
            draw.text(pt(tx, ty), edge["label"], font=font(12), fill=stroke, anchor="mm")

    def render_png(self, path):
        image = Image.new("RGB", (self.width * SCALE, self.height * SCALE), BG)
        draw = ImageDraw.Draw(image)
        draw.text(pt(60, 42), self.title, font=font(30, True), fill=TEXT, anchor="lm")
        draw.text(pt(60, 88), self.subtitle, font=font(17), fill=MUTED, anchor="lm")
        for band in self.bands:
            self._draw_band(draw, band)
        for edge in self.edges:
            self._draw_edge(draw, edge)
        for node in self.nodes.values():
            self._draw_node(draw, node)
        for item in self.texts:
            color = TONES[item["tone"]][1] if item["tone"] != "muted" else MUTED
            draw.text(
                pt(item["x"], item["y"]),
                item["value"],
                font=font(item["size"], item["bold"]),
                fill=color,
                anchor="lm" if item["align"] == "left" else "mm",
            )
        image.resize((self.width, self.height), Image.Resampling.LANCZOS).save(path)

    def render_drawio(self, path):
        mxfile = ET.Element("mxfile", host="drawio", version="26.0.0")
        diagram = ET.SubElement(mxfile, "diagram", name=self.name)
        model = ET.SubElement(
            diagram,
            "mxGraphModel",
            dx=str(self.width),
            dy=str(self.height),
            grid="0",
            page="1",
            pageWidth=str(self.width),
            pageHeight=str(self.height),
            background=BG,
        )
        root = ET.SubElement(model, "root")
        ET.SubElement(root, "mxCell", id="0")
        ET.SubElement(root, "mxCell", id="1", parent="0")

        def cell(cid, value, style, x, y, w, h, parent="1"):
            ET.SubElement(
                root,
                "mxCell",
                id=cid,
                value=value,
                style=style,
                vertex="1",
                parent=parent,
            ).append(ET.Element("mxGeometry", x=str(x), y=str(y), width=str(w), height=str(h), **{"as": "geometry"}))

        cell(
            "canvas",
            "",
            "rounded=0;html=0;fillColor=%s;strokeColor=none;pointerEvents=0;" % BG,
            0,
            0,
            self.width,
            self.height,
        )
        cell(
            "heading",
            "<b>%s</b>" % html.escape(self.title),
            "text;html=1;strokeColor=none;fillColor=none;fontColor=%s;fontSize=28;fontStyle=1;fontFamily=Verdana;align=left;verticalAlign=middle;" % TEXT,
            60,
            22,
            self.width - 120,
            44,
        )
        cell(
            "subheading",
            html.escape(self.subtitle),
            "text;html=1;strokeColor=none;fillColor=none;fontColor=%s;fontSize=15;fontFamily=Verdana;align=left;verticalAlign=middle;" % MUTED,
            60,
            70,
            self.width - 120,
            32,
        )
        for index, band in enumerate(self.bands):
            fill, stroke = TONES[band["tone"]]
            cell(
                "band%d" % index,
                "<b>%s</b>" % html.escape(band["label"]),
                "rounded=1;arcSize=8;dashed=1;dashPattern=8 8;html=1;whiteSpace=wrap;fillColor=%s;strokeColor=%s;strokeWidth=2;verticalAlign=top;spacingTop=8;spacingLeft=16;fontColor=%s;fontSize=14;fontStyle=1;fontFamily=Verdana;align=left;"
                % (fill, stroke, stroke),
                band["x"],
                band["y"],
                band["w"],
                band["h"],
            )
        for key, node in self.nodes.items():
            fill, stroke = TONES[node["tone"]]
            value = "<b>%s</b>" % html.escape(node["title"])
            for line in node["lines"]:
                value += "<br/>%s" % html.escape(line)
            cell(
                "node_%s" % key,
                value,
                "rounded=1;arcSize=12;html=1;whiteSpace=wrap;fillColor=%s;strokeColor=%s;strokeWidth=2;fontColor=%s;fontSize=%d;fontFamily=Verdana;align=center;verticalAlign=middle;spacing=6;"
                % (fill, stroke, TEXT, node["size"]),
                node["x"],
                node["y"],
                node["w"],
                node["h"],
            )
        for index, item in enumerate(self.texts):
            color = TONES[item["tone"]][1] if item["tone"] != "muted" else MUTED
            cell(
                "label%d" % index,
                html.escape(item["value"]),
                "text;html=1;strokeColor=none;fillColor=none;fontColor=%s;fontSize=%d;fontStyle=%d;fontFamily=Verdana;align=%s;verticalAlign=middle;"
                % (color, item["size"], 1 if item["bold"] else 0, item["align"]),
                item["x"] - 300 if item["align"] == "center" else item["x"],
                item["y"] - 14,
                600 if item["align"] == "center" else self.width - item["x"] - 40,
                28,
            )
        for index, edge in enumerate(self.edges):
            stroke = TONES[edge["tone"]][1]
            attributes = dict(
                id="edge%d" % index,
                value=html.escape(edge["label"] or ""),
                style="edgeStyle=orthogonalEdgeStyle;rounded=0;html=1;endArrow=block;endFill=1;strokeColor=%s;strokeWidth=2;dashed=%d;fontColor=%s;fontSize=12;"
                % (stroke, 1 if edge["dashed"] else 0, stroke),
                edge="1",
                parent="1",
            )
            if edge["src"]:
                attributes["source"] = "node_%s" % edge["src"]
            if edge["dst"]:
                attributes["target"] = "node_%s" % edge["dst"]
            ec = ET.SubElement(root, "mxCell", **attributes)
            geometry = ET.SubElement(ec, "mxGeometry", relative="1", **{"as": "geometry"})
            points = self.edge_points(edge)
            if not edge["src"]:
                geometry.append(ET.Element("mxPoint", x=str(points[0][0]), y=str(points[0][1]), **{"as": "sourcePoint"}))
            if not edge["dst"]:
                geometry.append(ET.Element("mxPoint", x=str(points[-1][0]), y=str(points[-1][1]), **{"as": "targetPoint"}))
            if edge["points"]:
                for x, y in edge["points"][1:-1]:
                    geometry.append(ET.Element("mxPoint", x=str(x), y=str(y), **{"as": "points"}))
        ET.indent(mxfile, space="  ")
        ET.ElementTree(mxfile).write(path, encoding="utf-8", xml_declaration=True)


def architecture():
    d = Diagram(
        "RAfMSG-architecture",
        "RAfMSG — Two-Tier Seam Architecture",
        "Executable contracts first: a deterministic reference core, engine-shaped adapters, and standalone subsystem seams — verified by headless tests.",
    )
    d.band(60, 140, 1560, 330, "ENGINE-SHAPED ADAPTERS  —  the same contracts expressed with real crates", "purple")
    d.band(60, 510, 1000, 360, "DETERMINISTIC REFERENCE CORE  —  self-contained, BTreeMap-ordered, f64", "cyan")
    d.band(1100, 510, 520, 360, "STANDALONE SEAM SUBSYSTEMS  —  contract-first, test-only", "green")
    d.band(60, 900, 1560, 270, "SEAM CONTRACT TESTS  —  the actual product", "amber")

    adapters = [
        ("ecs", 90, 196, "bevy_ecs substrate", ["ZST role markers", "ordered SystemSets"], "purple"),
        ("prediction", 610, 196, "lightyear prediction", ["HistoryBuffer", "10-tick rollback cap"], "purple"),
        ("input", 1130, 196, "input authority", ["sequenced inputs", "pluggable validators"], "purple"),
        ("network", 90, 336, "network adapter", ["bincode · UDP · delta", "ARQ · heartbeat"], "purple"),
        ("physics3d", 610, 336, "rapier3d-f64", ["fixed 60 Hz", "sorted snapshots"], "purple"),
        ("flight", 1130, 336, "flight dynamics", ["12-thruster allocation", "g-limit clamps"], "purple"),
    ]
    for key, x, y, title, lines, tone in adapters:
        d.node(key, x, y, 480, 108, title, lines, tone)

    core = [
        ("types", 90, 566, "types / physics", ["f64 Vec3/Quat · DT 1/60", "one shared integrator"], "cyan"),
        ("world", 410, 566, "world", ["BTreeMap order", "opt-in replication"], "cyan"),
        ("sim", 730, 566, "sim", ["server input buffer", "client rollback"], "cyan"),
        ("ifcs", 90, 706, "ifcs", ["control law", "thrust + torque"], "cyan"),
        ("net", 410, 706, "net", ["seeded LCG", "latency/jitter/loss"], "cyan"),
        ("harness", 730, 706, "test_harness", ["headless runner", "JSON reports"], "cyan"),
    ]
    for key, x, y, title, lines, tone in core:
        d.node(key, x, y, 280, 108, title, lines, tone)

    seams = [
        ("meshing", 1130, 566, "meshing", ["single authority", "2-phase handoff"], "green"),
        ("origin", 1390, 566, "floating origin", ["f64 world", "local-frame rebase"], "green"),
        ("interest", 1130, 706, "interest", ["R-tree index", "10/12 km hysteresis"], "green"),
        ("backend", 1390, 706, "backend", ["HMAC · sessions", "SQLite persistence"], "green"),
    ]
    for key, x, y, title, lines, tone in seams:
        d.node(key, x, y, 240, 108, title, lines, tone)

    d.node("tests", 360, 950, 960, 140, "8 seam contracts · 114 headless tests",
           ["bit-exact math · latency/loss/fault injection · handoff state machines · SQLite restart · UDP loopback"], "amber")

    for src, dst in [("sim", "prediction"), ("net", "network"), ("ifcs", "flight")]:
        d.edge(src, dst, tone="amber", dashed=True)
    d.text(840, 458, "same contract, different substrate", 13, "amber", align="center")

    d.edge(points=[(60, 300), (42, 300), (42, 1020), (360, 1020)], tone="green")
    d.edge(points=[(870, 870), (870, 950)], tone="green")
    d.edge(points=[(1360, 870), (1360, 950)], tone="green")
    d.text(140, 930, "verified by", 12, "green", align="center")
    d.text(1360, 888, "verified by", 12, "green", align="center")
    d.text(840, 1120, "No integrated game binary or renderer: each tier is an executable specification of a seam.",
           16, "muted", align="center")
    return d


def tick_lifecycle():
    d = Diagram(
        "RAfMSG-tick-lifecycle",
        "One Tick — Server Authority, Client Prediction, Reconciliation",
        "60 Hz logical clock · one shared f64 integrator · input replay drives convergence after loss, jitter and injected faults.",
        height=1000,
    )
    top = [
        ("ingest", 60, 160, "Input ingest", ["owner · range · sequence", "hooks · rate limit"], "cyan"),
        ("buffer", 350, 160, "Hold last input", ["keyed by tick", "duplicate/stale rejected"], "cyan"),
        ("server", 640, 160, "Server step", ["advance all entities", "DT = 1/60"], "cyan"),
        ("snapshot", 930, 160, "Opt-in snapshot", ["replicate=true only", "server_only hidden"], "cyan"),
        ("wire", 1220, 160, "Network fault model", ["latency · jitter · loss", "seeded LCG"], "amber"),
    ]
    for key, x, y, title, lines, tone in top:
        d.node(key, x, y, 250, 130, title, lines, tone)

    bottom = [
        ("deliver", 1220, 510, "Monotonic delivery", ["drop stale snapshots", "count ignored"], "purple"),
        ("predict", 930, 510, "Local prediction", ["input history replay", "predicted bodies"], "purple"),
        ("reconcile", 640, 510, "Reconciliation", ["replay snapshot+1 .. now", "pos 0.01 / ori 0.001"], "purple"),
        ("views", 350, 510, "Client views", ["confirmed vs predicted", "separate state for tests"], "purple"),
    ]
    for key, x, y, title, lines, tone in bottom:
        d.node(key, x, y, 250, 130, title, lines, tone)

    for src, dst in [("ingest", "buffer"), ("buffer", "server"), ("server", "snapshot"), ("snapshot", "wire")]:
        d.edge(src, dst, tone="cyan")
    d.edge("wire", "deliver", tone="amber")
    for src, dst in [("deliver", "predict"), ("predict", "reconcile"), ("reconcile", "views")]:
        d.edge(src, dst, tone="green")
    d.edge(points=[(350, 575), (185, 575), (185, 290)], tone="muted")
    d.text(120, 430, "next tick", 13, "muted", align="center")
    d.edge(points=[(1345, 290), (1345, 400), (765, 400), (765, 510)], tone="red", dashed=True)
    d.text(1055, 385, "fault injection: +5.0 to pos.x → must converge", 13, "red", align="center")

    d.band(60, 730, 1560, 210, "TWO RECONCILIATION TIERS  —  identical invariants, different bounds", "cyan")
    d.node("reference", 90, 790, 700, 120, "Reference client (sim.rs)",
           ["BTreeMap history · no rollback cap · fixed epsilon constants", "diagnostic counters: rollbacks, stale snapshots"], "cyan")
    d.node("adapter", 890, 790, 700, 120, "Adapter client (prediction.rs)",
           ["lightyear HistoryBuffer · 10-tick cap · 32-tick history", "interpolation delay 6 ticks · configurable epsilons"], "purple")
    d.text(840, 965, "Both tiers accept snapshots monotonically and share physics::advance; only the adapter bounds CPU work.",
           15, "muted", align="center")
    return d


def seam_map():
    d = Diagram(
        "RAfMSG-seam-map",
        "Eight Seams — One Contract Shape",
        "Each seam declares an input surface, invariants and an executable test suite. Seams 1–4 also have dedicated contract files; seams 5–8 live with their subsystem tests.",
        height=1080,
    )
    cards = [
        ("s1", 60, 170, "S1 · Replication", "cyan", ["contract: seam1_replication.rs", "world.rs · sim.rs", "opt-in state; owner-gated mutation"]),
        ("s2", 450, 170, "S2 · Input authority", "cyan", ["contract: seam2_authority.rs", "input.rs · input_authority.rs", "sequenced, validated, hold-last-input"]),
        ("s3", 840, 170, "S3 · Physics prediction", "cyan", ["contract: seam3_determinism.rs", "prediction.rs · prediction_reconciliation.rs", "bit-exact replay; bounded rollback"]),
        ("s4", 1230, 170, "S4 · IFCS", "cyan", ["contract: seam4_ifcs.rs", "ifcs.rs · flight.rs", "clamped axes; g-limit; one integrator"]),
        ("s5", 60, 540, "S5 · Meshing", "green", ["test: server_meshing.rs", "meshing.rs", "single authority; two-phase handoff"]),
        ("s6", 450, 540, "S6 · Floating origin", "green", ["test: floating_origin.rs", "floating_origin.rs", "local-frame physics; pop-free rebase"]),
        ("s7", 840, 540, "S7 · Interest", "green", ["test: interest_management.rs", "interest.rs", "R-tree; 10/12 km hysteresis"]),
        ("s8", 1230, 540, "S8 · Backend", "green", ["test: backend_services.rs", "backend.rs", "HMAC tokens; sessions; SQLite"]),
    ]
    for key, x, y, title, tone, lines in cards:
        d.node(key, x, y, 350, 300, title, lines, tone, size=17)
    d.text(60, 900, "cyan = reference-core contract      green = subsystem contract      amber = contract test file", 15, "muted")
    d.text(60, 945, "The harness reserves a slot for every seam; its seam_templates() entries are placeholders — real coverage is in the named test files.",
           15, "muted")
    d.text(60, 990, "Total: 114 tests across 17 integration test files, all runnable headlessly with cargo test.", 15, "muted")
    return d


def main():
    out = Path(__file__).resolve().parent
    for diagram in (architecture(), tick_lifecycle(), seam_map()):
        stem = diagram.name.lower().replace("rafmsg-", "")
        diagram.render_png(out / (stem + ".png"))
        diagram.render_drawio(out / (stem + ".drawio"))


if __name__ == "__main__":
    main()
