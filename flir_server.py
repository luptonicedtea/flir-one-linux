#!/usr/bin/env python3
"""
Live MJPEG web viewer for a FLIR ONE camera.

Runs a background thread that keeps the USB stream open and continuously
decodes frames, and a small HTTP server that serves:
  /             -- HTML viewer page
  /thermal.mjpg -- MJPEG stream of the thermal image w/ overlay
  /visible.mjpg -- MJPEG stream of the visible camera

Usage: ./venv/bin/python flir_server.py [port]
"""
import io
import json
import struct
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
import usb.core
import usb.util
from PIL import Image, ImageDraw, ImageOps, ImageFont

VENDOR_ID = 0x09CB
PRODUCT_ID = 0x1996
MAGIC = bytes([0xEF, 0xBE, 0x00, 0x00])
BUF_LIMIT = 1 << 20

RAW_W, RAW_H = 82, 62
PIXEL_OFFSET = 30
FRAME_W, FRAME_H = 80, 60

DISPLAY_SIZE = (640, 480)

# Generic Planck constants (not calibrated for this specific unit -- rough estimate only)
PLANCK_R1, PLANCK_B, PLANCK_F, PLANCK_O, PLANCK_R2 = 16528.178, 1427.5, 1.0, -1307.0, 0.012258549
TEMP_REFLECTED, EMISSIVITY = 20.0, 0.95


def planck_raw_to_celsius(raw):
    raw = raw.astype(np.float64) * 4
    raw_refl = PLANCK_R1 / (PLANCK_R2 * (np.exp(PLANCK_B / (TEMP_REFLECTED + 273.15)) - PLANCK_F)) - PLANCK_O
    raw_obj = (raw - (1 - EMISSIVITY) * raw_refl) / EMISSIVITY
    return PLANCK_B / np.log(PLANCK_R1 / (PLANCK_R2 * (raw_obj + PLANCK_O)) + PLANCK_F) - 273.15


class Calibration:
    """User-supplied (raw_count, known_celsius) points -> linear fit, used in
    place of the generic uncalibrated Planck constants once we have >= 2 points."""

    def __init__(self):
        self.lock = threading.Lock()
        self.points = []  # list of {label, raw, known_c}
        self.slope = None
        self.intercept = None

    def add(self, label, raw, known_c):
        with self.lock:
            self.points.append({"label": label, "raw": raw, "known_c": known_c})
            self._refit()

    def remove(self, index):
        with self.lock:
            if 0 <= index < len(self.points):
                self.points.pop(index)
            self._refit()

    def clear(self):
        with self.lock:
            self.points.clear()
            self.slope = self.intercept = None

    def _refit(self):
        if len(self.points) >= 2:
            raws = np.array([p["raw"] for p in self.points], dtype=np.float64)
            knowns = np.array([p["known_c"] for p in self.points], dtype=np.float64)
            self.slope, self.intercept = np.polyfit(raws, knowns, 1)
        else:
            self.slope = self.intercept = None

    def status(self):
        with self.lock:
            return {
                "points": list(self.points),
                "slope": self.slope,
                "intercept": self.intercept,
                "active": self.slope is not None,
            }

    def raw_to_celsius(self, raw):
        with self.lock:
            slope, intercept = self.slope, self.intercept
        if slope is None:
            return planck_raw_to_celsius(raw)
        return raw * slope + intercept


calibration = Calibration()


def raw_to_celsius(raw):
    return calibration.raw_to_celsius(raw)


def set_interface(dev, interface, alt, extra=None):
    dev.ctrl_transfer(0x01, 0x0B, alt, interface, extra, timeout=1000)


def find_device():
    dev = usb.core.find(idVendor=VENDOR_ID, idProduct=PRODUCT_ID)
    if dev is None:
        sys.exit("FLIR ONE not found on USB bus")
    return dev


def wait_for_device(timeout_s=20):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        dev = usb.core.find(idVendor=VENDOR_ID, idProduct=PRODUCT_ID)
        if dev is not None:
            return dev
        time.sleep(0.5)
    sys.exit("device did not come back after reset")


def open_stream(dev):
    dev.set_configuration(3)
    for intf in (0, 1, 2):
        usb.util.claim_interface(dev, intf)
    try:
        set_interface(dev, 2, 0)
        set_interface(dev, 1, 0)
        set_interface(dev, 1, 1)
        set_interface(dev, 2, 1, [0, 0])
    except usb.core.USBError as e:
        print(f"control transfer failed ({e}), resetting device and retrying...")
        try:
            dev.reset()
        except usb.core.USBError:
            pass
        dev = wait_for_device()
        dev.set_configuration(3)
        for intf in (0, 1, 2):
            usb.util.claim_interface(dev, intf)
        set_interface(dev, 2, 0)
        set_interface(dev, 1, 0)
        set_interface(dev, 1, 1)
        set_interface(dev, 2, 1, [0, 0])
    return dev


def parse_thermal(thermal_raw):
    body = thermal_raw[PIXEL_OFFSET:]
    n = RAW_W * RAW_H
    raw = np.frombuffer(body[: n * 2], dtype="<u2").reshape(RAW_H, RAW_W)
    return raw[0:FRAME_H, 1 : 1 + FRAME_W].astype(np.float64)


class SharedState:
    def __init__(self):
        self.lock = threading.Lock()
        self.thermal_jpg = None
        self.visible_jpg = None
        self.gen = 0  # bumped every time a new frame pair is published
        self.fps = 0.0
        self.frame_count = 0
        self.connected = False
        self.last_error = None
        self.raw_pix = None  # latest raw 80x60 float array, for calibration capture

    def publish(self, thermal_jpg, visible_jpg, fps, frame_count, raw_pix):
        with self.lock:
            self.thermal_jpg = thermal_jpg
            self.visible_jpg = visible_jpg
            self.fps = fps
            self.frame_count = frame_count
            self.raw_pix = raw_pix
            self.gen += 1

    def raw_snapshot(self):
        with self.lock:
            return self.raw_pix, self.frame_count

    def snapshot(self):
        with self.lock:
            return self.thermal_jpg, self.visible_jpg, self.gen


state = SharedState()


def load_font(size):
    for path in (
        "/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf",
    ):
        try:
            return ImageFont.truetype(path, size)
        except OSError:
            continue
    return ImageFont.load_default()


FONT = load_font(16)
FONT_SM = load_font(13)


def render_thermal(pix, fps, frame_count, status_text):
    p2, p98 = np.percentile(pix, [2, 98])
    clipped = np.clip(pix, p2, p98)
    norm = ((clipped - p2) / max(1.0, (p98 - p2)) * 255).astype(np.uint8)

    small = Image.fromarray(norm, mode="L")
    colorized = ImageOps.colorize(small, black=(10, 0, 60), mid=(230, 80, 0), white=(255, 255, 180), midpoint=180)
    img = colorized.resize(DISPLAY_SIZE, Image.LANCZOS).convert("RGB")

    celsius = raw_to_celsius(pix)
    min_idx = np.unravel_index(np.argmin(celsius), celsius.shape)
    max_idx = np.unravel_index(np.argmax(celsius), celsius.shape)
    cy, cx = FRAME_H // 2, FRAME_W // 2
    min_c, max_c, center_c, avg_c = celsius[min_idx], celsius[max_idx], celsius[cy, cx], celsius.mean()

    sx, sy = DISPLAY_SIZE[0] / FRAME_W, DISPLAY_SIZE[1] / FRAME_H

    def to_disp(idx):
        return int(idx[1] * sx + sx / 2), int(idx[0] * sy + sy / 2)

    draw = ImageDraw.Draw(img)

    def crosshair(pt, color, label):
        x, y = pt
        r = 8
        draw.line((x - r, y, x + r, y), fill=color, width=2)
        draw.line((x, y - r, x, y + r), fill=color, width=2)
        draw.ellipse((x - r, y - r, x + r, y + r), outline=color, width=2)
        label_w = draw.textlength(label, font=FONT_SM)
        tx = x + r + 4 if x + r + 4 + label_w < DISPLAY_SIZE[0] else x - r - 4 - label_w
        ty = y - r - 4 if y - r - 4 > 0 else y + r + 4
        draw.text((tx, ty), label, font=FONT_SM, fill=color, stroke_width=2, stroke_fill=(0, 0, 0))

    crosshair(to_disp(max_idx), (255, 60, 60), f"{max_c:.1f}C")
    crosshair(to_disp(min_idx), (80, 160, 255), f"{min_c:.1f}C")

    # center crosshair (small, no label)
    ccx, ccy = to_disp((cy, cx))
    draw.line((ccx - 5, ccy, ccx + 5, ccy), fill=(255, 255, 255), width=1)
    draw.line((ccx, ccy - 5, ccx, ccy + 5), fill=(255, 255, 255), width=1)

    def text(pos, s, font=FONT, fill=(255, 255, 255)):
        draw.text(pos, s, font=font, fill=fill, stroke_width=2, stroke_fill=(0, 0, 0))

    text((8, 6), "FLIR ONE  LIVE", fill=(255, 220, 120))
    text((8, 26), f"FPS {fps:4.1f}   frame #{frame_count}")
    text((8, DISPLAY_SIZE[1] - 96), f"MAX  {max_c:5.1f}C", fill=(255, 120, 120))
    text((8, DISPLAY_SIZE[1] - 76), f"MIN  {min_c:5.1f}C", fill=(120, 180, 255))
    text((8, DISPLAY_SIZE[1] - 56), f"CTR  {center_c:5.1f}C", fill=(255, 255, 255))
    text((8, DISPLAY_SIZE[1] - 36), f"AVG  {avg_c:5.1f}C", fill=(220, 220, 220))
    cal_status = calibration.status()
    if cal_status["active"]:
        label = f"calibrated ({len(cal_status['points'])} pts)"
        color = (140, 255, 140)
    else:
        label = "approx, uncalibrated"
        color = (180, 180, 180)
    text((8, DISPLAY_SIZE[1] - 16), label, font=FONT_SM, fill=color)

    try:
        st = json.loads(status_text)
        status_short = f"shutter={st.get('shutterState','?')} ffc={st.get('ffcState','?')}"
    except (json.JSONDecodeError, ValueError):
        status_short = status_text[:40]
    text((DISPLAY_SIZE[0] - 8 - draw.textlength(status_short, font=FONT_SM), 6), status_short,
         font=FONT_SM, fill=(180, 220, 180))

    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=85)
    return buf.getvalue()


def render_visible(jpg_bytes):
    img = Image.open(io.BytesIO(jpg_bytes)).convert("RGB")
    img.thumbnail(DISPLAY_SIZE, Image.LANCZOS)
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=85)
    return buf.getvalue()


def capture_loop():
    frame_count = 0
    fps_window = []
    while True:
        try:
            dev = find_device()
            dev = open_stream(dev)
            state.connected = True
            state.last_error = None
            print("device stream opened")
        except SystemExit as e:
            state.last_error = str(e)
            time.sleep(2)
            continue

        acc = bytearray()
        try:
            while True:
                for ep, to in ((0x81, 10), (0x83, 10)):
                    try:
                        dev.read(ep, 1 << 16, timeout=to)
                    except usb.core.USBError:
                        pass
                try:
                    chunk = bytes(dev.read(0x85, 1 << 16, timeout=100))
                except usb.core.USBError as e:
                    if e.errno == 19:
                        raise
                    continue

                if chunk.startswith(MAGIC) or (len(acc) + len(chunk)) >= BUF_LIMIT:
                    acc = bytearray()
                acc += chunk
                if not acc.startswith(MAGIC):
                    acc = bytearray()
                    continue
                if len(acc) < 24:
                    continue
                frame_size, thermal_size, jpg_size, status_size = struct.unpack_from("<IIII", acc, 8)
                if frame_size + 28 > len(acc):
                    continue

                thermal_raw = bytes(acc[: 28 + thermal_size])
                jpg_bytes = bytes(acc[28 + thermal_size : 28 + thermal_size + jpg_size])
                status = bytes(acc[28 + thermal_size + jpg_size : 28 + thermal_size + jpg_size + status_size])
                acc = bytearray()

                status_text = "".join(chr(b) for b in status if b >= 32)

                now = time.time()
                fps_window.append(now)
                fps_window[:] = [t for t in fps_window if now - t < 2.0]
                fps = (len(fps_window) - 1) / max(1e-6, (fps_window[-1] - fps_window[0])) if len(fps_window) > 1 else 0.0

                frame_count += 1
                try:
                    pix = parse_thermal(thermal_raw)
                    thermal_jpg = render_thermal(pix, fps, frame_count, status_text)
                    visible_jpg = render_visible(jpg_bytes)
                    state.publish(thermal_jpg, visible_jpg, fps, frame_count, pix)
                except Exception as e:
                    print("render error:", e)
        except usb.core.USBError as e:
            print("stream error, reconnecting:", e)
            state.connected = False
            try:
                set_interface(dev, 2, 0)
            except usb.core.USBError:
                pass
            usb.util.dispose_resources(dev)
            time.sleep(1)


HTML_PAGE = """<!doctype html>
<html><head><title>FLIR ONE Live</title>
<style>
body { background:#111; color:#eee; font-family: system-ui, sans-serif; margin:0; padding:16px; }
h1 { font-size: 16px; font-weight: 600; color:#ffcf87; }
.row { display:flex; gap:16px; flex-wrap:wrap; }
img { max-width: 640px; width:100%; border-radius:8px; border:1px solid #333; background:#000; }
.card { flex: 1 1 320px; }
.label { font-size: 12px; color:#999; margin-bottom:4px; }
.panel { margin-top:20px; padding:14px 16px; background:#1a1a1a; border:1px solid #333; border-radius:8px; max-width: 900px; }
.panel h2 { font-size:13px; text-transform:uppercase; letter-spacing:.04em; color:#ffcf87; margin:0 0 10px; }
.form-row { display:flex; gap:8px; align-items:center; flex-wrap:wrap; margin-bottom:10px; }
input[type=text], input[type=number] { background:#0d0d0d; border:1px solid #444; color:#eee; padding:6px 8px; border-radius:5px; font-size:14px; }
input[type=number] { width:90px; }
select { background:#0d0d0d; border:1px solid #444; color:#eee; padding:6px 4px; border-radius:5px; }
button { background:#3a3a3a; color:#eee; border:1px solid #555; padding:7px 14px; border-radius:5px; cursor:pointer; font-size:14px; }
button:hover { background:#4a4a4a; }
button.primary { background:#8a5a1f; border-color:#a56a25; }
button.primary:hover { background:#a56a25; }
table { width:100%; border-collapse: collapse; font-size:13px; margin-top:8px; }
th, td { text-align:left; padding:5px 8px; border-bottom:1px solid #2a2a2a; }
th { color:#999; font-weight:500; }
.fit { font-size:13px; color:#9fd89f; margin-top:8px; min-height:18px; }
.hint { font-size:12px; color:#888; margin-top:2px; }
.del { background:#5a2a2a; border-color:#7a3a3a; padding:2px 8px; font-size:12px; }
</style></head>
<body>
<h1>FLIR ONE — Live View</h1>
<div class="row">
  <div class="card">
    <div class="label">Thermal</div>
    <img src="/thermal.mjpg">
  </div>
  <div class="card">
    <div class="label">Visible</div>
    <img src="/visible.mjpg">
  </div>
</div>

<div class="panel">
  <h2>Calibration</h2>
  <div class="form-row">
    <input type="text" id="label" placeholder="label (e.g. mouth)" size="16">
    <input type="number" id="knownTemp" placeholder="known temp" step="0.1">
    <select id="unit"><option value="C">&deg;C</option><option value="F">&deg;F</option></select>
    <button class="primary" id="captureBtn">Capture (Space)</button>
    <button id="clearBtn">Clear all</button>
  </div>
  <div class="hint">Center the target under the small white crosshair, enter its known temperature, then press Space or click Capture. Uses the current center-pixel raw reading.</div>
  <div class="fit" id="fitLine"></div>
  <table id="pointsTable">
    <thead><tr><th>#</th><th>Label</th><th>Raw</th><th>Known</th><th></th></tr></thead>
    <tbody id="pointsBody"></tbody>
  </table>
</div>

<script>
async function refreshCalibration() {
  const r = await fetch('/calibration.json');
  const cal = await r.json();
  const body = document.getElementById('pointsBody');
  body.innerHTML = '';
  cal.points.forEach((p, i) => {
    const tr = document.createElement('tr');
    tr.innerHTML = `<td>${i}</td><td>${p.label || ''}</td><td>${p.raw.toFixed(0)}</td>` +
      `<td>${p.known_c.toFixed(1)}&deg;C</td><td><button class="del" data-i="${i}">x</button></td>`;
    body.appendChild(tr);
  });
  body.querySelectorAll('.del').forEach(b => b.onclick = async () => {
    await fetch('/calibration/' + b.dataset.i, {method: 'DELETE'});
    refreshCalibration();
  });
  const fitEl = document.getElementById('fitLine');
  if (cal.active) {
    fitEl.textContent = `Fit active: C = ${cal.slope.toFixed(5)} * raw + ${cal.intercept.toFixed(2)}  (${cal.points.length} points)`;
  } else {
    fitEl.textContent = cal.points.length === 1
      ? 'Need one more point to compute a fit.'
      : 'No calibration points yet -- using generic (uncalibrated) constants.';
  }
}

async function capture() {
  const label = document.getElementById('label').value;
  const val = parseFloat(document.getElementById('knownTemp').value);
  if (isNaN(val)) { alert('Enter a known temperature first'); return; }
  const unit = document.getElementById('unit').value;
  const known_c = unit === 'F' ? (val - 32) * 5 / 9 : val;
  const raw = await (await fetch('/raw.json')).json();
  if (raw.error) { alert('No frame available yet'); return; }
  await fetch('/calibration/add', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({label, raw: raw.center_raw, known_c})
  });
  refreshCalibration();
}

document.getElementById('captureBtn').onclick = capture;
document.getElementById('clearBtn').onclick = async () => {
  await fetch('/calibration/clear', {method: 'POST'});
  refreshCalibration();
};
document.addEventListener('keydown', (e) => {
  const tag = (document.activeElement && document.activeElement.tagName) || '';
  if (e.code === 'Space' && tag !== 'INPUT' && tag !== 'TEXTAREA' && tag !== 'SELECT') {
    e.preventDefault();
    capture();
  }
});

refreshCalibration();
setInterval(refreshCalibration, 4000);
</script>
</body></html>
"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass

    def do_GET(self):
        if self.path == "/":
            body = HTML_PAGE.encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif self.path in ("/thermal.mjpg", "/visible.mjpg"):
            which = "thermal" if "thermal" in self.path else "visible"
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            last_gen = -1
            try:
                while True:
                    t_jpg, v_jpg, gen = state.snapshot()
                    jpg = t_jpg if which == "thermal" else v_jpg
                    if jpg is not None and gen != last_gen:
                        last_gen = gen
                        self.wfile.write(b"--frame\r\n")
                        self.wfile.write(b"Content-Type: image/jpeg\r\n")
                        self.wfile.write(f"Content-Length: {len(jpg)}\r\n\r\n".encode())
                        self.wfile.write(jpg)
                        self.wfile.write(b"\r\n")
                    time.sleep(0.03)
            except (BrokenPipeError, ConnectionResetError):
                pass
        elif self.path == "/raw.json":
            pix, frame_count = state.raw_snapshot()
            if pix is None:
                payload = json.dumps({"error": "no frame yet"}).encode()
                self.send_response(503)
            else:
                cy, cx = FRAME_H // 2, FRAME_W // 2
                min_idx = np.unravel_index(np.argmin(pix), pix.shape)
                max_idx = np.unravel_index(np.argmax(pix), pix.shape)
                payload = json.dumps({
                    "frame_count": frame_count,
                    "center_raw": float(pix[cy, cx]),
                    "min_raw": float(pix[min_idx]),
                    "max_raw": float(pix[max_idx]),
                    "min_xy": [int(min_idx[1]), int(min_idx[0])],
                    "max_xy": [int(max_idx[1]), int(max_idx[0])],
                }).encode()
                self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)
        elif self.path == "/calibration.json":
            self._send_json(calibration.status())
        else:
            self.send_response(404)
            self.end_headers()

    def _read_json_body(self):
        length = int(self.headers.get("Content-Length", 0))
        raw = self.rfile.read(length) if length else b"{}"
        return json.loads(raw or b"{}")

    def _send_json(self, obj, code=200):
        payload = json.dumps(obj, default=float).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def do_POST(self):
        if self.path == "/calibration/add":
            body = self._read_json_body()
            try:
                label = str(body.get("label", ""))[:40]
                raw = float(body["raw"])
                known_c = float(body["known_c"])
            except (KeyError, TypeError, ValueError):
                self._send_json({"error": "expected {label, raw, known_c}"}, 400)
                return
            calibration.add(label, raw, known_c)
            self._send_json(calibration.status())
        elif self.path == "/calibration/clear":
            calibration.clear()
            self._send_json(calibration.status())
        else:
            self.send_response(404)
            self.end_headers()

    def do_DELETE(self):
        if self.path.startswith("/calibration/"):
            try:
                index = int(self.path.rsplit("/", 1)[-1])
            except ValueError:
                self._send_json({"error": "bad index"}, 400)
                return
            calibration.remove(index)
            self._send_json(calibration.status())
        else:
            self.send_response(404)
            self.end_headers()


def main():
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8765
    t = threading.Thread(target=capture_loop, daemon=True)
    t.start()
    server = ThreadingHTTPServer(("127.0.0.1", port), Handler)
    print(f"Serving on http://127.0.0.1:{port}")
    server.serve_forever()


if __name__ == "__main__":
    main()
