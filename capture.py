#!/usr/bin/env python3
"""
Grab one frame (visible JPEG + raw 16-bit thermal) from a FLIR ONE camera
over USB bulk transfers, using the reverse-engineered protocol documented in
https://github.com/fnoop/flirone-v4l2 (src/flirone.c).

Vendor ID 0x09cb / Product ID 0x1996 = FLIR ONE.
"""
import struct
import sys
import time

import numpy as np
import usb.core
import usb.util
from PIL import Image

VENDOR_ID = 0x09CB
PRODUCT_ID = 0x1996

DEBUG = False
MAGIC = bytes([0xEF, 0xBE, 0x00, 0x00])
BUF_LIMIT = 1 << 20  # 1 MiB, matches BUF85SIZE in the reference implementation

# Layout determined empirically (this device's firmware differs from the older
# "FLIR ONE G2" protocol the reference C code targeted): within thermal_raw
# (which starts with the 28-byte outer frame header), pixel data is a plain
# 82x62 row-major uint16-LE grid starting at byte 30. Column 0, the last
# column, and the last 2 rows are telemetry/dark-reference pixels -- cropping
# them leaves the native 80x60 Lepton sensor image.
RAW_W, RAW_H = 82, 62
PIXEL_OFFSET = 30
FRAME_W, FRAME_H = 80, 60


def set_interface(dev, interface, alt, extra=None):
    # bmRequestType 0x01 = OUT | Standard | Recipient=Interface
    # bRequest 0x0B = SET_INTERFACE (standard USB request)
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

    # Sequence lifted directly from flirone-v4l2's EPloop() state machine.
    try:
        set_interface(dev, 2, 0)  # stop interface 2 (FRAME)
        set_interface(dev, 1, 0)  # stop interface 1 (FILEIO)
        set_interface(dev, 1, 1)  # start interface 1 (FILEIO) -- unused here
        set_interface(dev, 2, 1, [0, 0])  # start interface 2 -> begins streaming on EP 0x85
    except usb.core.USBError as e:
        # A previous run may have left the device mid-stream and unresponsive
        # to new control requests. A port reset clears that, mirroring
        # libusb_reset_device() in the reference C code -- but it triggers a
        # full re-enumeration, so we must reacquire a fresh device handle.
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
    return raw[0:FRAME_H, 1 : 1 + FRAME_W].astype(np.uint16)  # crop telemetry border


def raw_to_celsius(raw):
    # Generic Planck constants pulled from a sample FLIR ONE EXIF blob
    # (plank.h in flirone-v4l2) -- NOT calibrated for this specific unit,
    # so treat absolute values as rough estimates only.
    PlanckR1, PlanckB, PlanckF, PlanckO, PlanckR2 = 16528.178, 1427.5, 1.0, -1307.0, 0.012258549
    TempReflected, Emissivity = 20.0, 0.95
    raw = raw.astype(np.float64) * 4
    raw_refl = PlanckR1 / (PlanckR2 * (np.exp(PlanckB / (TempReflected + 273.15)) - PlanckF)) - PlanckO
    raw_obj = (raw - (1 - Emissivity) * raw_refl) / Emissivity
    return PlanckB / np.log(PlanckR1 / (PlanckR2 * (raw_obj + PlanckO)) + PlanckF) - 273.15


def capture_frame(dev, timeout_s=15):
    # The reference implementation polls EP 0x81 and 0x83 on every loop
    # iteration too, even though we only care about 0x85's frame data --
    # leaving them unread seems to stall the whole vendor-specific protocol.
    acc = bytearray()
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        for ep, to in ((0x81, 10), (0x83, 10)):
            try:
                dev.read(ep, 1 << 16, timeout=to)
            except usb.core.USBError:
                pass

        try:
            chunk = dev.read(0x85, 1 << 16, timeout=100)
        except usb.core.USBError as e:
            if e.errno == 19:  # ENODEV, device gone
                raise
            # timeouts and transient EIO/EPIPE happen while the stream spins up;
            # the reference implementation just logs and keeps polling.
            if DEBUG:
                sys.stderr.write(f"85err:{e} ")
            continue
        chunk = bytes(chunk)
        if DEBUG:
            sys.stderr.write(f"85ok:{len(chunk)} ")

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
            continue  # need more chunks

        thermal_raw = bytes(acc[: 28 + thermal_size])
        jpg_bytes = bytes(acc[28 + thermal_size : 28 + thermal_size + jpg_size])
        status = bytes(acc[28 + thermal_size + jpg_size : 28 + thermal_size + jpg_size + status_size])
        acc = bytearray()
        yield thermal_raw, jpg_bytes, status

    return


def capture_good_frame(dev, overall_timeout_s=20):
    """Skip frames sent during the power-on FFC (flat-field correction)
    shutter calibration cycle."""
    deadline = time.time() + overall_timeout_s
    for thermal_raw, jpg_bytes, status in capture_frame(dev, timeout_s=overall_timeout_s):
        status_text = "".join(chr(b) for b in status if b >= 32)
        if '"shutterState":"FFC"' not in status_text:
            return thermal_raw, jpg_bytes, status
        print(f"  skipping FFC frame (thermal={len(thermal_raw)}B)")
        if time.time() > deadline:
            break
    sys.exit("never got a non-FFC frame in time")


def main():
    dev = find_device()
    print("Found FLIR ONE")
    try:
        dev = open_stream(dev)
        print("Streaming started, waiting for a frame...")

        thermal_raw, jpg_bytes, status = capture_good_frame(dev)
        print(f"Got frame: thermal={len(thermal_raw)}B jpg={len(jpg_bytes)}B status={len(status)}B")
        print("Status text:", "".join(chr(b) for b in status if b >= 32))

        with open("visible.jpg", "wb") as f:
            f.write(jpg_bytes)
        print("Wrote visible.jpg")

        pix = parse_thermal(thermal_raw).astype(np.float64)
        np.save("thermal_raw.npy", pix)

        p2, p98 = np.percentile(pix, [2, 98])
        clipped = np.clip(pix, p2, p98)
        norm = ((clipped - p2) / max(1.0, (p98 - p2)) * 255).astype(np.uint8)

        Image.fromarray(norm, mode="L").resize((640, 480), Image.LANCZOS).save("thermal_gray.png")
        print("Wrote thermal_gray.png")

        from PIL import ImageOps

        colorized = ImageOps.colorize(
            Image.fromarray(norm, mode="L"), black=(10, 0, 60), mid=(230, 80, 0), white=(255, 255, 180), midpoint=180
        )
        colorized.resize((640, 480), Image.LANCZOS).save("thermal_color.png")
        print("Wrote thermal_color.png")

        celsius = raw_to_celsius(pix)
        cy, cx = FRAME_H // 2, FRAME_W // 2
        print(f"Approx temps (uncalibrated): min={celsius.min():.1f}C max={celsius.max():.1f}C center={celsius[cy,cx]:.1f}C")
    finally:
        try:
            set_interface(dev, 2, 0)  # stop streaming so the device doesn't
        except usb.core.USBError:      # get wedged for the next run
            pass
        usb.util.dispose_resources(dev)


if __name__ == "__main__":
    main()
