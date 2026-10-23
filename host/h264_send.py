#!/usr/bin/env python3
"""Send an H.264 video from the PC to the PolarFire decoder (serial or Ethernet).

    h264_send.py --list
    h264_send.py -p COM5 video.264                 send an Annex-B Baseline stream
    h264_send.py -p /dev/ttyUSB2 film.mp4 --size 1280x720 --bitrate 1.2M
                                                   convert with ffmpeg on the fly
    h264_send.py -p udp://192.168.10.2 film.mp4 --size 1920x1080 --bitrate 8M
                                                   over Ethernet (kit/ firmware)
    ffmpeg -i cam.mp4 ... -f h264 - | h264_send.py -p udp://192.168.10.2 -
                                                   stream from a pipe
    h264_send.py -p COM5 --status                  decoder status only

Files that are not a Constrained Baseline Annex-B stream (.mp4, .mkv,
High profile .264, ...) are converted with ffmpeg (must be in PATH) while
they are sent.  The serial link carries about baud * 0.8 bit/s (2 Mbaud:
1.6 Mbit/s); a stream with a higher bit rate still decodes correctly, only
slower than real time.  Serial ports need pyserial (pip install pyserial).
"""
import argparse
import os
import sys
import time

import h264link as hl


def list_ports():
    from serial.tools import list_ports as lp
    ports = list(lp.comports())
    if not ports:
        print("no serial ports found")
    for p in ports:
        print("%-16s %s" % (p.device, p.description))


def human(n):
    for unit in ("B", "KB", "MB", "GB"):
        if n < 1024 or unit == "GB":
            return "%.1f %s" % (n, unit) if unit != "B" else "%d B" % n
        n /= 1024.0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("input", nargs="?", help="video file (.264 / .h264 or anything ffmpeg reads), - = stdin")
    ap.add_argument("-p", "--port", help="serial port (COM5, /dev/ttyUSB0, ...) or udp://IP[:PORT]")
    ap.add_argument("-b", "--baud", type=int, default=2000000, help="baud rate (default 2000000, must match G_BAUD)")
    ap.add_argument("--list", action="store_true", help="list serial ports")
    ap.add_argument("--status", action="store_true", help="only print the decoder status")
    ap.add_argument("--convert", action="store_true", help="always convert with ffmpeg")
    ap.add_argument("--raw", action="store_true", help="never convert, send the file as it is")
    ap.add_argument("--size", help="output size when converting, e.g. 1280x720")
    ap.add_argument("--bitrate", help="bit rate when converting, e.g. 1.2M (default: fits the link)")
    ap.add_argument("--fps", type=float, help="frame rate when converting")
    ap.add_argument("--no-wait", action="store_true", help="do not wait for the decoder after END")
    a = ap.parse_args()

    if a.list:
        list_ports()
        return 0
    if not a.port:
        ap.error("--port is required (see --list)")
    port, baud = hl.open_port(a.port, a.baud)
    snd = hl.Sender(port, baud, log=lambda m: print("\n" + m))
    try:
        if a.status:
            print(snd.status().describe())
            return 0
        if not a.input:
            ap.error("input file missing")
        convert = a.convert and a.input != "-"
        if not a.raw and not convert and a.input != "-":
            ext = os.path.splitext(a.input)[1].lower()
            prof = hl.sps_profile(a.input) if ext in (".264", ".h264", ".avc", ".bin") else None
            if prof is None:
                convert = True
                print("not an Annex-B H.264 file: converting with ffmpeg")
            elif prof[0] != 66:
                convert = True
                print("profile_idc %d (not Baseline): converting with ffmpeg" % prof[0])
        if convert:
            size = tuple(int(v) for v in a.size.lower().split("x")) if a.size else None
            br = hl.parse_bitrate(a.bitrate) or hl.auto_bitrate(baud)
            print("ffmpeg: Constrained Baseline, %s, %.2f Mbit/s" % (a.size or "original size", br / 1e6))
            chunks = hl.ffmpeg_chunks(a.input, size, br, a.fps)
            total = None
        else:
            chunks = hl.read_chunks(a.input)
            total = os.path.getsize(a.input) if a.input != "-" else None

        r = snd.start()
        print("link up: FIFO %s, decoder reset" % human(r.free + 1))

        def progress(done, last, dt):
            rate = done * 8 / dt / 1e6 if dt > 0 else 0
            pics = last.pictures if last else 0
            pct = " %5.1f%%" % (100.0 * done / total) if total else ""
            sys.stdout.write("\r%s sent%s  %.2f Mbit/s  pictures %d   " % (human(done), pct, rate, pics))
            sys.stdout.flush()

        t0 = time.monotonic()
        n = snd.send(chunks, progress)
        snd.end()
        dt = time.monotonic() - t0
        print("\n%s in %.1f s (%.2f Mbit/s); frames %d, retransmitted %d, NAK %d, timeouts %d" % (
            human(n), dt, n * 8 / dt / 1e6 if dt else 0, snd.stats["frames"], snd.stats["retx"],
            snd.stats["nak"], snd.stats["timeout"]))
        if not a.no_wait:
            r = snd.wait_done(timeout=30, progress=lambda r: sys.stdout.write("\rpictures %d   " % r.pictures))
            print("\n" + r.describe())
            if r.error:
                return 2
        return 0
    except hl.LinkError as e:
        print("\nerror:", e)
        return 1
    except KeyboardInterrupt:
        print("\nstopped")
        return 1
    finally:
        port.close()


if __name__ == "__main__":
    sys.exit(main())
