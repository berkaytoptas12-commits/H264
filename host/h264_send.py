#!/usr/bin/env python3
"""Send an H.264 video from the PC to the PolarFire decoder over Ethernet.

    h264_send.py video.264                         send an Annex-B Baseline stream to the kit
                                                   (kit/ firmware, default 192.168.10.2)
    h264_send.py --ip 192.168.1.50 film.mp4 --size 1920x1080 --bitrate 8M
                                                   convert with ffmpeg on the fly
    ffmpeg -i cam.mp4 ... -f h264 - | h264_send.py -
                                                   stream from a pipe
    h264_send.py --status                          decoder status only
    h264_send.py --info                            kit firmware status (fabric found, counters)
    h264_send.py --console                         follow the kit's console (over Ethernet)

Files that are not a Constrained Baseline Annex-B stream (.mp4, .mkv,
High profile .264, ...) are converted with ffmpeg (must be in PATH) while
they are sent.  The optional UART link (rtl/h264_dec_uart_top.vhd) is used
with --serial PORT [--baud N] instead (needs pyserial).
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


def display(ip, port, fps, flags):
    """HDMI display settings through the kit firmware"""
    try:
        c = hl.KitConsole(ip, port)
        r = c.set_display(fps, flags)
        c.close()
    except OSError:
        r = None
    if r is None or r == 0xFFFFFFFF:
        print("warning: display settings not accepted (no answer from the kit firmware / fabric)")
        return 1
    print("display: %.2f fps%s" % ((r >> 16) / 100.0, ", colour bars" if r & 2 else ""))
    return 0


def kit_console(ip, port, follow):
    try:
        c = hl.KitConsole(ip, port)
    except OSError as e:
        print("error: %s: %s" % (ip, e))
        return 1
    if not follow:
        d = c.info()
        if d is None:
            print("no answer from the kit at %s (UDP %d)" % (ip, port))
            return 1
        for k, v in d.items():
            print("%-14s %s" % (k, v))
        return 0
    print("kit console %s, UDP %d (Ctrl-C ends)" % (ip, port))
    silent = False
    try:
        while True:
            res = c.read()
            if res is None:
                if not silent:
                    print("... no answer from the kit")
                silent = True
            else:
                silent = False
                text, note = res
                if note:
                    print("-- kit restarted --" if note == "restart" else "...")
                sys.stdout.write(text)
                sys.stdout.flush()
            time.sleep(0.5)
    except KeyboardInterrupt:
        return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("input", nargs="?", help="video file (.264 / .h264 or anything ffmpeg reads), - = stdin")
    ap.add_argument("--ip", default=os.environ.get("H264_KIT_IP", "192.168.10.2"),
                    help="IP address of the kit (default 192.168.10.2 or $H264_KIT_IP)")
    ap.add_argument("--port", type=int, default=hl.UDP_PORT, help="UDP port (default %d)" % hl.UDP_PORT)
    ap.add_argument("--status", action="store_true", help="only print the decoder status")
    ap.add_argument("--info", action="store_true", help="print the kit firmware status")
    ap.add_argument("--console", action="store_true", help="follow the kit console (Ctrl-C ends)")
    ap.add_argument("--convert", action="store_true", help="always convert with ffmpeg")
    ap.add_argument("--raw", action="store_true", help="never convert, send the file as it is")
    ap.add_argument("--size", help="output size when converting, e.g. 1280x720")
    ap.add_argument("--bitrate", help="bit rate when converting, e.g. 1.2M (default: fits the link)")
    ap.add_argument("--fps", type=float, help="frame rate when converting and for the HDMI display "
                    "(default: from the file, else 30)")
    ap.add_argument("--bars", action="store_true", help="show colour bars on HDMI (link test) and exit")
    ap.add_argument("--bitrev", action="store_true", help="HDMI lanes with reversed bit order "
                    "(try when the monitor shows nothing)")
    ap.add_argument("--no-wait", action="store_true", help="do not wait for the decoder after END")
    uart = ap.add_argument_group("optional UART link instead of Ethernet")
    uart.add_argument("--serial", metavar="PORT", help="serial port (COM5, /dev/ttyUSB0, ...)")
    uart.add_argument("-b", "--baud", type=int, default=2000000, help="baud rate (default 2000000, = G_BAUD)")
    uart.add_argument("--list", action="store_true", help="list serial ports")
    a = ap.parse_args()

    if a.list:
        list_ports()
        return 0
    if a.info or a.console or a.bars:
        if a.serial:
            ap.error("--info / --console / --bars work over Ethernet only")
        if a.bars:
            return display(a.ip, a.port + 1, a.fps or 30.0, 2 | (4 if a.bitrev else 0))
        return kit_console(a.ip, a.port + 1, a.console)
    try:
        if a.serial:
            port, baud = hl.open_port(a.serial, a.baud)
        else:
            port, baud = hl.UdpPort(a.ip, a.port), hl.UDP_BAUD
    except OSError as e:
        print("error: cannot open %s: %s" % (a.serial or "%s:%d" % (a.ip, a.port), e))
        return 1
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

        if not a.serial:
            display(a.ip, a.port + 1, a.fps or hl.video_fps(a.input) or 30.0, 4 if a.bitrev else 0)
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
