"""PC side of the H.264 stream link to the PolarFire decoder (rtl/h264_link.vhd).

Frame (PC -> FPGA):
    55 AA | type | seq | len (2, LE, <= 1024) | payload | crc16 (2, LE)
Response (FPGA -> PC, 16 bytes):
    5A A5 | type | seq | free (2) | pictures (4) | flags | w/h (3) | crc16 (2)
crc16 = CCITT (poly 0x1021, init 0xFFFF, no reflection) over type .. payload
(responses: type .. byte 13).

The sender keeps up to WINDOW frames in flight, never more payload than the
FIFO space (credit) the last response reported, retransmits everything from
the expected frame on a NAK (go-back-N) and after a timeout.

Transports: UDP (Ethernet through the MSS firmware in kit/, one frame per
datagram, rtl/h264_dec_mss_top.vhd) or a serial port (optional UART link,
rtl/h264_dec_uart_top.vhd; also used by the tests).
"""
import binascii
import collections
import select
import shutil
import socket
import struct
import subprocess
import sys
import time

T_DATA, T_END, T_START, T_STATUS = 0x01, 0x02, 0x03, 0x04
R_ACK, R_NAK, R_STATUS = 0x81, 0x82, 0x83
MAXLEN = 1024
WINDOW = 16
UDP_PORT = 5000               # link frames; the kit's console answers on UDP_PORT + 1
UDP_BAUD = 100000000          # timing model of the Ethernet path (~80 Mbit/s payload)
AUTO_BITRATE_MAX = 8000000    # default ffmpeg bit rate on fast links

F_ERROR, F_DONE, F_END, F_EMPTY = 0x01, 0x02, 0x04, 0x08


def crc16(data, crc=0xFFFF):
    """CRC-16/CCITT, poly 0x1021, no reflection (binascii runs it in C)"""
    return binascii.crc_hqx(bytes(data), crc)


def build_frame(ftype, seq, payload=b""):
    if len(payload) > MAXLEN:
        raise ValueError("payload too long")
    body = bytes([ftype, seq & 0xFF]) + struct.pack("<H", len(payload)) + bytes(payload)
    return b"\x55\xAA" + body + struct.pack("<H", crc16(body))


class Response(collections.namedtuple("Response", "rtype seq free pictures flags width height")):
    @property
    def error(self):
        return bool(self.flags & F_ERROR)

    @property
    def done(self):
        return bool(self.flags & F_DONE)

    @property
    def empty(self):
        return bool(self.flags & F_EMPTY)

    def describe(self):
        names = {R_ACK: "ACK", R_NAK: "NAK", R_STATUS: "STATUS"}
        st = []
        if self.error:
            st.append("decoder error")
        if self.flags & F_END:
            st.append("END received")
        if self.done:
            st.append("stream done")
        return "%s seq=%d free=%d pictures=%d %dx%d %s" % (
            names.get(self.rtype, hex(self.rtype)), self.seq, self.free, self.pictures,
            self.width, self.height, ", ".join(st))


def build_response(rtype, seq, free, pictures, flags, width, height):
    """FPGA side encoding (used by the emulator and the tests)."""
    body = bytes([rtype, seq & 0xFF]) + struct.pack("<HIB", min(free, 0xFFFF), pictures & 0xFFFFFFFF, flags)
    body += bytes([width & 0xFF, ((height & 0xF) << 4) | ((width >> 8) & 0xF), (height >> 4) & 0xFF])
    return b"\x5A\xA5" + body + struct.pack("<H", crc16(body))


class ResponseParser:
    """Byte stream -> Response objects (bad CRC responses are dropped)."""

    def __init__(self):
        self.buf = bytearray()
        self.bad = 0

    def feed(self, data):
        self.buf += data
        out = []
        while True:
            i = self.buf.find(b"\x5A\xA5")
            if i < 0:
                del self.buf[:max(0, len(self.buf) - 1)]
                return out
            del self.buf[:i]
            if len(self.buf) < 16:
                return out
            r = bytes(self.buf[:16])
            if struct.unpack("<H", r[14:16])[0] != crc16(r[2:14]):
                self.bad += 1
                del self.buf[:1]
                continue
            del self.buf[:16]
            free, pics, flags = struct.unpack("<HIB", r[4:11])
            width = r[11] | ((r[12] & 0xF) << 8)
            height = (r[12] >> 4) | (r[13] << 4)
            out.append(Response(r[2], r[3], free, pics, flags, width, height))


class LinkError(Exception):
    pass


def _before(a, b):
    """seq a comes before seq b (mod 256, window < 128)"""
    return 0 < ((b - a) & 0xFF) <= 128


class Sender:
    """Streams bytes to the decoder over a pyserial-like port
    (write(), read(n), in_waiting, timeout)."""

    def __init__(self, port, baud=2000000, log=None):
        self.port = port
        self.baud = baud
        # base retransmission timeout: a LAN answers in well under a millisecond
        self.udp = isinstance(port, UdpPort)
        self.rto = 0.03 if self.udp else 0.25
        self.parser = ResponseParser()
        self.last = None              # last response
        self.log = log or (lambda msg: None)
        self.stats = collections.Counter()
        self._stop = False

    # -- low level -----------------------------------------------------
    def _poll(self, wait):
        """read responses for up to 'wait' seconds (returns at the first one)"""
        end = time.monotonic() + wait
        got = []
        while True:
            n = self.port.in_waiting
            data = self.port.read(n if n else 1)
            if data:
                got += self.parser.feed(data)
                if got and not self.port.in_waiting:
                    break
            if time.monotonic() >= end:
                break
        for r in got:
            self.last = r
            self.stats["resp"] += 1
        return got

    def _byte_time(self):
        return 10.0 / self.baud

    def stop(self):
        self._stop = True

    def _command(self, ftype, seq, want, tries=6, timeout=0.3):
        frame = build_frame(ftype, seq)
        for _ in range(tries):
            self.port.write(frame)
            end = time.monotonic() + timeout
            while time.monotonic() < end:
                for r in self._poll(0.02):
                    if want(r):
                        return r
        if self.udp:
            raise LinkError("no answer from the kit at %s:%d (type %02x) - IP address, cable, firmware?"
                            % (self.port.addr + (ftype,)))
        raise LinkError("no answer from the FPGA (type %02x) - port, baud rate, cabling?" % ftype)

    # -- commands ------------------------------------------------------
    def start(self):
        """reset the decoder and the link; data frames start at seq 1"""
        self.port.reset_input_buffer()
        self.parser = ResponseParser()
        r = self._command(T_START, 0, lambda r: r.rtype == R_ACK and r.seq == 0)
        self.seq = 1
        return r

    def status(self, tries=6, timeout=0.3):
        return self._command(T_STATUS, 0, lambda r: r.rtype == R_STATUS, tries, timeout)

    def send(self, chunks, progress=None):
        """send the byte chunks (iterable of bytes); returns bytes sent"""
        self._stop = False
        pending = bytearray()
        it = iter(chunks)
        eof = False
        # frames not acknowledged yet: [seq, frame, length, sent]
        outstanding = collections.deque()
        free = self.last.free if self.last else 0
        acked_bytes = 0
        last_rx = time.monotonic()
        nak_hold = 0.0
        last_status = 0.0
        t0 = time.monotonic()

        while True:
            if self._stop:
                raise LinkError("stopped")
            while not eof and len(pending) < MAXLEN:
                try:
                    pending += next(it)
                except StopIteration:
                    eof = True
            if eof and not pending and not outstanding:
                break
            # credit: FIFO space minus the frames on the way
            avail = free - sum(o[2] for o in outstanding if o[3])
            for o in outstanding:          # retransmissions first, in order
                if o[3]:
                    continue
                if o[2] > avail:
                    break
                self.port.write(o[1])
                o[3] = True
                avail -= o[2]
                self.stats["retx"] += 1
            if all(o[3] for o in outstanding):
                while pending and len(outstanding) < WINDOW:
                    n = min(MAXLEN, len(pending))
                    if n > avail:
                        break
                    frame = build_frame(T_DATA, self.seq, bytes(pending[:n]))
                    del pending[:n]
                    self.port.write(frame)
                    outstanding.append([self.seq, frame, n, True])
                    self.seq = (self.seq + 1) & 0xFF
                    avail -= n
                    self.stats["frames"] += 1
            # responses
            for r in self._poll(0.005 if pending else 0.02):
                last_rx = time.monotonic()
                if r.rtype == R_NAK:
                    self.stats["nak"] += 1
                    while outstanding and _before(outstanding[0][0], r.seq):
                        acked_bytes += outstanding.popleft()[2]
                    if time.monotonic() >= nak_hold:
                        # go-back-N from the expected frame
                        for o in outstanding:
                            o[3] = False
                        nak_hold = time.monotonic() + self.rto / 5 + \
                            sum(o[2] for o in outstanding) * 1.2 * self._byte_time()
                else:
                    while outstanding and not _before(r.seq, outstanding[0][0]):
                        acked_bytes += outstanding.popleft()[2]
                free = r.free
                if r.error:
                    self.log("decoder reports a stream error (unsupported or corrupt stream)")
            now = time.monotonic()
            in_flight = sum(o[2] for o in outstanding if o[3])
            # timeout: send everything not acknowledged again
            if in_flight and now - last_rx > self.rto + in_flight * 2 * self._byte_time():
                for o in outstanding:
                    o[3] = False
                self.stats["timeout"] += 1
                last_rx = now
            # waiting for room in the FIFO with nothing in flight: ask for its level
            if not in_flight and (pending or outstanding) and now - last_status > 0.02:
                self.port.write(build_frame(T_STATUS, 0))
                last_status = now
            if progress:
                progress(acked_bytes, self.last, now - t0)
        return acked_bytes

    def end(self):
        """mark the end of the stream (TLAST on the last byte)"""
        seq = self.seq
        r = self._command(T_END, seq, lambda r: r.rtype == R_ACK and r.seq == seq)
        self.seq = (seq + 1) & 0xFF
        return r

    def wait_done(self, timeout=10.0, progress=None):
        """poll STATUS until the decoder has delivered all frames"""
        end = time.monotonic() + timeout
        r = self.last
        while time.monotonic() < end:
            r = self.status()
            if progress:
                progress(r)
            if r.done or r.error:
                return r
            time.sleep(0.05)
        return r


class UdpPort:
    """pyserial-like byte channel over UDP (write() = one datagram)"""

    def __init__(self, host, port=UDP_PORT, timeout=0.01):
        self.addr = (socket.gethostbyname(host), port)
        self.timeout = timeout
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
        self.sock.connect(self.addr)
        self.sock.setblocking(False)
        self.buf = bytearray()

    def _fill(self, wait):
        try:
            r, _, _ = select.select([self.sock], [], [], wait)
        except ValueError:
            return
        while r:
            try:
                self.buf += self.sock.recv(65536)
            except OSError:                    # nothing more, or an ICMP error (unreachable)
                break

    def write(self, data):
        try:
            self.sock.send(data)
        except OSError:
            pass                               # lost; the link protocol resends
        return len(data)

    @property
    def in_waiting(self):
        if not self.buf:
            self._fill(0)
        return len(self.buf)

    def read(self, n=1):
        if not self.buf:
            self._fill(self.timeout)
        d = bytes(self.buf[:n])
        del self.buf[:n]
        return d

    def reset_input_buffer(self):
        self._fill(0)
        self.buf.clear()

    def close(self):
        self.sock.close()


class KitConsole:
    """console text and status of the kit firmware over UDP (kit/src/service.c)"""

    def __init__(self, host, port=UDP_PORT + 1):
        self.addr = (socket.gethostbyname(host), port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.connect(self.addr)
        self.offset = 0               # next console byte to fetch
        self.uptime = None            # kit uptime at the last answer (ms)

    def _ask(self, req, want, timeout, tries):
        for _ in range(tries):
            try:
                self.sock.send(req)
            except OSError:
                pass
            end = time.monotonic() + timeout
            while True:
                left = end - time.monotonic()
                if left <= 0:
                    break
                self.sock.settimeout(left)
                try:
                    d = self.sock.recv(2048)
                except socket.timeout:
                    break
                except OSError:               # ICMP error (unreachable): wait out the try
                    time.sleep(min(left, 0.05))
                    continue
                if d[:5] == b"H264" + want:
                    return d[5:]
        return None

    def info(self, timeout=0.3, tries=3):
        """dict of the kit's "key=value" status lines, None without an answer"""
        d = self._ask(b"H264I", b"i", timeout, tries)
        if d is None:
            return None
        out = {}
        for line in d.decode("utf-8", "replace").splitlines():
            k, _, v = line.partition("=")
            if k:
                out[k] = v
        return out

    def read(self, timeout=0.3, tries=1):
        """new console text since the last call as (text, note), None without an answer;
        note: None, "lost" (older text overwritten in the kit) or "restart" (kit rebooted)"""
        chunks, note = [], None
        while True:
            d = self._ask(b"H264L" + struct.pack("<I", self.offset), b"l", timeout, tries)
            if d is None:
                return ("".join(chunks), note) if chunks else None
            first, total, uptime = struct.unpack("<III", d[:12])
            if self.uptime is not None and (uptime < self.uptime or first < self.offset):
                note, chunks = "restart", []
            elif first > self.offset:
                note = note or "lost"
            self.uptime = uptime
            self.offset = first + len(d) - 12
            chunks.append(d[12:].decode("utf-8", "replace").replace("\r\n", "\n"))
            if self.offset >= total:
                return "".join(chunks), note

    def close(self):
        self.sock.close()


def local_address(host, port=UDP_PORT):
    """local IPv4 address the PC uses to reach host (nothing is sent), None without a route"""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect((host, port))
        return s.getsockname()[0]
    except OSError:
        return None
    finally:
        s.close()


def system_ping(host, timeout=1.0):
    """one ICMP echo with the system ping command: True / False, None if there is none"""
    exe = shutil.which("ping")
    if not exe:
        return None
    if sys.platform.startswith("win"):
        cmd = [exe, "-n", "1", "-w", str(int(timeout * 1000)), host]
    elif sys.platform == "darwin":
        cmd = [exe, "-c", "1", "-W", str(int(timeout * 1000)), host]
    else:
        cmd = [exe, "-c", "1", "-W", str(max(1, round(timeout))), host]
    try:
        r = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=timeout + 5,
                           creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
    except (OSError, subprocess.TimeoutExpired):
        return False
    # Windows ping exits with 0 on "destination host unreachable" too; only real replies carry TTL
    return r.returncode == 0 and b"ttl=" in r.stdout.lower()


def open_port(name, baud=2000000):
    """serial port name or udp://IP[:PORT]; returns (port, baud used for timing)"""
    if name.lower().startswith("udp://"):
        host, _, p = name[6:].strip("/").partition(":")
        return UdpPort(host, int(p) if p else UDP_PORT), UDP_BAUD
    import serial
    return serial.Serial(name, baud, timeout=0.01, write_timeout=5), baud


def read_chunks(path, size=64 * 1024):
    """file contents in chunks; "-" reads stdin (e.g. a live ffmpeg pipe)"""
    if path == "-":
        f = sys.stdin.buffer
        while True:
            b = f.read1(size) if hasattr(f, "read1") else f.read(size)
            if not b:
                return
            yield b
    with open(path, "rb") as f:
        while True:
            b = f.read(size)
            if not b:
                return
            yield b


def sps_profile(path):
    """profile_idc / constraint flags of the first SPS of an Annex-B file
    (None if not found)"""
    with open(path, "rb") as f:
        data = f.read(1 << 20)
    i = 0
    while True:
        i = data.find(b"\x00\x00\x01", i)
        if i < 0 or i + 6 > len(data):
            return None
        if data[i + 3] & 0x1F == 7:
            return data[i + 4], data[i + 5]
        i += 3


def ffmpeg_cmd(src, size=None, bitrate=None, fps=None, out="-"):
    """ffmpeg command that turns any video into a Constrained Baseline
    Annex-B stream (pipe to stdout with out='-')"""
    exe = shutil.which("ffmpeg") or "ffmpeg"
    cmd = [exe, "-hide_banner", "-loglevel", "error", "-nostdin", "-i", src, "-an"]
    vf = []
    if size:
        w, h = size
        vf.append("scale=%d:%d" % (w, h))
    vf.append("format=yuv420p")
    cmd += ["-vf", ",".join(vf)]
    if fps:
        cmd += ["-r", str(fps)]
    cmd += ["-c:v", "libx264", "-profile:v", "baseline", "-level", "4.0", "-preset", "medium",
            "-x264-params", "bframes=0:cabac=0:8x8dct=0:weightp=0"]
    if bitrate:
        cmd += ["-b:v", str(bitrate), "-maxrate", str(bitrate), "-bufsize", str(2 * int(bitrate))]
    else:
        cmd += ["-crf", "23"]
    cmd += ["-f", "h264", out]
    return cmd


def ffmpeg_chunks(src, size=None, bitrate=None, fps=None, chunk=16 * 1024):
    """converted stream straight from ffmpeg's stdout"""
    proc = subprocess.Popen(ffmpeg_cmd(src, size, bitrate, fps), stdout=subprocess.PIPE)
    try:
        while True:
            b = proc.stdout.read(chunk)
            if not b:
                break
            yield b
    finally:
        proc.stdout.close()
        rc = proc.wait()
        if rc != 0:
            raise LinkError("ffmpeg failed (exit code %d)" % rc)


def parse_bitrate(s):
    if s is None:
        return None
    s = str(s).strip().lower()
    mul = 1
    if s.endswith("k"):
        mul, s = 1000, s[:-1]
    elif s.endswith("m"):
        mul, s = 1000000, s[:-1]
    return int(float(s) * mul)


def link_bitrate(baud):
    """usable payload rate of the link in bit/s (8N1, ~1 % framing)"""
    return int(baud * 0.8 * 0.99)


def auto_bitrate(baud):
    """default bit rate when converting: 3/4 of the link, at most AUTO_BITRATE_MAX"""
    return min(link_bitrate(baud) * 3 // 4, AUTO_BITRATE_MAX)
