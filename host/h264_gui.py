#!/usr/bin/env python3
"""Graphical sender for the PolarFire H.264 decoder over Ethernet (Tkinter).

    python h264_gui.py

Enter the kit's IP address (firmware in kit/, default 192.168.10.2), press
"Bağlantıyı test et" to check the path PC -> MSS -> fabric, pick the video
and press "Gönder".  The "Kit konsolu" tab shows the kit firmware's console,
read over Ethernet (UDP port + 1), so no UART is needed.  Files that are not a Constrained Baseline Annex-B
stream are converted with ffmpeg on the fly (size / bit rate from the
fields).  The window shows the progress and the decoder status (pictures,
resolution, errors) reported by the FPGA.
"""
import os
import queue
import socket
import sys
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import h264link as hl

KIT_IP = os.environ.get("H264_KIT_IP", "192.168.10.2")   # same default as h264_send.py
SIZES = ["orijinal", "1920x1080", "1280x720", "854x480", "640x480", "352x288"]
# connection test: line prefix and colour per step state
STEP_STYLE = {
    "ok": ("✔ ", "#1a7f37"), "fail": ("✘ ", "#cf222e"), "warn": ("! ", "#bc4c00"),
    "skip": ("– ", "#6e7781"), "info": ("  ", None),
    "pass": ("SONUÇ: ", "#1a7f37"), "result-fail": ("SONUÇ: ", "#cf222e"),
}


class App:
    def __init__(self, root):
        self.root = root
        root.title("H.264 → PolarFire çözücü (Ethernet)")
        style = ttk.Style(root)
        if "clam" in style.theme_names():
            style.theme_use("clam")
        self.q = queue.Queue()
        self.worker = None
        self.sender = None

        f = ttk.Frame(root, padding=10)
        f.grid(sticky="nsew")
        root.columnconfigure(0, weight=1)
        root.rowconfigure(0, weight=1)
        f.columnconfigure(1, weight=1)

        net = ttk.Frame(f)
        net.grid(row=0, column=0, columnspan=3, sticky="ew")
        ttk.Label(net, text="Kit IP").pack(side="left")
        self.ip = ttk.Entry(net, width=16)
        self.ip.insert(0, KIT_IP)
        self.ip.pack(side="left", padx=(4, 12))
        ttk.Label(net, text="UDP port").pack(side="left")
        self.udp_port = ttk.Entry(net, width=6)
        self.udp_port.insert(0, str(hl.UDP_PORT))
        self.udp_port.pack(side="left", padx=(4, 12))
        self.btn_test = ttk.Button(net, text="Bağlantıyı test et", command=self.test_link)
        self.btn_test.pack(side="left")

        ttk.Label(f, text="Video").grid(row=1, column=0, sticky="w", pady=(6, 0))
        self.path = tk.StringVar()
        ttk.Entry(f, textvariable=self.path).grid(row=1, column=1, sticky="ew", padx=4, pady=(6, 0))
        ttk.Button(f, text="Seç…", command=self.pick).grid(row=1, column=2, sticky="ew", pady=(6, 0))

        opt = ttk.LabelFrame(f, text="Dönüştürme (ffmpeg, Baseline olmayan dosyalar için)", padding=6)
        opt.grid(row=2, column=0, columnspan=3, sticky="ew", pady=6)
        self.force = tk.BooleanVar(value=False)
        ttk.Checkbutton(opt, text="her zaman dönüştür", variable=self.force).grid(row=0, column=0, sticky="w")
        ttk.Label(opt, text="Boyut").grid(row=0, column=1, padx=(12, 2))
        self.size = ttk.Combobox(opt, width=10, values=SIZES)
        self.size.set("1280x720")
        self.size.grid(row=0, column=2)
        ttk.Label(opt, text="Bit hızı").grid(row=0, column=3, padx=(12, 2))
        self.bitrate = ttk.Entry(opt, width=8)
        self.bitrate.insert(0, "otomatik")
        self.bitrate.grid(row=0, column=4)

        disp = ttk.LabelFrame(f, text="Ekran (HDMI)", padding=6)
        disp.grid(row=3, column=0, columnspan=3, sticky="ew", pady=(0, 6))
        ttk.Label(disp, text="Kare hızı (fps)").grid(row=0, column=0)
        self.fps = ttk.Entry(disp, width=9)
        self.fps.insert(0, "otomatik")
        self.fps.grid(row=0, column=1, padx=(4, 12))
        self.bars = tk.BooleanVar(value=False)
        ttk.Checkbutton(disp, text="renk çubukları (HDMI testi)", variable=self.bars,
                        command=self.apply_display).grid(row=0, column=2, sticky="w")
        self.bitrev = tk.BooleanVar(value=False)
        ttk.Checkbutton(disp, text="TMDS bit sırası ters", variable=self.bitrev,
                        command=self.apply_display).grid(row=0, column=3, sticky="w", padx=(12, 0))
        ttk.Button(disp, text="Uygula", command=self.apply_display).grid(row=0, column=4, padx=(12, 0))

        b = ttk.Frame(f)
        b.grid(row=4, column=0, columnspan=3, sticky="ew")
        self.btn_send = ttk.Button(b, text="Gönder", command=self.send)
        self.btn_send.pack(side="left")
        self.btn_stop = ttk.Button(b, text="Durdur", command=self.stop, state="disabled")
        self.btn_stop.pack(side="left", padx=4)
        ttk.Button(b, text="Durum", command=self.status).pack(side="left")

        self.prog = ttk.Progressbar(f, mode="determinate", maximum=1000)
        self.prog.grid(row=5, column=0, columnspan=3, sticky="ew", pady=6)
        self.info = tk.StringVar(value="hazır")
        ttk.Label(f, textvariable=self.info).grid(row=6, column=0, columnspan=3, sticky="w")
        self.dec = tk.StringVar(value="çözücü: -")
        ttk.Label(f, textvariable=self.dec).grid(row=7, column=0, columnspan=3, sticky="w")

        nb = ttk.Notebook(f)
        nb.grid(row=8, column=0, columnspan=3, sticky="nsew", pady=(6, 0))
        f.rowconfigure(8, weight=1)
        tab_log = ttk.Frame(nb)
        tab_con = ttk.Frame(nb)
        nb.add(tab_log, text="Günlük")
        nb.add(tab_con, text="Kit konsolu")
        self.log = self.text_box(tab_log)
        for state, (_, colour) in STEP_STYLE.items():
            if colour:
                self.log.tag_configure(state, foreground=colour)
        top = ttk.Frame(tab_con)
        top.pack(fill="x", pady=(2, 2))
        self.watch = tk.BooleanVar(value=True)
        ttk.Checkbutton(top, text="kit konsolunu izle", variable=self.watch).pack(side="left")
        self.con_state = tk.StringVar(value="kit konsolu: -")
        ttk.Label(top, textvariable=self.con_state).pack(side="left", padx=12)
        ttk.Button(top, text="Temizle", command=lambda: self.clear(self.con)).pack(side="right")
        self.con = self.text_box(tab_con)
        self.con_job = None
        self.kc = None                # KitConsole, used by one console job at a time
        self.kc_key = None

        root.after(100, self.pump)
        root.after(500, self.console_tick)

    # -- helpers -----------------------------------------------------------
    @staticmethod
    def text_box(parent):
        box = ttk.Frame(parent)
        box.pack(fill="both", expand=True)
        t = tk.Text(box, height=12, width=80, state="disabled", wrap="word")
        sb = ttk.Scrollbar(box, orient="vertical", command=t.yview)
        t.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y")
        t.pack(side="left", fill="both", expand=True)
        return t

    @staticmethod
    def append(t, text, tags=()):
        at_end = t.yview()[1] >= 0.999
        t["state"] = "normal"
        t.insert("end", text, tags)
        t["state"] = "disabled"
        if at_end:
            t.see("end")

    @staticmethod
    def clear(t):
        t["state"] = "normal"
        t.delete("1.0", "end")
        t["state"] = "disabled"

    def say(self, msg, tag=None):
        self.q.put(("log", (msg, tag)))

    def step(self, state, msg):
        """one line of the connection test"""
        self.say(STEP_STYLE[state][0] + msg, state)

    def pick(self):
        p = filedialog.askopenfilename(title="Video seç", filetypes=[
            ("H.264 / video", "*.264 *.h264 *.mp4 *.mkv *.mov *.avi *.ts"), ("Hepsi", "*.*")])
        if p:
            self.path.set(p)

    def describe(self, r):
        st = []
        if r.error:
            st.append("HATA (desteklenmeyen / bozuk akış)")
        if r.done:
            st.append("akış bitti")
        return "çözücü: %d resim, %dx%d, FIFO boş alan %d bayt %s" % (
            r.pictures, r.width, r.height, r.free, "- " + ", ".join(st) if st else "")

    def settings(self):
        """widget values, read in the GUI thread (Tk is not thread safe); ValueError if invalid"""
        ip = self.ip.get().strip()
        port = int(self.udp_port.get())
        if not ip or not 0 < port < 65536:
            raise ValueError
        fps = self.fps.get().strip().replace(",", ".")
        if fps not in ("", "otomatik"):
            fps = float(fps)
            if not 1 <= fps <= 60:
                raise ValueError
        else:
            fps = None
        return dict(ip=ip, port=port, force=self.force.get(), size=self.size.get(),
                    bitrate=self.bitrate.get().strip(), fps=fps, bars=self.bars.get(),
                    bitrev=self.bitrev.get())

    def checked_settings(self):
        try:
            return self.settings()
        except ValueError:
            messagebox.showerror("Hata", "Kit IP adresini, UDP portunu (1-65535) ve kare hızını "
                                 "(1-60 ya da otomatik) kontrol edin")
            return None

    def open_sender(self, cfg):
        return hl.Sender(hl.UdpPort(cfg["ip"], cfg["port"]), hl.UDP_BAUD, log=self.say)

    def busy(self):
        return self.worker is not None and self.worker.is_alive()

    def start_worker(self, target, *args):
        self.btn_send["state"] = "disabled"
        self.btn_test["state"] = "disabled"
        self.worker = threading.Thread(target=target, args=args, daemon=True)
        self.worker.start()

    @staticmethod
    def link_error(e):
        if isinstance(e, hl.LinkError) and "no answer" in str(e):
            return "Kit yanıt vermiyor. \"Bağlantıyı test et\" ile nedenini görebilirsiniz."
        if isinstance(e, OSError):
            return "Ağa ulaşılamıyor (%s). PC'nin Ethernet bağlantısını ve adresini kontrol edin." % e
        return str(e)

    # -- HDMI display --------------------------------------------------------
    def set_display(self, cfg, fps):
        """send frame rate / colour bars to the kit (worker thread); True on success"""
        try:
            c = hl.KitConsole(cfg["ip"], cfg["port"] + 1)
            try:
                r = c.set_display(fps, (2 if cfg["bars"] else 0) | (4 if cfg["bitrev"] else 0))
            finally:
                c.close()
        except OSError:
            r = None
        if r is None or r == 0xFFFFFFFF:
            self.say("ekran ayarı gönderilemedi (kit yazılımı ya da fabric yanıt vermedi)", "warn")
            return False
        self.say("ekran: %.2f fps%s" % ((r >> 16) / 100.0, ", renk çubukları" if r & 2 else ""))
        return True

    def apply_display(self):
        if self.busy():
            return
        cfg = self.checked_settings()
        if not cfg:
            return
        fps = cfg["fps"] or 30.0
        self.worker = threading.Thread(target=self.set_display, args=(cfg, fps), daemon=True)
        self.worker.start()

    # -- kit console (over Ethernet) -----------------------------------------
    def console_tick(self):
        """every second: fetch new console text in the background (GUI thread)"""
        if self.watch.get() and not (self.con_job and self.con_job.is_alive()):
            try:
                ip, port = self.ip.get().strip(), int(self.udp_port.get()) + 1
            except ValueError:
                ip = None
            if ip:
                self.con_job = threading.Thread(target=self.console_job, args=(ip, port), daemon=True)
                self.con_job.start()
        self.root.after(1000, self.console_tick)

    def console_job(self, ip, port):
        try:
            if self.kc_key != (ip, port):
                if self.kc:
                    self.kc.close()
                    self.q.put(("con", "── %s:%d ──\n" % (ip, port)))
                self.kc, self.kc_key = hl.KitConsole(ip, port), (ip, port)
            res = self.kc.read(timeout=0.3)
        except OSError:
            res = None
        if res is None:
            self.q.put(("con-state", "kit konsolu: yanıt yok (%s, UDP %d)" % (ip, port)))
            return
        text, note = res
        self.q.put(("con-state", "kit konsolu: bağlı, kit %d sn'dir çalışıyor" % (self.kc.uptime // 1000)))
        if note == "restart":
            text = "── kit yeniden başladı ──\n" + text
        elif note == "lost":
            text = "…\n" + text
        if text:
            self.q.put(("con", text))

    # -- connection test -----------------------------------------------------
    def test_link(self):
        if self.busy():
            return
        cfg = self.checked_settings()
        if cfg:
            self.info.set("bağlantı test ediliyor…")
            self.say("── Bağlantı testi: %s, UDP %d ──" % (cfg["ip"], cfg["port"]))
            self.start_worker(self.test_job, cfg)

    def test_job(self, cfg):
        try:
            ok, summary = self.run_test(cfg)
        except Exception as e:      # unexpected: show it rather than a silent thread death
            ok, summary = False, "beklenmeyen hata: %s" % e
        self.step("pass" if ok else "result-fail", summary)
        self.q.put(("info", summary))
        self.q.put(("idle", None))

    def run_test(self, cfg):
        """PC address -> ping (MSS) -> firmware status (MSS) -> link STATUS (fabric);
        returns (ok, summary)"""
        try:
            ip = socket.gethostbyname(cfg["ip"])
        except OSError:
            self.step("fail", "adres anlaşılamadı: %s" % cfg["ip"])
            return False, "Kit IP adresini kontrol edin (ör. 192.168.10.2)."

        # 1. the PC's own address towards the kit (/24 assumed, like the firmware default)
        local = hl.local_address(ip, cfg["port"])
        same = local is not None and local.rsplit(".", 1)[0] == ip.rsplit(".", 1)[0]
        net = ip.rsplit(".", 1)[0] + ".x"
        if local is None:
            self.step("fail", "PC'den %s adresine yol yok (Ethernet kartı bağlı ve adresli mi?)" % ip)
        elif same:
            self.step("ok", "PC bu adresle gönderecek: %s (kitle aynı alt ağ)" % local)
        else:
            self.step("warn", "PC %s adresiyle gönderecek; kit %s ağında. PC'nin Ethernet kartına "
                      "%s/24 sabit adres verin (ör. %s.1)" % (local, net, net, ip.rsplit(".", 1)[0]))

        # 2. ping: answered by the MSS firmware alone
        ping = hl.system_ping(ip)
        if ping is None:
            self.step("skip", "ping komutu bulunamadı, bu adım atlandı")
        elif ping:
            self.step("ok", "ping yanıt veriyor: kit ağda, MSS yazılımı çalışıyor")
        else:
            self.step("fail", "ping yanıt vermiyor")

        # 3. firmware status on UDP port + 1 (the first request also probes the fabric)
        kc = self.kc
        console_seen = (self.kc_key == (ip, cfg["port"] + 1) and kc is not None and kc.uptime is not None)
        kit = None
        try:
            c = hl.KitConsole(ip, cfg["port"] + 1)
            try:
                kit = c.info(timeout=0.4, tries=3)
            finally:
                c.close()
        except OSError as e:
            self.step("fail", "UDP soketi açılamadı: %s" % e)
        fabric = kit.get("fabric") if kit else None
        base = kit.get("fabric_base", "?") if kit else "?"
        if kit is None:
            self.step("fail", "kit yazılımı bilgi isteğine yanıt vermedi (UDP %d)" % (cfg["port"] + 1))
        else:
            up = int(kit.get("uptime_ms", "0") or 0) // 1000
            self.step("ok", "kit yazılımı yanıt veriyor: %s, %d sn'dir çalışıyor, Ethernet bağlantısı %s"
                      % (kit.get("fw", "?"), up, "var" if kit.get("link") == "1" else "yok"))
            if fabric == "ok":
                self.step("ok", "FPGA'daki köprü bulundu (h264_apb_bridge, %s)" % base)
            else:
                self.step("fail", "FPGA'daki köprü bulunamadı (%s adresinde ID okunamadı)" % base)

        # 4. link STATUS: answered by h264_link in the fabric, through the MSS
        if fabric == "missing":
            self.step("skip", "köprü bulunamadığı için fabric durum sorgusu atlandı")
            r = None
        else:
            r = self.probe_link(cfg)

        if r is not None:
            if r.error:
                return True, "Bağlantı tamam. Çözücü hata bildiriyor; START ile (Gönder) sıfırlanır."
            return True, "Bağlantı tamam, video gönderilebilir."
        if fabric == "missing":
            return False, ("Kit yazılımı çalışıyor ama FPGA'daki köprüyü bulamadı. FPGA'ya h264_dec_mss_top "
                           "içeren bitstream yüklü mü, FABRIC_BASE (%s) Libero bellek haritasındaki "
                           "adresle aynı mı, FIC3 etkin mi?" % base)
        if kit is not None:
            return False, ("Kit yazılımı ve köprü çalışıyor ama h264_link yanıt vermiyor: fabric saatini "
                           "ve reset'ini (h264_dec_mss_top) kontrol edin.")
        if ping and console_seen:
            return False, ("Kit konsolu az önce UDP'ye yanıt veriyordu, kit yazılımı şimdi vermiyor: FPGA'ya "
                           "(FIC3) erişirken takılmış. MSS Configurator'da FIC3'ün etkin ve saatli olduğunu, "
                           "FABRIC_BASE'in Libero bellek haritasındaki adresle aynı olduğunu kontrol edin; "
                           "sonra kiti sıfırlayın.")
        if ping:
            return False, ("Kit ping'e yanıt veriyor ama UDP isteklerine vermiyor. Ya PC'nin güvenlik duvarı "
                           "gelen UDP'yi engelliyor (Windows: Python'a izin verin) ya da kit yazılımı FPGA'ya "
                           "(FIC3) erişirken takıldı: FIC3 etkin ve saatli mi, FABRIC_BASE doğru mu? Kit "
                           "konsolu sekmesindeki son satıra bakın, kiti sıfırlayıp yeniden deneyin.")
        if local is None or not same:
            return False, "PC ile kit aynı ağda değil: önce PC'nin Ethernet adresini düzeltin."
        return False, ("Kite ulaşılamıyor: kabloyu ve bağlantı ışıklarını, kitin IP adresini "
                       "(kit/src/app_config.h, APP_IP) ve kitte yazılımın çalıştığını kontrol edin. Ağ "
                       "hiç çalışmıyorsa kiti yalnızca isteğe bağlı UART konsolu ya da hata ayıklayıcı "
                       "(SoftConsole) gösterebilir; kit az önce yanıt veriyorduysa sıfırlayın.")

    def probe_link(self, cfg):
        """STATUS requests to h264_link with round trip times; the response or None"""
        try:
            s = self.open_sender(cfg)
        except OSError as e:
            self.step("fail", "UDP soketi açılamadı: %s" % e)
            return None
        try:
            try:
                r = s.status(tries=3, timeout=0.3)
            except hl.LinkError:
                self.step("fail", "fabric'ten yanıt yok (UDP %d, durum sorgusu 3 kez)" % cfg["port"])
                return None
            rtt = []
            for _ in range(10):
                t0 = time.monotonic()
                try:
                    r = s.status(tries=1, timeout=0.3)
                    rtt.append(time.monotonic() - t0)
                except hl.LinkError:
                    pass
            if rtt:
                self.step("ok", "fabric yanıt veriyor (h264_link): gidiş-dönüş ort. %.2f ms, en çok "
                          "%.2f ms, %d/10 yanıt" % (1e3 * sum(rtt) / len(rtt), 1e3 * max(rtt), len(rtt)))
            else:
                self.step("warn", "fabric ilk sorguya yanıt verdi, sonraki 10 sorguya vermedi")
            self.step("info", self.describe(r))
            self.q.put(("dec", r))
            return r
        finally:
            s.port.close()

    # -- actions -----------------------------------------------------------
    def status(self):
        if self.busy():
            return
        cfg = self.checked_settings()
        if not cfg:
            return

        def job():
            try:
                s = self.open_sender(cfg)
                try:
                    r = s.status()
                    self.q.put(("dec", r))
                    self.say(self.describe(r))
                finally:
                    s.port.close()
            except Exception as e:
                self.q.put(("err", self.link_error(e)))
        self.worker = threading.Thread(target=job, daemon=True)
        self.worker.start()

    def stop(self):
        if self.sender:
            self.sender.stop()

    def send(self):
        if self.busy():
            return
        path = self.path.get()
        if not os.path.isfile(path):
            messagebox.showerror("Hata", "Video dosyası seçin")
            return
        cfg = self.checked_settings()
        if not cfg:
            return
        self.btn_stop["state"] = "normal"
        self.start_worker(self.job, path, cfg)

    def job(self, path, cfg):
        try:
            convert = cfg["force"]
            prof = hl.sps_profile(path) if os.path.splitext(path)[1].lower() in (".264", ".h264") else None
            if prof is None or prof[0] != 66:
                convert = True
            if convert and not hl.find_tool("ffmpeg"):
                self.q.put(("err", "Bu dosya dönüştürülmeli ama ffmpeg bulunamadı. libx264'lü ffmpeg.exe'yi "
                                   "bu programın yanına (ya da yanındaki ffmpeg\\bin klasörüne) koyun veya "
                                   "PATH'e ekleyin. Constrained Baseline .264 dosyaları ffmpeg olmadan gönderilir."))
                return
            if convert:
                size = None if cfg["size"] == "orijinal" else tuple(int(v) for v in cfg["size"].split("x"))
                br = cfg["bitrate"]
                br = hl.parse_bitrate(br) if br and br != "otomatik" else hl.auto_bitrate(hl.UDP_BAUD)
                self.say("ffmpeg ile dönüştürülüyor: Constrained Baseline, %s, %.2f Mbit/s"
                         % (cfg["size"], br / 1e6))
                chunks = hl.ffmpeg_chunks(path, size, br)
                total = None
            else:
                chunks = hl.read_chunks(path)
                total = os.path.getsize(path)
                self.say("dosya olduğu gibi gönderiliyor (%d bayt)" % total)
            fps = cfg["fps"] or hl.video_fps(path) or 30.0
            self.set_display(cfg, fps)
            self.sender = s = self.open_sender(cfg)
            try:
                r = s.start()
                self.say("bağlantı kuruldu, çözücü sıfırlandı (FIFO %d bayt)" % (r.free + 1))
                last_ui = [0.0]

                def progress(done, last, dt):
                    now = time.monotonic()
                    if now - last_ui[0] < 0.1:
                        return
                    last_ui[0] = now
                    self.q.put(("prog", (done, total, dt, last)))

                t0 = time.monotonic()
                n = s.send(chunks, progress)
                s.end()
                dt = time.monotonic() - t0
                self.say("%d bayt %.1f s'de gönderildi (%.2f Mbit/s), yeniden gönderilen çerçeve %d"
                         % (n, dt, n * 8 / dt / 1e6 if dt else 0, s.stats["retx"]))
                self.q.put(("prog", (n, n, dt, s.last)))
                r = s.wait_done(timeout=30, progress=lambda r: self.q.put(("dec", r)))
                self.q.put(("dec", r))
                self.say("bitti: " + self.describe(r)[len("çözücü: "):])
            finally:
                s.port.close()
        except Exception as e:
            self.q.put(("err", self.link_error(e)))
        finally:
            self.q.put(("idle", None))

    def pump(self):
        try:
            while True:
                kind, v = self.q.get_nowait()
                if kind == "log":
                    msg, tag = v
                    self.append(self.log, msg + "\n", (tag,) if tag else ())
                elif kind == "con":
                    self.append(self.con, v)
                elif kind == "con-state":
                    self.con_state.set(v)
                elif kind == "prog":
                    done, total, dt, last = v
                    if total:
                        self.prog["value"] = 1000 * done / max(total, 1)
                    else:
                        self.prog["mode"] = "indeterminate"
                        self.prog.step(5)
                    self.info.set("%.1f KB gönderildi, %.2f Mbit/s" % (done / 1024, done * 8 / dt / 1e6 if dt else 0))
                    if last:
                        self.dec.set(self.describe(last))
                elif kind == "dec":
                    self.dec.set(self.describe(v))
                elif kind == "info":
                    self.info.set(v)
                elif kind == "err":
                    self.say("HATA: " + v, "fail")
                    messagebox.showerror("Hata", v)
                elif kind == "idle":
                    self.btn_send["state"] = "normal"
                    self.btn_test["state"] = "normal"
                    self.btn_stop["state"] = "disabled"
                    self.prog["mode"] = "determinate"
                    self.sender = None
        except queue.Empty:
            pass
        self.root.after(100, self.pump)


def main():
    if sys.platform.startswith("win"):
        try:                    # sharp text on scaled (high DPI) Windows displays
            import ctypes
            ctypes.windll.shcore.SetProcessDpiAwareness(1)
        except (AttributeError, OSError):
            pass
    root = tk.Tk()
    App(root)
    root.mainloop()


if __name__ == "__main__":
    main()
