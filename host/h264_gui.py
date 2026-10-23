#!/usr/bin/env python3
"""Graphical sender for the PolarFire H.264 decoder (Tkinter + pyserial).

    python h264_gui.py

Pick the serial port and the video, press "Gönder".  Files that are not a
Constrained Baseline Annex-B stream are converted with ffmpeg on the fly
(size / bit rate from the fields).  The window shows the progress and the
decoder status (pictures, resolution, errors) reported by the FPGA.
"""
import os
import queue
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import h264link as hl

SIZES = ["orijinal", "1920x1080", "1280x720", "854x480", "640x480", "352x288"]


class App:
    def __init__(self, root):
        self.root = root
        root.title("H.264 → PolarFire çözücü")
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

        ttk.Label(f, text="Seri port").grid(row=0, column=0, sticky="w")
        self.port = ttk.Combobox(f, width=30)
        self.port.grid(row=0, column=1, sticky="ew", padx=4)
        ttk.Button(f, text="Yenile", command=self.refresh_ports).grid(row=0, column=2, sticky="ew")

        ttk.Label(f, text="Baud").grid(row=1, column=0, sticky="w")
        self.baud = ttk.Combobox(f, width=12, values=["2000000", "3000000", "1000000", "921600", "12000000"])
        self.baud.set("2000000")
        self.baud.grid(row=1, column=1, sticky="w", padx=4)

        ttk.Label(f, text="Video").grid(row=2, column=0, sticky="w")
        self.path = tk.StringVar()
        ttk.Entry(f, textvariable=self.path).grid(row=2, column=1, sticky="ew", padx=4)
        ttk.Button(f, text="Seç…", command=self.pick).grid(row=2, column=2, sticky="ew")

        opt = ttk.LabelFrame(f, text="Dönüştürme (ffmpeg, Baseline olmayan dosyalar için)", padding=6)
        opt.grid(row=3, column=0, columnspan=3, sticky="ew", pady=6)
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

        self.log = tk.Text(f, height=10, width=80, state="disabled")
        self.log.grid(row=8, column=0, columnspan=3, sticky="nsew", pady=(6, 0))
        f.rowconfigure(8, weight=1)

        self.refresh_ports()
        root.after(100, self.pump)

    # -- helpers -----------------------------------------------------------
    def say(self, msg):
        self.q.put(("log", msg))

    def refresh_ports(self):
        try:
            from serial.tools import list_ports
            ports = [p.device for p in list_ports.comports()]
        except Exception:
            ports = []
        self.port["values"] = ports
        if ports and not self.port.get():
            self.port.set(ports[0])

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
        """widget values, read in the GUI thread (Tk is not thread safe)"""
        return dict(port=self.port.get(), baud=int(self.baud.get()), force=self.force.get(),
                    size=self.size.get(), bitrate=self.bitrate.get().strip())

    def open_sender(self, cfg):
        import serial
        port = serial.Serial(cfg["port"], cfg["baud"], timeout=0.01, write_timeout=5)
        return hl.Sender(port, cfg["baud"], log=self.say)

    # -- actions -----------------------------------------------------------
    def status(self):
        if self.worker and self.worker.is_alive():
            return
        cfg = self.settings()

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
                self.q.put(("err", str(e)))
        self.worker = threading.Thread(target=job, daemon=True)
        self.worker.start()

    def stop(self):
        if self.sender:
            self.sender.stop()

    def send(self):
        path = self.path.get()
        if not os.path.isfile(path):
            messagebox.showerror("Hata", "Video dosyası seçin")
            return
        if not self.port.get():
            messagebox.showerror("Hata", "Seri port seçin")
            return
        try:
            cfg = self.settings()
        except ValueError:
            messagebox.showerror("Hata", "Baud değeri sayı olmalı")
            return
        self.btn_send["state"] = "disabled"
        self.btn_stop["state"] = "normal"
        self.worker = threading.Thread(target=self.job, args=(path, cfg), daemon=True)
        self.worker.start()

    def job(self, path, cfg):
        try:
            baud = cfg["baud"]
            convert = cfg["force"]
            prof = hl.sps_profile(path) if os.path.splitext(path)[1].lower() in (".264", ".h264") else None
            if prof is None or prof[0] != 66:
                convert = True
            if convert:
                size = None if cfg["size"] == "orijinal" else tuple(int(v) for v in cfg["size"].split("x"))
                br = cfg["bitrate"]
                br = hl.parse_bitrate(br) if br and br != "otomatik" else hl.link_bitrate(baud) * 3 // 4
                self.say("ffmpeg ile dönüştürülüyor: Constrained Baseline, %s, %.2f Mbit/s"
                         % (cfg["size"], br / 1e6))
                chunks = hl.ffmpeg_chunks(path, size, br)
                total = None
            else:
                chunks = hl.read_chunks(path)
                total = os.path.getsize(path)
                self.say("dosya olduğu gibi gönderiliyor (%d bayt)" % total)
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
            self.q.put(("err", str(e)))
        finally:
            self.q.put(("idle", None))

    def pump(self):
        try:
            while True:
                kind, v = self.q.get_nowait()
                if kind == "log":
                    self.log["state"] = "normal"
                    self.log.insert("end", v + "\n")
                    self.log.see("end")
                    self.log["state"] = "disabled"
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
                elif kind == "err":
                    self.say("HATA: " + v)
                    messagebox.showerror("Hata", v)
                elif kind == "idle":
                    self.btn_send["state"] = "normal"
                    self.btn_stop["state"] = "disabled"
                    self.prog["mode"] = "determinate"
                    self.sender = None
        except queue.Empty:
            pass
        self.root.after(100, self.pump)


def main():
    root = tk.Tk()
    App(root)
    root.mainloop()


if __name__ == "__main__":
    main()
