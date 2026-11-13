    """
    Visualisasi realtime IMU (Roll & Pitch) dari MAVLink lewat USB CDC.

    Pesan MAVLink yang dibaca:
    HEARTBEAT            -> hanya untuk memastikan link hidup
    NAMED_VALUE_FLOAT    -> "comp_roll", "comp_pitch"  (derajat, hasil complementary filter)
    ATTITUDE             -> roll & pitch hasil Kalman (radian, otomatis diubah ke derajat)

    Setiap pesan ATTITUDE yang datang = satu titik baru di grafik. Firmware harus
    mengirim NAMED_VALUE_FLOAT SEBELUM ATTITUDE pada tiap siklus (sudah begitu di mav_telemetry.h).

    Install:
    pip install pymavlink pyserial matplotlib numpy

    Jalankan:
    python imu_mavlink.py                      (default COM5, 115200)
    python imu_mavlink.py --port COM7
    python imu_mavlink.py --port udpin:0.0.0.0:14550   (untuk tes via UDP)

    PENTING: tutup PuTTY / Serial Monitor / Mission Planner dulu,
    karena satu port COM hanya bisa dibuka oleh satu program.
    """

    import argparse
    import math
    import os
    import threading
    import time
    from collections import deque

    os.environ.setdefault("MAVLINK20", "1")  # pakai MAVLink 2 (harus sebelum import mavutil)

    import matplotlib.pyplot as plt
    import numpy as np
    from matplotlib.animation import FuncAnimation
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    from pymavlink import mavutil

    # ====================== KONFIGURASI ======================
    PORT = "COM5"
    BAUD = 115200            # untuk CDC nilainya tidak berpengaruh, tapi tetap diisi
    WINDOW = 400             # jumlah sampel yang tampil di grafik (sumbu X: 0..400)
    Y_LIM = (-100, 100)      # rentang sumbu Y grafik (derajat)
    INTERVAL_MS = 40         # periode refresh tampilan (ms)
    BOX_SIZE = (0.8, 1.6, 0.2)   # panjang X, lebar Y, tebal Z balok
    ROLL_SIGN = 1            # ganti ke -1 kalau arah miring roll terbalik
    PITCH_SIGN = 1           # ganti ke -1 kalau arah miring pitch terbalik
    LINK_TIMEOUT = 1.5       # detik tanpa ATTITUDE -> dianggap data berhenti
    # =========================================================


    class IMUData:
        """Penyimpan data bersama antara thread MAVLink dan thread GUI."""

        def __init__(self, n):
            self.lock = threading.Lock()
            self.kal_roll = deque(maxlen=n)
            self.kal_pitch = deque(maxlen=n)
            self.comp_roll = deque(maxlen=n)
            self.comp_pitch = deque(maxlen=n)
            self.comp_latest = {"comp_roll": 0.0, "comp_pitch": 0.0}
            self.connected = False
            self.last_rx = 0.0
            self.rate = 0.0

        def set_named(self, name, value):
            with self.lock:
                if name in self.comp_latest:
                    self.comp_latest[name] = value

        def push_attitude(self, roll_deg, pitch_deg):
            with self.lock:
                self.kal_roll.append(roll_deg)
                self.kal_pitch.append(pitch_deg)
                self.comp_roll.append(self.comp_latest["comp_roll"])
                self.comp_pitch.append(self.comp_latest["comp_pitch"])
                self.last_rx = time.time()

        def snapshot(self):
            with self.lock:
                if not self.kal_roll:
                    return None
                return (list(self.kal_roll), list(self.kal_pitch),
                        list(self.comp_roll), list(self.comp_pitch),
                        self.rate, self.last_rx)


    def mavlink_worker(conn, baud, data, stop):
        """Thread pembaca MAVLink. Otomatis menyambung ulang kalau putus."""
        master = None
        warned = False
        got_heartbeat = False
        count, t_rate = 0, time.time()

        while not stop.is_set():
            if master is None:
                try:
                    master = mavutil.mavlink_connection(conn, baud=baud, dialect="common")
                    data.connected = True
                    warned = False
                    got_heartbeat = False
                    print(f"[OK] {conn} terbuka, menunggu pesan MAVLink...")
                except Exception as e:
                    data.connected = False
                    if not warned:
                        print(f"[!] Gagal membuka {conn}: {e}")
                        try:
                            import serial.tools.list_ports
                            ports = ", ".join(p.device for p in serial.tools.list_ports.comports()) or "-"
                            print(f"    Port tersedia: {ports}")
                        except Exception:
                            pass
                        print("    Pastikan PuTTY / Mission Planner / program lain sudah ditutup.")
                        warned = True
                    time.sleep(1.0)
                    continue

            try:
                msg = master.recv_match(
                    type=["HEARTBEAT", "ATTITUDE", "NAMED_VALUE_FLOAT"],
                    blocking=True, timeout=1)
            except Exception as e:
                data.connected = False
                print(f"[!] Koneksi terputus ({e}), mencoba lagi...")
                try:
                    master.close()
                except Exception:
                    pass
                master = None
                continue

            now = time.time()
            if now - t_rate >= 1.0:
                data.rate = count / (now - t_rate)
                count, t_rate = 0, now

            if msg is None:
                continue

            kind = msg.get_type()
            if kind == "NAMED_VALUE_FLOAT":
                name = msg.name
                if isinstance(name, bytes):
                    name = name.decode("utf-8", errors="ignore")
                data.set_named(name.strip("\x00 "), float(msg.value))
            elif kind == "ATTITUDE":
                data.push_attitude(math.degrees(msg.roll), math.degrees(msg.pitch))
                count += 1
            elif kind == "HEARTBEAT" and not got_heartbeat:
                got_heartbeat = True
                print(f"[OK] Heartbeat diterima dari sysid={msg.get_srcSystem()} "
                    f"compid={msg.get_srcComponent()}")

        if master is not None:
            try:
                master.close()
            except Exception:
                pass


    def rotation_matrix(roll_deg, pitch_deg):
        """Roll = putar sumbu X, Pitch = putar sumbu Y. R = Ry(pitch) @ Rx(roll)."""
        r, p = np.radians([roll_deg, pitch_deg])
        rx = np.array([[1, 0, 0],
                    [0, np.cos(r), -np.sin(r)],
                    [0, np.sin(r), np.cos(r)]])
        ry = np.array([[np.cos(p), 0, np.sin(p)],
                    [0, 1, 0],
                    [-np.sin(p), 0, np.cos(p)]])
        return ry @ rx


    def make_box(size):
        lx, ly, lz = (s / 2 for s in size)
        verts = np.array([
            [-lx, -ly, -lz], [lx, -ly, -lz], [lx, ly, -lz], [-lx, ly, -lz],
            [-lx, -ly, lz], [lx, -ly, lz], [lx, ly, lz], [-lx, ly, lz],
        ])
        faces = [[0, 1, 2, 3], [4, 5, 6, 7], [0, 1, 5, 4],
                [2, 3, 7, 6], [1, 2, 6, 5], [0, 3, 7, 4]]
        return verts, faces


    def main():
        ap = argparse.ArgumentParser(description="Visualisasi realtime Roll/Pitch dari MAVLink")
        ap.add_argument("--port", default=PORT, help=f"port serial atau connection string (default {PORT})")
        ap.add_argument("--baud", type=int, default=BAUD, help=f"baudrate (default {BAUD})")
        args = ap.parse_args()

        data = IMUData(WINDOW)
        stop = threading.Event()
        threading.Thread(target=mavlink_worker, args=(args.port, args.baud, data, stop),
                        daemon=True).start()

        # ---------- Figure (latar putih) ----------
        plt.style.use("default")
        fig = plt.figure(figsize=(13, 6), facecolor="white")
        try:
            fig.canvas.manager.set_window_title("IMU Realtime (MAVLink) - Roll & Pitch")
        except Exception:
            pass
        title = fig.suptitle("Menunggu data...", fontsize=13)

        gs = fig.add_gridspec(1, 2, width_ratios=[1, 1.25])
        ax3d = fig.add_subplot(gs[0], projection="3d")
        ax = fig.add_subplot(gs[1])
        fig.subplots_adjust(left=0.03, right=0.97, bottom=0.08, top=0.9, wspace=0.12)

        # ---------- Balok 3D ----------
        ax3d.set_xlim(-1.5, 1.5)
        ax3d.set_ylim(-1.5, 1.5)
        ax3d.set_zlim(-1.5, 1.5)
        ax3d.set_box_aspect((1, 1, 1))
        ax3d.set_xlabel("X")
        ax3d.set_ylabel("Y")
        ax3d.set_zlabel("Z")

        base_verts, faces = make_box(BOX_SIZE)
        box = Poly3DCollection([base_verts[f] for f in faces],
                            facecolors="#1f77d4", edgecolors="black",
                            linewidths=1, alpha=0.95)
        ax3d.add_collection3d(box)

        # ---------- Grafik garis ----------
        green, gold = "#1b9e77", "#c9a227"
        (l_kx,) = ax.plot([], [], color=green, lw=2, label="Kalman X (roll)")
        (l_ky,) = ax.plot([], [], color=gold, lw=2, label="Kalman Y (pitch)")
        (l_cx,) = ax.plot([], [], color=green, lw=1, ls="--", alpha=0.8, label="Comp X")
        (l_cy,) = ax.plot([], [], color=gold, lw=1, ls="--", alpha=0.8, label="Comp Y")
        ax.set_xlim(0, WINDOW)
        ax.set_ylim(*Y_LIM)
        ax.set_xlabel("sampel")
        ax.set_ylabel("derajat")
        ax.grid(True, alpha=0.4)
        ax.legend(loc="upper left")

        def update(_frame):
            snap = data.snapshot()
            if snap is None:
                title.set_text(
                    f"Port {args.port} terbuka, menunggu pesan ATTITUDE..." if data.connected
                    else f"Menyambung ke {args.port}..."
                )
                return

            kr, kp, cr, cp, rate, last_rx = snap
            x = np.arange(len(kr))
            l_kx.set_data(x, kr)
            l_ky.set_data(x, kp)
            l_cx.set_data(x, cr)
            l_cy.set_data(x, cp)

            roll, pitch = kr[-1], kp[-1]
            rot = rotation_matrix(ROLL_SIGN * roll, PITCH_SIGN * pitch)
            v = base_verts @ rot.T
            box.set_verts([v[f] for f in faces])

            if time.time() - last_rx > LINK_TIMEOUT:
                title.set_text("Data MAVLink berhenti / terputus")
            else:
                title.set_text(f"Roll: {roll:.1f}°   Pitch: {pitch:.1f}°   ({rate:.0f} Hz)")

        anim = FuncAnimation(fig, update, interval=INTERVAL_MS,
                            blit=False, cache_frame_data=False)
        fig._anim = anim  # jaga referensi agar tidak di-garbage-collect

        try:
            plt.show()
        finally:
            stop.set()


    if __name__ == "__main__":
        main()
