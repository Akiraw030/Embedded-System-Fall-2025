# server_debug.py
import socket
import threading
import json
import queue
import time
import traceback
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

HOST = '0.0.0.0'
PORT = 8002

data_q = queue.Queue()
event_q = queue.Queue()

def client_thread(conn, addr):
    print(f"[client_thread] started for {addr}")
    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    buff = b''
    try:
        while True:
            try:
                d = conn.recv(1024)
            except Exception as e:
                print(f"[{addr}] recv() raised: {e}")
                break

            if not d:
                print(f"[{addr}] connection closed by peer (recv returned 0 bytes)")
                break

            # show exactly what bytes arrived
            print(f"[{addr}] RAW RECV ({len(d)} bytes): {repr(d)}")

            buff += d
            # Accept both \n and \r\n line endings; split on \n then strip trailing \r
            while b'\n' in buff:
                line, buff = buff.split(b'\n', 1)
                # remove trailing CR if present
                if line.endswith(b'\r'):
                    line = line[:-1]
                if not line:
                    continue
                # for debugging, show the raw line before parsing
                print(f"[{addr}] RAW LINE: {repr(line)}")
                try:
                    s = line.decode('utf-8', errors='replace')
                    obj = json.loads(s)
                except Exception:
                    print(f"[{addr}] JSON parse error on line: {repr(line)}")
                    traceback.print_exc()
                    # print the raw string so you can inspect it
                    print(f"[{addr}] got (raw decoded): {s!r}")
                    continue

                # if it's an accelerometer sample
                if obj.get('type') == 'accel':
                    # defensive: ensure x,y,z keys exist and are numbers
                    try:
                        x = float(obj['x']); y = float(obj['y']); z = float(obj['z'])
                    except Exception as e:
                        print(f"[{addr}] accel message missing x/y/z or bad types: {obj} ({e})")
                        continue
                    ts = obj.get('ts', time.time())
                    data_q.put((ts, x, y, z))
                elif obj.get('event') == 'significant_motion':
                    ts = obj.get('ts', time.time())
                    event_q.put(ts)
                    print("[EVENT] SIGNIFICANT MOTION at", ts)

    finally:
        try:
            conn.close()
        except:
            pass
        print(f"[client_thread] exiting for {addr}")

def server():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((HOST, PORT))
    s.listen(5)
    print(f"Listening on {HOST}:{PORT}")
    while True:
        conn, addr = s.accept()
        print("Connected:", addr)
        t = threading.Thread(target=client_thread, args=(conn, addr), daemon=True)
        t.start()

# Simple plotting: display magnitude history
xs, ys = [], []
fig, ax = plt.subplots()
ln, = plt.plot([], [], lw=1)
def init():
    ax.set_xlim(0,200)
    ax.set_ylim(0,2)          # start with 0..2 g
    ax.set_title("Accel magnitude (last 200 samples)")
    ax.set_xlabel("samples")
    ax.set_ylabel("magnitude (g)")
    return ln,

def update(frame):
    updated = False
    while not data_q.empty():
        ts, x, y, z = data_q.get()
        # convert from mg -> g (if your STM32 sends mg)
        xg = float(x) / 1000.0
        yg = float(y) / 1000.0
        zg = float(z) / 1000.0
        mag = (xg*xg + yg*yg + zg*zg)**0.5
        ys.append(mag)
        if len(ys) > 200:
            ys.pop(0)
        updated = True

    if updated:
        ln.set_data(range(len(ys)), ys)
        # adjust x limits if needed (keeps window width 200)
        ax.set_xlim(0, 200)
        # autoscale y with a margin: keep min at 0
        ymin = 0.0
        ymax = max(ys) if ys else 1.0
        # avoid zero range
        if ymax <= 0:
            ymax = 1.0
        # add 10% headroom
        margin = ymax * 0.1
        ax.set_ylim(ymin, ymax + margin)

    # highlight events
    while not event_q.empty():
        ts = event_q.get()
        print("[plot] Event ts:", ts)

    return ln,

if __name__ == '__main__':
    import _thread
    _thread.start_new_thread(server, ())
    ani = FuncAnimation(fig, update, init_func=init, interval=200, cache_frame_data=False)
    plt.show()
