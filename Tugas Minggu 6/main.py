import network
import time
import gc
import json
import uasyncio as asyncio
from machine import Pin, PWM, ADC, DAC
from array import array

# isi WiFi yang mau dipakai, dicoba berurutan (harus 2.4 GHz)
WIFI_LIST = [("vivoY35", "FERY2006"), ("WiFi Gedung Terpadu", "")]
HOST = "penetas-telur"

TS_MS = 1000
TS = 1.0
N_LOG = 300

pwm = PWM(Pin(26), freq=1000)
pwm.duty_u16(0)
adc = ADC(Pin(33))
adc.atten(ADC.ATTN_11DB)
dac = DAC(Pin(25))

st = {
    "mode": "AI_AUTO", "kp": 18.0, "ki": 0.18, "kd": 20.0,
    "sp": 37.8, "pwm_set": 36.0, "sim": 1, "day_s": 86400, "elapsed": 0.0,
    "amb": 28.0, "al": 36.5, "ah": 38.0, "pr": 39.5, "pause": 0,
    "temp": 0.0, "setpoint": 0.0, "pwm": 0.0, "day": 1, "fase": "", "hum": 55,
    "p": 0.0, "i": 0.0, "d": 0.0, "ff": 0.0, "emb": 0.0, "kf": 1.0, "watt": 0.0,
    "adc_raw": 0, "adc_v": 0.0, "dac_raw": 0, "alarm": "normal", "door": 0,
    "t": 0.0, "n": 0, "ip": "mencari WiFi..."
}
SIMPAN = ("mode", "kp", "ki", "kd", "sp", "pwm_set", "sim", "day_s", "elapsed", "amb", "al", "ah", "pr")

try:
    f = open("config.json")
    data = json.load(f)
    f.close()
    for k in SIMPAN:
        if k in data:
            st[k] = data[k]
except:
    pass


def simpan():
    data = {}
    for k in SIMPAN:
        data[k] = st[k]
    f = open("config.json", "w")
    json.dump(data, f)
    f.close()


# profil 21 hari (titik: hari, nilai), nilai di antaranya dihitung lurus
TP = ((0, 37.8), (7, 37.8), (18, 37.5), (19, 37.2), (21, 37.0))
HP = ((0, 55), (18, 52), (18.3, 67), (21, 70))


def interp(pts, x):
    for i in range(1, len(pts)):
        x0, y0 = pts[i - 1]
        x1, y1 = pts[i]
        if x <= x1:
            return y0 + (y1 - y0) * (x - x0) / (x1 - x0)
    return pts[-1][1]


def read_v():
    s = 0
    for _ in range(16):
        s += adc.read_uv()
    return s / 16 / 1e6


class PID:
    def __init__(self, dt, alpha=0.8):
        self.dt = dt
        self.alpha = alpha
        self.reset()

    def reset(self):
        self.i = 0.0
        self.d = 0.0
        self.prev_pv = None

    def update(self, sp, pv, kp, ki, kd, ff=0.0):
        e = sp - pv
        if self.prev_pv is None:
            self.prev_pv = pv
        d_raw = -(pv - self.prev_pv) / self.dt
        self.d = self.alpha * self.d + (1 - self.alpha) * d_raw
        self.prev_pv = pv

        p = kp * e
        i_new = self.i + ki * e * self.dt
        u = ff + p + i_new + kd * self.d

        if u > 100:
            uc = 100.0
            if e < 0:
                self.i = i_new
        elif u < 0:
            uc = 0.0
            if e > 0:
                self.i = i_new
        else:
            uc = u
            self.i = i_new
        if self.i > 100:
            self.i = 100.0
        if self.i < -100:
            self.i = -100.0

        st["p"], st["i"], st["d"] = p, self.i, kd * self.d
        return uc


pid = PID(TS)
sim_temp = 28.0
door_until = 0
simpan_timer = 0


class Ring:
    def __init__(self, n):
        self.n = n
        self.cols = [array("f", [0.0] * n) for _ in range(4)]
        self.i = 0
        self.cnt = 0

    def add(self, *vals):
        for c, v in zip(self.cols, vals):
            c[self.i] = v
        self.i = (self.i + 1) % self.n
        if self.cnt < self.n:
            self.cnt += 1

    def idx(self):
        start = (self.i - self.cnt) % self.n
        for k in range(self.cnt):
            yield (start + k) % self.n


log = Ring(N_LOG)


def control_step():
    global sim_temp, simpan_timer
    if not st["pause"]:
        st["elapsed"] += TS
    d = st["elapsed"] / st["day_s"]
    if d > 21:
        d = 21.0
    day = int(d) + 1
    if day > 21:
        day = 21

    sp_ai = interp(TP, d)
    hum = int(interp(HP, d) + 0.5)
    emb = 0.0
    if d > 10:
        emb = min(0.6, (d - 10) * 0.06)
    if day <= 7:
        fase = "Pembentukan embrio"
    elif day <= 18:
        fase = "Pertumbuhan embrio"
    else:
        fase = "Fase tetas"

    door = time.ticks_diff(door_until, time.ticks_ms()) > 0

    if st["sim"]:
        loss = 0.01
        if door:
            loss = 0.025
        sim_temp += (0.0027 * st["pwm"] + emb * 0.01 - loss * (sim_temp - st["amb"])) * TS
        if sim_temp > 50:
            sim_temp = 50.0
        if sim_temp < 0:
            sim_temp = 0.0
        raw = int(sim_temp / 50 * 255 + 0.5)
        st["dac_raw"] = raw
        dac.write(raw)

    v = read_v()
    st["adc_v"] = v
    st["adc_raw"] = adc.read()
    if st["sim"]:
        temp = v / 3.3 * 50
    else:
        temp = v * 100

    ff = 0.0
    kf = 1.0
    if st["mode"] == "AI_AUTO":
        sp = sp_ai
        if abs(sp - temp) > 1.0:
            kf = 1.5
        ff = (sp - st["amb"] - emb) * 3.7
        if ff < 0:
            ff = 0.0
        if pid.d < -0.04:
            ff += 10
        u = pid.update(sp, temp, st["kp"] * kf, st["ki"], st["kd"], ff)
    elif st["mode"] == "PID_MANUAL":
        sp = st["sp"]
        u = pid.update(sp, temp, st["kp"], st["ki"], st["kd"])
    else:
        sp = st["sp"]
        u = st["pwm_set"]

    alarm = "normal"
    if temp < st["al"]:
        alarm = "terlalu dingin"
    if temp > st["ah"]:
        alarm = "terlalu panas"
    if temp > st["pr"] or temp < 5 or temp > 60:
        u = 0.0
        alarm = "proteksi aktif"

    pwm.duty_u16(int(u / 100 * 65535))

    st["temp"], st["setpoint"], st["pwm"] = temp, sp, u
    st["day"], st["fase"], st["hum"], st["alarm"] = day, fase, hum, alarm
    st["ff"], st["emb"], st["kf"], st["watt"] = ff, emb, kf, u * 0.4
    st["door"] = 1 if door else 0
    st["t"] = st["t"] + TS
    st["n"] += 1
    log.add(st["t"], sp, temp, u)

    simpan_timer += TS
    if simpan_timer >= 300:
        simpan_timer = 0
        simpan()


async def control_loop():
    nxt = time.ticks_add(time.ticks_ms(), TS_MS)
    while True:
        d = time.ticks_diff(nxt, time.ticks_ms())
        if d > 0:
            await asyncio.sleep_ms(d)
        elif d < -TS_MS:
            nxt = time.ticks_ms()
        nxt = time.ticks_add(nxt, TS_MS)
        control_step()


sta = network.WLAN(network.STA_IF)


async def wifi_task():
    try:
        network.hostname(HOST)
    except Exception:
        pass
    sta.active(True)
    while True:
        if not sta.isconnected():
            st["ip"] = "mencari WiFi..."
            for ssid, pw in WIFI_LIST:
                try:
                    sta.disconnect()
                    sta.connect(ssid, pw)
                except Exception:
                    pass
                for _ in range(15):
                    if sta.isconnected():
                        break
                    await asyncio.sleep(1)
                if sta.isconnected():
                    break
            if sta.isconnected():
                st["ip"] = sta.ifconfig()[0]
                print("Buka: http://%s/ atau http://%s.local/" % (st["ip"], HOST))
        await asyncio.sleep(5)


def parse_q(path):
    q = {}
    if "?" in path:
        path, qs = path.split("?", 1)
        for kv in qs.split("&"):
            if "=" in kv:
                k, v = kv.split("=", 1)
                q[k] = v.replace(",", ".")
    return path, q


def fnum(q, key, lo, hi):
    try:
        v = float(q[key])
    except Exception:
        return None
    return lo if v < lo else hi if v > hi else v


def state_json():
    s = {}
    for k, v in st.items():
        s[k] = round(v, 3) if isinstance(v, float) else v
    return json.dumps(s)


def hist_json():
    out = [[], [], [], []]
    for i in log.idx():
        for j in range(4):
            out[j].append(round(log.cols[j][i], 2))
    return json.dumps({"t": out[0], "sp": out[1], "temp": out[2], "pwm": out[3]})


async def send(w, body, ctype="application/json"):
    w.write("HTTP/1.0 200 OK\r\nContent-Type: %s\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n" % ctype)
    w.write(body)
    await w.drain()


async def handle(r, w):
    global door_until, sim_temp
    try:
        line = await r.readline()
        while True:
            h = await r.readline()
            if not h or h == b"\r\n":
                break
        parts = line.decode().split()
        if len(parts) < 2:
            return
        path, q = parse_q(parts[1])

        if path == "/" or path == "/index.html":
            w.write("HTTP/1.0 200 OK\r\nContent-Type: text/html; charset=utf-8\r\nConnection: close\r\n\r\n")
            with open("index.html", "rb") as f:
                while True:
                    b = f.read(1024)
                    if not b:
                        break
                    w.write(b)
                    await w.drain()

        elif path == "/api/state":
            await send(w, state_json())

        elif path == "/api/hist":
            await send(w, hist_json())

        elif path == "/api/cfg":
            if q.get("mode") in ("AI_AUTO", "PID_MANUAL", "PWM_MANUAL"):
                if q["mode"] != st["mode"]:
                    pid.reset()
                st["mode"] = q["mode"]
            for key, name, lo, hi in (("kp", "kp", 0, 100), ("ki", "ki", 0, 100), ("kd", "kd", 0, 100),
                                      ("sp", "sp", 30, 42), ("pwm_set", "pwm", 0, 100), ("amb", "amb", 10, 40),
                                      ("al", "al", 30, 40), ("ah", "ah", 30, 42), ("pr", "pr", 36, 45)):
                v = fnum(q, name, lo, hi)
                if v is not None:
                    st[key] = v
            simpan()
            await send(w, state_json())

        elif path == "/api/sim":
            if "on" in q:
                st["sim"] = 1 if q["on"] == "1" else 0
                pid.reset()
            v = fnum(q, "day_s", 10, 86400)
            if v is not None:
                d = st["elapsed"] / st["day_s"]
                st["day_s"] = int(v)
                st["elapsed"] = d * st["day_s"]
            simpan()
            await send(w, state_json())

        elif path == "/api/door":
            door_until = time.ticks_add(time.ticks_ms(), 20000)
            await send(w, state_json())

        elif path == "/api/cool":
            sim_temp = st["amb"]
            await send(w, state_json())

        elif path == "/api/pause":
            st["pause"] = 0 if st["pause"] else 1
            await send(w, state_json())

        elif path == "/api/jump":
            v = fnum(q, "day", 1, 21)
            if v is not None:
                st["elapsed"] = (int(v) - 1) * st["day_s"] + 1.0
                pid.reset()
                simpan()
            await send(w, state_json())

        elif path == "/api/batch":
            st["elapsed"] = 0.0
            pid.reset()
            simpan()
            await send(w, state_json())

        else:
            w.write("HTTP/1.0 404 Not Found\r\n\r\nNot found")
            await w.drain()
    except Exception as ex:
        print("HTTP err:", ex)
    finally:
        try:
            w.close()
            await w.wait_closed()
        except Exception:
            pass
        gc.collect()


async def main():
    await asyncio.start_server(handle, "0.0.0.0", 80)
    asyncio.create_task(wifi_task())
    asyncio.create_task(control_loop())
    while True:
        await asyncio.sleep(5)
        gc.collect()


asyncio.run(main())