import network
import json
import time

try:
    f = open("config.json")
    cfg = json.load(f)
    f.close()
except:
    cfg = {}

ssid = cfg.get("ssid", "")
password = cfg.get("password", "")

wlan = network.WLAN(network.STA_IF)
wlan.active(True)

if ssid:
    wlan.connect(ssid, password)
    for i in range(20):
        if wlan.isconnected():
            break
        time.sleep(1)

if wlan.isconnected():
    print("IP ESP32:", wlan.ifconfig()[0])
else:
    wlan.active(False)
    ap = network.WLAN(network.AP_IF)
    ap.active(True)
    ap.config(essid="Penetas-Telur", password="12345678")
    print("Mode AP, IP:", ap.ifconfig()[0])