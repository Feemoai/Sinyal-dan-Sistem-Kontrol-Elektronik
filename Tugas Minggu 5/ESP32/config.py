# config.py - Configuration for ESP32 Control System

# Wi-Fi
WIFI_SSID = "Nama Wifi"          # <-- ganti
WIFI_PASS = "Password Wifi"      # <-- ganti
AP_SSID = "ESP32-CTRL"           # Access Point SSID jika gagal terhubung WiFi
AP_PASS = "12345678"             # Access Point password

# Control System
TS_MS = 500                      # periode sampling (ms)
TS = TS_MS / 1000.0              # periode sampling (s)
N_LOG = 300                      # jumlah data historis (100-300 titik)
V_MAX = 3.3                      # tegangan maksimum DAC/ADC
SP_MAX = 3.3                     # batas setpoint maksimum

# Controller Parameters (default)
KP0 = 1.0
KI0 = 0.5
KD0 = 0.0
E_REF = 1.0                      # error (V) threshold untuk Adaptive

# Hardware Pins
ADC_PIN = 33                     # GPIO33 - ADC input
DAC_PIN = 25                     # GPIO25 - DAC output
