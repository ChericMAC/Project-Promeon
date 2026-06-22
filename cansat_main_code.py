import threading
import time
import math
import csv
import serial
import pynmea2
import board
import adafruit_icm20x
import adafruit_mlx90614
from adafruit_bme280 import basic as adafruit_bme280
import numpy as np
from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory
import smbus2

# pyLoRa - Configuración de Radio
from SX127x.LoRa import LoRa
from SX127x.board_config import BOARD as HW
from SX127x.constants import MODE

# --- CONFIGURACIÓN MAG GPS ---
ADDR_MAG = 0x0D

# Inicializar hardware LoRa
HW.setup()

# --- CONFIGURACIÓN DE PINES Y PARÁMETROS ---
factory = PiGPIOFactory()
PIN_SERVO_IZQ = 12
PIN_SERVO_DER = 13
GPS_PORT = "/dev/serial0"
FRECUENCIA_LOG = 0.5

LOG_LOCAL = "/home/promeon/vuelo_respaldo.csv"
LOG_WINDOWS = "/boot/firmware/datos_cansat.csv"

MIN_DIST_GPS = 2.0  # Metros mínimos de desplazamiento para actualizar yaw por GPS


class TransmisorLoRa:
    def __init__(self):
        self.lora = LoRa(4, 22, 0)
        try:
            self.lora.set_mode(MODE.SLEEP)
            time.sleep(0.1)
            self.lora.set_mode(MODE.STDBY)
            time.sleep(0.1)
        except Exception as e:
            print(f"Error cambiando modos LoRa: {e}")

        self.lora.set_pa_config(pa_select=1, max_power=0x0F, output_power=0x0F)
        self.lora.set_bw(0x07)
        self.lora.set_spreading_factor(7)
        self.lora.set_coding_rate(1)
        self.lora.set_preamble(8)
        self.lora.set_implicit_header_mode(False)
        self.lora.set_sync_word(0x12)
        self.lora.set_low_data_rate_optim(True)
        self.lora.set_rx_crc(True)
        print("LoRa Ra-02 configurado (2km+)")

    def enviar_telemetria(self, d, fase):
        try:
            trama = (
                f"{fase};{int(round(d['lat'], 5) * 100000)};{int(round(d['lon'], 5) * 100000)};"
                f"{int(round(d['alt_pres'], 1) * 10)};{int(round(d['hum'], 1)*10)};"
                f"{int(round(d['temp_amb'], 1)*10)};{int(round(d['temp_suelo'], 2)*100)}"
            )
            payload = list(trama.encode('utf-8'))
            self.lora.set_mode(MODE.STDBY)
            self.lora.write_payload(payload)
            self.lora.set_mode(MODE.TX)
            time.sleep(0.1)
            self.lora.set_mode(MODE.STDBY)
            print(f"Radio -> Enviado: {trama}")
        except Exception as e:
            print(f"Error en radio: {e}")


class NavegacionCanSat:
    def __init__(self, cansat_system):
        self.cs = cansat_system
        self.servo_izq = Servo(PIN_SERVO_IZQ, pin_factory=factory)
        self.servo_der = Servo(PIN_SERVO_DER, pin_factory=factory)

        self.rt = False
        self.lt = False
        self.moviendo_izq = False
        self.moviendo_der = False

        self.distancia_a_home = 0
        self.rumbo_a_home = 0
        self.umbral_buitre = 25.0
        self.umbral_error_giro = 15.0

    def calcular_distancia(self, lat1, lon1, lat2, lon2):
        R = 6371000
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlambda = math.radians(lon2 - lon1)
        a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2
        return R * (2 * math.atan2(math.sqrt(a), math.sqrt(1-a)))

    def calcular_rumbo(self, lat1, lon1, lat2, lon2):
        lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
        dLon = lon2 - lon1
        y = math.sin(dLon) * math.cos(lat2)
        x = math.cos(lat1)*math.sin(lat2) - math.sin(lat1)*math.cos(lat2)*math.cos(dLon)
        return (math.degrees(math.atan2(y, x)) + 360) % 360

    def _ejecutar_movimiento(self, lado, valor, duracion, estado_final):
        try:
            servo = self.servo_izq if lado == "IZQ" else self.servo_der
            servo.value = valor
            time.sleep(duracion)
            servo.value = None
        finally:
            if lado == "IZQ":
                self.lt = estado_final
                self.moviendo_izq = False
            else:
                self.rt = estado_final
                self.moviendo_der = False

    def aplicar_tension(self, izq_pct, der_pct):
        if izq_pct <= 5:
            if self.lt and not self.moviendo_izq:
                self.moviendo_izq = True
                threading.Thread(target=self._ejecutar_movimiento, args=("IZQ", -0.3, 1, False), daemon=True).start()
        else:
            if not self.lt and not self.moviendo_izq:
                self.moviendo_izq = True
                threading.Thread(target=self._ejecutar_movimiento, args=("IZQ", 0.3, 1, True), daemon=True).start()

        if der_pct <= 5:
            if self.rt and not self.moviendo_der:
                self.moviendo_der = True
                threading.Thread(target=self._ejecutar_movimiento, args=("DER", -0.3, 1, False), daemon=True).start()
        else:
            if not self.rt and not self.moviendo_der:
                self.moviendo_der = True
                threading.Thread(target=self._ejecutar_movimiento, args=("DER", 0.3, 1, True), daemon=True).start()

    def actualizar_navegacion(self):
        if self.cs.home_lat is None or self.cs.datos['lat'] == 0:
            self.aplicar_tension(0, 0)
            return

        with self.cs.lock:
            lat, lon, yaw = self.cs.datos['lat'], self.cs.datos['lon'], self.cs.datos['yaw']

        self.distancia_a_home = self.calcular_distancia(lat, lon, self.cs.home_lat, self.cs.home_long)
        self.rumbo_a_home = self.calcular_rumbo(lat, lon, self.cs.home_lat, self.cs.home_long)

        if self.distancia_a_home < self.umbral_buitre:
            alt_relativa = self.cs.datos['alt_pres'] - (self.cs.home_alt or 0)
            if alt_relativa > 10:
                self.aplicar_tension(0, 40)
            else:
                self.aplicar_tension(0, 0)
            return

        error_giro = (self.rumbo_a_home - yaw + 180) % 360 - 180

        if abs(error_giro) <= self.umbral_error_giro:
            self.aplicar_tension(0, 0)
        else:
            factor = min(abs(error_giro) / 90.0, 1.0)
            tension = 30 + factor * 40
            if error_giro > 0:
                self.aplicar_tension(0, tension)
            else:
                self.aplicar_tension(tension, 0)


class CanSatSystem:
    def __init__(self):
        self.roll_offset = 87.849044
        self.pitch_offset = 3.513806
        self.running = True
        self.datos = {
            "lat": 0.0, "lon": 0.0, "alt_pres": 0.0, "temp_amb": 0.0, "hum": 0.0,
            "roll": 0.0, "pitch": 0.0, "yaw": 0.0, "acc_z": 0.0, "temp_suelo": 0.0, "ts": 0
        }
        self.home_lat = self.home_long = self.home_alt = None
        self._prev_gps = None  # Punto GPS anterior para calcular yaw por diferencia
        self.descenso = False
        self.lock = threading.Lock()
        self.historial_alt = []

        # Sensores I2C
        try:
            self.i2c = board.I2C()
            self.bme = adafruit_bme280.Adafruit_BME280_I2C(self.i2c, address=0x76)
            self.imu = adafruit_icm20x.ICM20948(self.i2c, address=0x68)
            self.termica = adafruit_mlx90614.MLX90614(self.i2c)
            print("I2C OK")
        except Exception as e:
            print(f"Fallo I2C: {e}")

        # GPS NMEA
        try:
            self.ser = serial.Serial(GPS_PORT, baudrate=9600, timeout=0.1)
            self.ser.write(b"\xb5\x62\x06\x08\x06\x00\xf4\x01\x01\x00\x01\x00\x0b\x77")
            print("GPS 2Hz OK")
        except Exception as e:
            print(f"Fallo GPS: {e}"); self.ser = None

    def enviar_datos_radio(self):
        if not hasattr(self, 'radio'):
            try: self.radio = TransmisorLoRa()
            except: self.radio = None

        if self.radio:
            estado = "D" if self.descenso else "V"
            with self.lock:
                snap = self.datos.copy()
            self.radio.enviar_telemetria(snap, estado)

    def control_servos(self):
        navegador = NavegacionCanSat(self)
        while self.running:
            with self.lock:
                alt_actual = self.datos["alt_pres"]

            if not self.descenso and self.home_alt is not None:
                self.historial_alt.append(alt_actual)
                if len(self.historial_alt) > 20:
                    self.historial_alt.pop(0)
                    if self.historial_alt[-1] < self.historial_alt[0] - 5 and (alt_actual - self.home_alt > 100):
                        self.descenso = True
                        print(">>> APOGEO DETECTADO <<<")

            if self.descenso:
                navegador.actualizar_navegacion()
            else:
                navegador.aplicar_tension(0, 0)
            time.sleep(0.1)

    def hilo_sensores_i2c(self):
        while self.running:
            try:
                t, h, a = self.bme.temperature, self.bme.relative_humidity, self.bme.altitude
                acc = self.imu.acceleration

                # Roll y Pitch (ICM Vertical)
                r = np.degrees(np.arctan2(acc[1], acc[2])) - self.roll_offset
                p = np.degrees(np.arctan2(-acc[0], np.sqrt(acc[1]**2 + acc[2]**2))) - self.pitch_offset

                # --- YAW POR DIFERENCIA GPS ---
                with self.lock:
                    prev = self._prev_gps
                    lat_act = self.datos["lat"]
                    lon_act = self.datos["lon"]
                    y = self.datos["yaw"]  # Valor por defecto: mantener último yaw

                if prev and prev[0] != 0 and lat_act != 0:
                    d_lat = math.radians(lat_act - prev[0])
                    d_lon = math.radians(lon_act - prev[1])
                    lat1_r = math.radians(prev[0])
                    lat2_r = math.radians(lat_act)
                    x_r = math.sin(d_lon) * math.cos(lat2_r)
                    x_l = math.cos(lat1_r) * math.sin(lat2_r) - math.sin(lat1_r) * math.cos(lat2_r) * math.cos(d_lon)
                    # Distancia aproximada para filtrar GPS estático
                    dist = math.sqrt(d_lat**2 + d_lon**2) * 6371000
                    if dist >= MIN_DIST_GPS:
                        y = (math.degrees(math.atan2(x_r, x_l)) + 360) % 360
                # Si dist < MIN_DIST_GPS o no hay prev, se mantiene el último yaw

                with self.lock:
                    self.datos.update({
                        "temp_amb": t, "hum": h, "alt_pres": a,
                        "roll": r, "pitch": p, "yaw": y,
                        "acc_z": acc[2], "temp_suelo": self.termica.object_temperature, "ts": time.time()
                    })
                if a != 0 and self.home_alt is None:
                    self.home_alt = a

            except:
                pass
            time.sleep(0.02)  # 50 Hz

    def hilo_gps(self):
        while self.running:
            try:
                if self.ser and self.ser.in_waiting > 0:
                    linea = self.ser.readline().decode('ascii', errors='replace').strip()
                    if "GGA" in linea:
                        msg = pynmea2.parse(linea)
                        if msg.gps_qual > 0:
                            lat_nueva, lon_nueva = msg.latitude, msg.longitude
                            with self.lock:
                                # Guardar posición anterior antes de sobrescribir
                                self._prev_gps = (self.datos["lat"], self.datos["lon"])
                                self.datos["lat"], self.datos["lon"] = lat_nueva, lon_nueva
                            if self.home_lat is None:
                                self.home_lat, self.home_long = lat_nueva, lon_nueva
                                print(f"Home: {self.home_lat}, {self.home_long}")
            except:
                pass
            time.sleep(0.01)

    def hilo_log_y_radio(self):
        try:
            with open(LOG_LOCAL, 'a', newline='') as f1, open(LOG_WINDOWS, 'a', newline='') as f2:
                writer1 = csv.DictWriter(f1, fieldnames=self.datos.keys())
                writer2 = csv.DictWriter(f2, fieldnames=self.datos.keys())

                if f1.tell() == 0: writer1.writeheader()
                if f2.tell() == 0: writer2.writeheader()

                ciclo = 0
                while self.running:
                    with self.lock: snap = self.datos.copy()

                    writer1.writerow(snap)
                    writer2.writerow(snap)

                    f1.flush()
                    f2.flush()

                    if ciclo % 2 == 0: self.enviar_datos_radio()
                    ciclo += 1
                    time.sleep(FRECUENCIA_LOG)
        except Exception as e:
            print(f"Error escribiendo Logs: {e}")


if __name__ == "__main__":
    cansat = CanSatSystem()
    hilos = [
        threading.Thread(target=cansat.hilo_sensores_i2c, daemon=True),
        threading.Thread(target=cansat.hilo_gps, daemon=True),
        threading.Thread(target=cansat.hilo_log_y_radio, daemon=True),
        threading.Thread(target=cansat.control_servos, daemon=True),
    ]
    for t in hilos: t.start()
    print("--- CanSat Online ---")
    try:
        while True: time.sleep(1)
    except KeyboardInterrupt:
        cansat.running = False
        print("\nCerrando...")
