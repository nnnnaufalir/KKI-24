import numpy as np
import cv2
from ultralytics import YOLO
import math
import redis
import json
import time
from pymavlink import mavutil

# Menghubungkan ke Redis dengan IP host tertentu
client = redis.StrictRedis(host='103.59.95.141', port=6379, db=0)

# Menghubungkan ke flight controller melalui port USB/serial
print("Connecting to vehicle...")
# connection = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
connection = mavutil.mavlink_connection('COM17', baud=115200)
connection.wait_heartbeat()
print("Connected to vehicle!")

# Override nilai RC1-RC4
rc1_value_awal = 1500  # Nilai untuk RC channel 1 (range: 1000-2000)
rc2_value_awal = 1500  # Nilai untuk RC channel 2 (range: 1000-2000)
rc3_value_awal = 1000  # Nilai untuk RC channel 3 (range: 1000-2000)
rc4_value_awal = 1500  # Nilai untuk RC channel 4 (range: 1000-2000)

# lurus#
rc1_value_lurus = 1500  # Nilai untuk RC channel 2 (range: 1000-2000)
rc3_value_lurus = 1500

# kiri mentok#
rc1_value_kiri_mentok = 1000  # Nilai untuk RC channel 2 (range: 1000-2000)

# kanan mentok#
rc1_value_kanan_mentok = 2000  # Nilai untuk RC channel 2 (range: 1000-2000)


# kiri sitik
rc1_value_kiri_sitik = 1300

# kanan sitik
rc1_value_kanan_sitik = 1800
# Nilai untuk RC channel 2 (range: 1000-2000)

# Fungsi kirim gambar ke redis


def store_image(image_path, key):
    """Store an image in Redis."""
    # Read the image in binary mode
    with open(image_path, 'rb') as image_file:
        image_data = image_file.read()

    # Store the image in Redis under the given key
    client.set(key, image_data)
    print(f"Image stored in Redis with key: {key}")

# Fungsi untuk mendapatkan data dari vehicle (flight controller)


def override_rc_channels(connection, rc1_value, rc2_value, rc3_value, rc4_value):
    # Mengirimkan perintah COMMAND_LONG untuk override channel RC1-RC4
    try:
        connection.mav.rc_channels_override_send(
            # target_system (ID dari flight controller)
            connection.target_system,
            # target_component (biasanya ID dari flight controller)
            connection.target_component,
            rc1_value,  # Nilai override untuk RC channel 1 (1000-2000)
            rc2_value,  # Nilai override untuk RC channel 2 (1000-2000)
            rc3_value,  # Nilai override untuk RC channel 3 (1000-2000)
            rc4_value,  # Nilai override untuk RC channel 4 (1000-2000)
            # Set remaining channels (RC5 to RC8) to 0 (no override)
            0, 0, 0, 0
        )
        print(
            f"RC channels overridden: RC1={rc1_value}, RC2={rc2_value}, RC3={rc3_value}, RC4={rc4_value}")
    except Exception as e:
        print(f"Failed to override RC channels: {e}")


def get_vehicle_data():
    # Menerima pesan MAVLink
    msg_gps = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    msg_att = connection.recv_match(type='ATTITUDE', blocking=True)
    msg_bat = connection.recv_match(type='SYS_STATUS', blocking=True)
    msg_vfr = connection.recv_match(type='VFR_HUD', blocking=True)
    msg_rc = connection.recv_match(type='RC_CHANNELS', blocking=True)

    # Coba untuk menerima pesan SERVO_OUTPUT_RAW, tapi tidak wajib blocking=True
    msg_servo = connection.recv_match(type='SERVO_OUTPUT_RAW', blocking=False)

    # Mengambil data GPS (lokasi, ketinggian, dan ground course)
    gps = {
        'lat': msg_gps.lat / 1e7,   # Latitude
        'lon': msg_gps.lon / 1e7,   # Longitude
        'alt': msg_gps.alt / 1e3,   # Altitude (meters)
        'cog': msg_gps.hdg / 100.0  # Course over ground (degrees)
    }

    # Mengambil data status baterai
    batt_status = {
        'voltage': msg_bat.voltage_battery / 1000.0,  # Tegangan baterai dalam volt
        'current': msg_bat.current_battery / 100.0,   # Arus baterai dalam Ampere
        'level': msg_bat.battery_remaining            # Level baterai dalam %
    }

    # Mengambil data attitude (roll, pitch, yaw)
    attitude = {
        'roll': math.degrees(msg_att.roll),
        'pitch': math.degrees(msg_att.pitch),
        'yaw': math.degrees(msg_att.yaw)
    }

    # Mengambil data kecepatan tanah dalam beberapa satuan
    speed = {
        'ground_speed': msg_vfr.groundspeed,    # Groundspeed dalam m/s
        'kmh': msg_vfr.groundspeed * 3.6,       # Groundspeed dalam km/h
        'knot': msg_vfr.groundspeed * 1.94384   # Groundspeed dalam knot
    }

    # Mengambil data tambahan lainnya
    heading = msg_vfr.heading  # Compass heading
    baro = msg_vfr.alt  # Altitude relatif dari barometer

    # Mengambil data RC channel
    rc_channels = {
        'rc1': msg_rc.chan1_raw,
        'rc2': msg_rc.chan2_raw,
        'rc3': msg_rc.chan3_raw,
        'rc4': msg_rc.chan4_raw,
        'rc5': msg_rc.chan5_raw,
        'rc6': msg_rc.chan6_raw
    }

    # Mengambil nilai servo output jika tersedia
    if msg_servo:
        servo_output = {
            'steer': msg_servo.servo1_raw,  # Servo 1
            'th_mid': msg_servo.servo3_raw,  # Servo 3
            'th_left': msg_servo.servo5_raw,  # Servo 5
            'th_right': msg_servo.servo7_raw  # Servo 7
        }
    else:
        # Jika pesan servo tidak ada, set sebagai None atau nilai default
        servo_output = {
            'steer': None,
            'th_mid': None,
            'th_left': None,
            'th_right': None
        }

    # Mendapatkan mode kendaraan
    msg_heartbeat = connection.recv_match(type='HEARTBEAT', blocking=True)
    mode = mavutil.mode_string_v10(msg_heartbeat)

    # Memeriksa apakah kendaraan armed
    is_armed = msg_heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED > 0

    # Memeriksa apakah kendaraan dapat di-arm
    is_armable = (
        mode in ["GUIDED", "AUTO", "STABILIZE"] and
        batt_status['level'] > 20 and  # Level baterai lebih dari 20%
        msg_bat.current_battery >= 0  # Status sistem tidak menunjukkan masalah
    )

    # Mengembalikan semua data dalam satu dictionary
    return {
        "gps": gps,
        "bat_status": batt_status,
        "heading": heading,
        "speed": speed,
        "baro": baro,
        "attitude": attitude,
        "rc_channels": rc_channels,  # Data RC
        "servo_output": servo_output,  # Data Servo
        "mode": mode,                  # Mode kendaraan
        "is_armed": is_armed,           # Status armed
        "is_armable": is_armable,        # Status armable
    }

# Fungsi untuk memposting data ke Redis


def post_data(data):
    # Mengambil data yang relevan untuk dikirim ke Redis
    information = {
        "latitude": data['gps']['lat'],
        "longitude": data['gps']['lon'],
        "cog": data['gps']['cog'],  # Course Over Ground
        "sog_knot": data['speed']['knot'],  # Speed Over Ground dalam knots
        "sog_kmh": data['speed']['kmh'],  # Speed Over Ground dalam km/h
    }

    # Mengirim data ke Redis dengan key 'username'
    client.set('username', json.dumps(information))

# Fungsi untuk arming dan disarming


def set_arm_state(arm: bool):
    mode = "GUIDED" if arm else "STABILIZE"
    connection.mav.set_mode_send(
        connection.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE, mode)

    if arm:
        connection.mav.command_long_send(
            connection.target_system,
            connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # Not in mission
            1,  # Arm
            0, 0, 0, 0, 0, 0  # No additional parameters
        )
    else:
        connection.mav.command_long_send(
            connection.target_system,
            connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # Not in mission
            0,  # Disarm
            0, 0, 0, 0, 0, 0  # No additional parameters
        )


print(get_vehicle_data())
################### -----------OPENCV-------###################
print("start cv")
# Membaca file coco.txt untuk daftar kelas
my_file = open("coco.txt", "r")
data = my_file.read()
class_list = data.split("\n")
my_file.close()

print(class_list)

# Warna deteksi untuk bola merah dan hijau
# Merah untuk bola merah, hijau untuk bola hijau
detection_colors = [[0, 0, 255], [0, 255, 0]]

print("Load Model")

model = YOLO("tutut2.pt", "v8")

# Ukuran frame video
frame_wid = 640
frame_hyt = 640
img_counter = 0

print("Start Camera")
cap = cv2.VideoCapture("A.mp4")
if not cap.isOpened():
    print("Cannot open camera")
    exit()

print("run")
prev_frame_time = 0  # Waktu sebelumnya
new_frame_time = 0   # Waktu saat ini

# Main loop untuk terus mengambil dan mengirim data
try:
    while True:
        # Mengambil data dari flight controller
        data = get_vehicle_data()
        post_data(data)

        # Mulai menghitung waktu
        new_frame_time = time.time()

        ret, frame = cap.read()

        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break

        # Resize frame untuk optimasi
        frame = cv2.resize(frame, (frame_wid, frame_hyt))

        # Melakukan prediksi pada frame
        detect_params = model.predict(
            source=[frame], conf=0.3, save=False, imgsz=640)

        DP = detect_params[0].numpy()

        red_ball = None  # Variabel untuk menyimpan bola merah terdekat
        green_ball = None  # Variabel untuk menyimpan bola hijau terdekat
        red_center = None  # Titik tengah bola merah
        green_center = None  # Titik tengah bola hijau
        min_dist_red = float('inf')  # Jarak minimum untuk bola merah
        min_dist_green = float('inf')  # Jarak minimum untuk bola hijau

        frame_center = np.array([320, 640])  # Pusat frame baru di (320, 0)

        if len(DP) != 0:
            for i in range(len(detect_params[0])):
                boxes = detect_params[0].boxes
                box = boxes[i]  # Mengambil satu bounding box
                # Mendapatkan ID kelas (misalnya, merah atau hijau)
                clsID = int(box.cls.numpy()[0])
                # Koordinat bounding box (x1, y1, x2, y2)
                bb = box.xyxy.numpy()[0]

                # Menghitung titik tengah dari bounding box
                box_center = np.array(
                    [(bb[0] + bb[2]) // 2, (bb[1] + bb[3]) // 2])

                # Menghitung jarak bounding box ke pusat frame (320, 0)
                dist_to_center = np.linalg.norm(frame_center - box_center)

                # Kategori bola merah
                if clsID == 1:  # Misal ID kelas 1 adalah bola merah
                    if dist_to_center < min_dist_red:
                        red_ball = box
                        red_center = box_center  # Simpan titik tengah bola merah
                        min_dist_red = dist_to_center
                # Kategori bola hijau
                elif clsID == 0:  # Misal ID kelas 0 adalah bola hijau
                    if dist_to_center < min_dist_green:
                        green_ball = box
                        green_center = box_center  # Simpan titik tengah bola hijau
                        min_dist_green = dist_to_center

        # Menampilkan bola merah terdekat
        if red_ball is not None:
            bb = red_ball.xyxy.numpy()[0]
            cv2.rectangle(
                frame,
                (int(bb[0]), int(bb[1])),
                (int(bb[2]), int(bb[3])),
                detection_colors[0],  # Warna merah
                2,
            )
            cv2.putText(
                frame,
                "Red Ball",
                (int(bb[0]), int(bb[1]) - 10),
                cv2.FONT_HERSHEY_COMPLEX,
                1,
                (255, 255, 255),
                2,
            )
            # Menampilkan titik tengah bola merah
            cv2.circle(frame, (int(red_center[0]), int(
                red_center[1])), 5, detection_colors[0], -1)

        # Menampilkan bola hijau terdekat
        if green_ball is not None:
            bb = green_ball.xyxy.numpy()[0]
            cv2.rectangle(
                frame,
                (int(bb[0]), int(bb[1])),
                (int(bb[2]), int(bb[3])),
                detection_colors[1],  # Warna hijau
                2,
            )
            cv2.putText(
                frame,
                "Green Ball",
                (int(bb[0]), int(bb[1]) - 10),
                cv2.FONT_HERSHEY_COMPLEX,
                1,
                (255, 255, 255),
                2,
            )
            # Menampilkan titik tengah bola hijau
            cv2.circle(frame, (int(green_center[0]), int(
                green_center[1])), 5, detection_colors[1], -1)

        # Menggambar garis yang menghubungkan bola merah dan hijau jika keduanya terdeteksi
        if red_center is not None and green_center is not None:
            # Menggambar garis antara bola merah dan hijau
            cv2.line(frame, (int(red_center[0]), int(red_center[1])),
                     (int(green_center[0]), int(green_center[1])), (255, 255, 255), 2)

            # Menghitung titik tengah dari garis penghubung antara bola merah dan hijau
            mid_point = np.array([(red_center[0] + green_center[0]) // 2,
                                  (red_center[1] + green_center[1]) // 2])

            # Gambar titik tengah garis penghubung
            cv2.circle(frame, (int(mid_point[0]), int(mid_point[1])),
                       5, (255, 255, 255), -1)

            # Menggambar garis dari pusat frame ke titik tengah garis penghubung
            cv2.line(frame, (int(frame_center[0]), int(frame_center[1])),
                     (int(mid_point[0]), int(mid_point[1])), (0, 255, 255), 2)

        # Menghitung FPS
        fps = 1 / (new_frame_time - prev_frame_time)
        prev_frame_time = new_frame_time

        # Menampilkan FPS pada frame
        cv2.putText(
            frame,
            f'FPS: {int(fps)}',
            (10, 50),  # Posisi teks FPS pada frame
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (255, 255, 255),  # Warna teks (putih)
            2,
            cv2.LINE_AA,
        )

        # Menampilkan frame yang sudah diproses
        cv2.imshow('ASV KKI 2024', frame)

        k = cv2.waitKey(1)
        if k % 256 == 32:
            # SPACE pressed
            img_name = "opencv_frame_{}.png".format(img_counter)
            cv2.imwrite(img_name, frame)
            print("{} written!".format(img_name))
            img_counter += 1
            if img_counter == 1:
                # Example usage
                image_path = img_name            # Replace with your image path
                key_1 = 'my_image_3'                    # Key to store the image
                store_image(image_path, key_1)
            elif img_counter == 2:
                # Example usage
                image_path = img_name            # Replace with your image path
                key_2 = 'my_image_4'                    # Key to store the image
                store_image(image_path, key_2)

        # Menghentikan loop saat tombol 'q' ditekan
        if cv2.waitKey(1) == ord('q'):
            break


except Exception as e:
    print(f"An error occurred: {e}")

finally:
    # Menutup koneksi dengan aman
    connection.close()
    cap.release()
    cv2.destroyAllWindows()
