from mahirkart import MahirSensor
import time

mk = MahirSensor()


while True:
    mk.read()
    print(mk.accel_x, mk.accel_y, mk.accel_z, mk.gyro_x, mk.gyro_y, mk.gyro_z, mk.angle_x, mk.angle_y, mk.angle_z, mk.mpu_temp, mk.temp, mk.press, mk.hum)
    time.sleep(0.2)
