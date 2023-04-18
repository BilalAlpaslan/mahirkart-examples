import machine
import math

__VERSION__ = "0.0.0"

SCL_PIN = 21
SDA_PIN = 20


def bytes_toint(msb, lsb, xlsb=None):
    '''
    Convert two bytes to signed integer (big endian)
    for little endian reverse msb, lsb arguments
    Can be used in an interrupt handler
    '''
    if not msb & 0x80:
        if xlsb is None:
            return msb << 8 | lsb
        return (msb << 16 | lsb << 8 | xlsb) >> 4
    return - (((msb ^ 255) << 8) | (lsb ^ 255) + 1)


class MahirSensor:
    """
    [EN]Class for reading data from BME280 and MPU6050 sensors on MahirKart with use of I2C 0 on Pin 21 (SCL) and Pin 20 (SDA)

    [TR]MahirKart üzerindeki BME280 ve MPU6050 sensörlerinden veri okumak için I2C 0 üzerinden Pin 21 (SCL) ve Pin 20 (SDA) kullanılarak oluşturulan sınıf
    """
    def __init__(self):
        self.i2c = machine.I2C(0, scl=machine.Pin(SCL_PIN), sda=machine.Pin(SDA_PIN), freq=400000)

        # MPU6050 sensörünü etkinleştir
        self.i2c.writeto_mem(0x68, 0x6B, b'\x00')

        # BME280 sensörünü etkinleştir
        self.i2c.writeto_mem(0x76, 0xF2, b'\x01')
        self.i2c.writeto_mem(0x76, 0xF4, b'\x27')
        self.i2c.writeto_mem(0x76, 0xF5, b'\xA0')

        self.accel_scale = 16384.0
        self.gyro_scale = 131.0

        self.accel_bias = [0, 0, 0]
        self.gyro_bias = [0, 0, 0]
        self.reference = [0, 0, 0]

        self.accel_x = 0
        self.accel_y = 0
        self.accel_z = 0
        self.gyro_x = 0
        self.gyro_y = 0
        self.gyro_z = 0
        self.angle_x = 0
        self.angle_y = 0
        self.angle_z = 0
        self.mpu_temp = 0
        self.temp = 0
        self.press = 0
        self.hum = 0

        self.calibrate_mpu6050(self.accel_bias, self.gyro_bias)

    def calibrate_mpu6050(self, accel_bias: list, gyro_bias: list) -> None:
        """
        [EN]Calibrate MPU6050 sensor

        [TR]MPU6050 sensörünü kalibre et
        """
        self.accel_bias = accel_bias
        self.gyro_bias = gyro_bias

        for _ in range(20):
            self.read_mpu6050()
            self.reference[0] += self.accel_x
            self.reference[1] += self.accel_y
            self.reference[2] += self.accel_z
        self.reference[0] /= 20
        self.reference[1] /= 20
        self.reference[2] /= 20

    def read_bme280(self) -> None:
        """
        [EN]Read data from BME280 sensor and assign them to class variables

        [TR]BME280 sensöründen verileri oku ve sınıf değişkenlerine ata
        """
        # Read temperature calibration data
        dig_T1 = self.i2c.readfrom_mem(0x76, 0x88, 2)
        dig_T1 = bytes_toint(dig_T1[1], dig_T1[0])
        dig_T2 = self.i2c.readfrom_mem(0x76, 0x8A, 2)
        dig_T2 = bytes_toint(dig_T2[1], dig_T2[0])
        dig_T3 = self.i2c.readfrom_mem(0x76, 0x8C, 2)
        dig_T3 = bytes_toint(dig_T3[1], dig_T3[0])

        # Read measurement data
        data = self.i2c.readfrom_mem(0x76, 0xF7, 8)
        press = bytes_toint(data[0], data[1], data[2]) / 1048576
        temp = bytes_toint(data[3], data[4], data[5]) / 100.0
        hum = (data[6] << 8 | data[7]) / 1024.0

        # Compute temperature in Celsius
        var1 = ((temp / 16384.0) - (dig_T1 / 1024.0)) * dig_T2
        var2 = (((temp / 131072.0) - (dig_T1 / 8192.0)) ** 2) * dig_T3
        t_fine = var1 + var2
        temp_c = t_fine / 5120.0

        self.temp = temp_c
        self.press = press
        self.hum = hum

    def read_mpu6050(self) -> None:
        """
        [EN]Read data from MPU6050 sensor and assign them to class variables

        [TR]MPU6050 sensöründen verileri oku ve sınıf değişkenlerine ata
        """
        data = self.i2c.readfrom_mem(0x68, 0x3B, 14)
        accel_x = bytes_toint(data[0], data[1]) / self.accel_scale - self.accel_bias[0]
        accel_y = bytes_toint(data[2], data[3]) / self.accel_scale - self.accel_bias[1]
        accel_z = bytes_toint(data[4], data[5]) / self.accel_scale - self.accel_bias[2]
        mpu_temp = bytes_toint(data[6], data[7]) / 340 + 36.53
        gyro_x = bytes_toint(data[8], data[9]) / self.gyro_scale - self.gyro_bias[0]
        gyro_y =  bytes_toint(data[10], data[11]) / self.gyro_scale - self.gyro_bias[1]
        gyro_z = bytes_toint(data[12], data[13]) / self.gyro_scale - self.gyro_bias[2]

        self.accel_x = accel_x
        self.accel_y = accel_y
        self.accel_z = accel_z
        self.gyro_x = gyro_x
        self.gyro_y = gyro_y
        self.gyro_z = gyro_z
        self.angle_x = math.atan2(accel_y, accel_z) * 180 / math.pi
        self.angle_y = math.atan2(accel_z, accel_x) * 180 / math.pi
        self.angle_z = math.atan2(accel_x, accel_y) * 180 / math.pi
        self.mpu_temp = mpu_temp

    def read(self) -> None:
        """
        [EN]Read data from BME280 and MPU6050 sensors and assign them to class variables

        [TR]BME280 ve MPU6050 sensörlerinden verileri oku ve sınıf değişkenlerine ata
        """
        self.read_bme280()
        self.read_mpu6050()
