from array import array
from struct import unpack, unpack_from
import machine
import math

__VERSION__ = "1.0.0"

SCL_PIN = 21
SDA_PIN = 20

# Operating Modes
BME280_OSAMPLE_1 = 1
BME280_OSAMPLE_2 = 2
BME280_OSAMPLE_4 = 3
BME280_OSAMPLE_8 = 4
BME280_OSAMPLE_16 = 5

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
    def __init__(self, mode=BME280_OSAMPLE_1):
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
        self.calibrate_bme280()

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

    def calibrate_bme280(self) -> None:
        # load calibration data
        BME280_REGISTER_CONTROL_HUM = 0xF2
        BME280_REGISTER_CONTROL = 0xF4

        dig_88_a1 = self.i2c.readfrom_mem(0X76, 0x88, 26)
        dig_e1_e7 = self.i2c.readfrom_mem(0X76, 0xE1, 7)
        self.dig_T1, self.dig_T2, self.dig_T3, self.dig_P1, \
            self.dig_P2, self.dig_P3, self.dig_P4, self.dig_P5, \
            self.dig_P6, self.dig_P7, self.dig_P8, self.dig_P9, \
            _, self.dig_H1 = unpack("<HhhHhhhhhhhhBB", dig_88_a1)
 
        self.dig_H2, self.dig_H3 = unpack("<hB", dig_e1_e7)
        e4_sign = unpack_from("<b", dig_e1_e7, 3)[0]
        self.dig_H4 = (e4_sign << 4) | (dig_e1_e7[4] & 0xF)
 
        e6_sign = unpack_from("<b", dig_e1_e7, 5)[0]
        self.dig_H5 = (e6_sign << 4) | (dig_e1_e7[4] >> 4)
 
        self.dig_H6 = unpack_from("<b", dig_e1_e7, 6)[0]
 
        self.i2c.writeto_mem(0X76, BME280_REGISTER_CONTROL,
                             bytearray([0x3F]))
        self.i2c.writeto_mem(0X76, BME280_REGISTER_CONTROL_HUM,
                                bytearray([0x03]))
        self.t_fine = 0
 
    def read_bme280(self) -> None:
        """
        [EN]Read data from BME280 sensor and assign them to class variables

        [TR]BME280 sensöründen verileri oku ve sınıf değişkenlerine ata
        """
        # Read temperature calibration data
        self._l8_barray = bytearray(8)
        self.i2c.readfrom_mem_into(0x76, 0xF7, self._l8_barray)
        readout = self._l8_barray
        # pressure(0xF7): ((msb << 16) | (lsb << 8) | xlsb) >> 4
        raw_press = ((readout[0] << 16) | (readout[1] << 8) | readout[2]) >> 4
        # temperature(0xFA): ((msb << 16) | (lsb << 8) | xlsb) >> 4
        raw_temp = ((readout[3] << 16) | (readout[4] << 8) | readout[5]) >> 4
        # humidity(0xFD): (msb << 8) | lsb
        raw_hum = (readout[6] << 8) | readout[7]
        
        # temperature
        var1 = ((raw_temp >> 3) - (self.dig_T1 << 1)) * (self.dig_T2 >> 11)
        var2 = (((((raw_temp >> 4) - self.dig_T1) *
                  ((raw_temp >> 4) - self.dig_T1)) >> 12) * self.dig_T3) >> 14
        self.t_fine = var1 + var2
        temp = (self.t_fine * 5 + 128) >> 8
 
        # pressure
        var1 = self.t_fine - 128000
        var2 = var1 * var1 * self.dig_P6
        var2 = var2 + ((var1 * self.dig_P5) << 17)
        var2 = var2 + (self.dig_P4 << 35)
        var1 = (((var1 * var1 * self.dig_P3) >> 8) +
                ((var1 * self.dig_P2) << 12))
        var1 = (((1 << 47) + var1) * self.dig_P1) >> 33
        if var1 == 0:
            pressure = 0
        else:
            p = 1048576 - raw_press
            p = (((p << 31) - var2) * 3125) // var1
            var1 = (self.dig_P9 * (p >> 13) * (p >> 13)) >> 25
            var2 = (self.dig_P8 * p) >> 19
            pressure = ((p + var1 + var2) >> 8) + (self.dig_P7 << 4)
 
        # humidity
        h = self.t_fine - 76800
        h = (((((raw_hum << 14) - (self.dig_H4 << 20) -
                (self.dig_H5 * h)) + 16384)
              >> 15) * (((((((h * self.dig_H6) >> 10) *
                            (((h * self.dig_H3) >> 11) + 32768)) >> 10) +
                          2097152) * self.dig_H2 + 8192) >> 14))
        h = h - (((((h >> 15) * (h >> 15)) >> 7) * self.dig_H1) >> 4)
        h = 0 if h < 0 else h
        h = 419430400 if h > 419430400 else h
        humidity = h >> 12

 
        pressure = pressure // 256
        pi = pressure // 100
        pd = pressure - pi * 100
 
        hi = humidity // 1024
        hd = humidity * 100 // 1024 - hi * 100
        
        self.temp = temp / 100
        self.press = int('{}{:02d}'.format(pi, pd))
        self.hum = int('{}{:02d}'.format(hi, hd))
        
        
        # dig_T1 = self.i2c.readfrom_mem(0x76, 0x88, 2)
        # dig_T2 = self.i2c.readfrom_mem(0x76, 0x8A, 2)
        # dig_T3 = self.i2c.readfrom_mem(0x76, 0x8C, 2)
        # dig_P1 = self.i2c.readfrom_mem(0x76, 0x8E, 2)
        # dig_P2 = self.i2c.readfrom_mem(0x76, 0x90, 2)
        # dig_P3 = self.i2c.readfrom_mem(0x76, 0x92, 2)
        # dig_P4 = self.i2c.readfrom_mem(0x76, 0x94, 2)
        # dig_P5 = self.i2c.readfrom_mem(0x76, 0x96, 2)
        # dig_P6 = self.i2c.readfrom_mem(0x76, 0x98, 2)
        # dig_P7 = self.i2c.readfrom_mem(0x76, 0x9A, 2)
        # dig_P8 = self.i2c.readfrom_mem(0x76, 0x9C, 2)
        # dig_P9 = self.i2c.readfrom_mem(0x76, 0x9E, 2)
        # dig_H1 = self.i2c.readfrom_mem(0x76, 0xA1, 1)
        # dig_H2 = self.i2c.readfrom_mem(0x76, 0xE1, 2)
        # dig_H3 = self.i2c.readfrom_mem(0x76, 0xE3, 1)
        # dig_H4 = self.i2c.readfrom_mem(0x76, 0xE4, 2)
        # dig_H5 = self.i2c.readfrom_mem(0x76, 0xE5, 2)

        # # Read measurement data
        # data = self.i2c.readfrom_mem(0x76, 0xF7, 8)
        # press = bytes_toint(data[0], data[1], data[2]) / 1048576
        # temp = bytes_toint(data[3], data[4], data[5]) / 100.0
        # hum = (data[6] << 8 | data[7]) / 1024.0

        # # Compute temperature in Celsius
        # var1 = ((temp / 16384.0) - (dig_T1 / 1024.0)) * dig_T2
        # var2 = (((temp / 131072.0) - (dig_T1 / 8192.0)) ** 2) * dig_T3
        # t_fine = var1 + var2
        # temp_c = t_fine / 5120.0

        # self.temp = temp_c
        # self.press = press
        # self.hum = hum

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