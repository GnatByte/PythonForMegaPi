import serial
import sys
import signal
from time import sleep
import glob
import struct
from multiprocessing import Manager
import threading


class MSerial:
    ser = None

    def __init__(self):
        print(self)

    def start(self, port='/dev/ttyAMA0'):
        self.ser = serial.Serial(port, 115200, timeout=10)

    def device(self):
        return self.ser

    @staticmethod
    def serial_ports():
        if sys.platform.startswith('win'):
            ports = ['COM%s' % (i + 1) for i in range(256)]
        elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
            ports = glob.glob('/dev/tty[A-Za-z]*')
        elif sys.platform.startswith('darwin'):
            ports = glob.glob('/dev/tty.*')
        else:
            raise EnvironmentError('Unsupported platform')

        result = []
        for port in ports:
            s = serial.Serial()
            s.port = port
            s.close()
            result.append(port)
        return result

    def write_package(self, package):
        self.ser.write(package)
        sleep(0.01)

    def read(self):
        return self.ser.read()

    def is_open(self):
        return self.ser.isOpen()

    def in_waiting(self):
        return self.ser.inWaiting()

    def close(self):
        self.ser.close()


M1 = 9
M2 = 10
A0 = 14
A1 = 15
A2 = 16
A3 = 17
A4 = 18
A6 = 19
A7 = 20
A8 = 21
A9 = 22
A10 = 23
A11 = 24


class MegaPi:
    def __init__(self):
        print("init MegaPi")
        signal.signal(signal.SIGINT, self.exit)
        self.manager = Manager()
        self.__selectors = self.manager.dict()
        self.buffer = []
        self.bufferIndex = 0
        self.isParseStart = False
        self.exiting = False
        self.isParseStartIndex = 0
        self.device = MSerial()

    def __del__(self):
        self.exiting = True

    def start(self, port='/dev/ttyAMA0'):
        self.device.start(port)
        sys.excepthook = self.excepthook
        th = threading.Thread(target=self.__on_read, args=(self.on_parse,))
        th.start()

    def excepthook(self, exc_type, value, traceback):
        print('Exception: ' + exc_type + ' Value: ' + value)
        print('TraceBack: ' + traceback)
        self.close()

    def close(self):
        self.device.close()

    def exit(self):
        self.exiting = True
        sys.exit(0)

    def __on_read(self, callback):
        while True:
            if self.exiting:
                break
            try:
                if self.device.is_open():
                    n = self.device.in_waiting()
                    for i in range(n):
                        r = ord(self.device.read())
                        callback(r)
                    sleep(0.01)
                else:
                    sleep(0.5)
            except Exception as ex:
                print(ex)
                self.close()
                sleep(1)

    def __write_package(self, pack):
        self.device.write_package(pack)

    def __write_request_package(self, device_id, port, callback):
        ext_id = ((port << 4) + device_id) & 0xff
        self.__do_callback(ext_id, callback)
        self.__write_package(bytearray([0xff, 0x55, 0x4, ext_id, 0x1, device_id, port]))

    def digital_read(self, pin, callback):
        self.__write_request_package(0x1e, pin, callback)

    def analog_read(self, pin, callback):
        self.__write_request_package(0x1f, pin, callback)

    def light_sensor_read(self, port, callback):
        self.__write_request_package(4, port, callback)

    def ultrasonic_sensor_read(self, port, callback):
        self.__write_request_package(1, port, callback)

    def line_follower_read(self, port, callback):
        self.__write_request_package(17, port, callback)

    def sound_sensor_read(self, port, callback):
        self.__write_request_package(7, port, callback)

    def pir_motion_sensor_read(self, port, callback):
        self.__write_request_package(15, port, callback)

    def potentiometer_read(self, port, callback):
        self.__write_request_package(4, port, callback)

    def limit_switch_read(self, port, callback):
        self.__write_request_package(21, port, callback)

    def temperature_read(self, port, callback):
        self.__write_request_package(2, port, callback)

    def touch_sensor_read(self, port, callback):
        self.__write_request_package(15, port, callback)

    def humidity_sensor_read(self, port, sensor_type, callback):
        device_id = 23
        ext_id = ((port << 4) + device_id) & 0xff
        self.__do_callback(ext_id, callback)
        self.__write_package(bytearray([0xff, 0x55, 0x5, ext_id, 0x1, device_id, port, sensor_type]))

    def joystick_read(self, port, axis, callback):
        device_id = 5
        ext_id = (((port + axis) << 4) + device_id) & 0xff
        self.__do_callback(ext_id, callback)
        self.__write_package(bytearray([0xff, 0x55, 0x5, ext_id, 0x1, device_id, port, axis]))

    def gas_sensor_read(self, port, callback):
        self.__write_request_package(25, port, callback)

    def flame_sensor_read(self, port, callback):
        self.__write_request_package(24, port, callback)

    def compass_read(self, port, callback):
        self.__write_request_package(26, port, callback)

    # ** NO IMPLEMENTATION IN LIB **
    # def angular_sensor_read(self, port, slot, callback):
    #     not_used = slot
    #     self.__write_request_package(28, port, callback)

    def button_read(self, port, callback):
        self.__write_request_package(22, port, callback)

    def gyro_read(self, port, axis, callback):
        device_id = 6
        ext_id = (((port + axis) << 4) + device_id) & 0xff
        self.__do_callback(ext_id, callback)
        self.__write_package(bytearray([0xff, 0x55, 0x5, ext_id, 0x1, device_id, port, axis]))

    def pressure_sensor_begin(self):
        self.__write_package(bytearray([0xff, 0x55, 0x3, 0x0, 0x2, 29]))

    def pressure_sensor_read(self, sensor_type, callback):
        self.__write_request_package(29, sensor_type, callback)

    def digital_write(self, pin, level):
        self.__write_package(bytearray([0xff, 0x55, 0x5, 0x0, 0x2, 0x1e, pin, level]))

    def pwm_write(self, pin, pwm):
        self.__write_package(bytearray([0xff, 0x55, 0x5, 0x0, 0x2, 0x20, pin, pwm]))

    def motor_run(self, port, speed):
        self.__write_package(bytearray([0xff, 0x55, 0x6, 0x0, 0x2, 0xa, port]
                                       + self.short2bytes(speed)))

    def motor_move(self, left_speed, right_speed):
        self.__write_package(bytearray([0xff, 0x55, 0x7, 0x0, 0x2, 0x5]
                                       + self.short2bytes(-left_speed)
                                       + self.short2bytes(right_speed)))

    def servo_run(self, port, slot, angle):
        self.__write_package(bytearray([0xff, 0x55, 0x6, 0x0, 0x2, 0xb, port, slot, angle]))

    def encoder_motor_run(self, slot, speed):
        device_id = 62
        self.__write_package(bytearray([0xff, 0x55, 0x07, 0x00, 0x02, device_id, 0x02, slot]
                                       + self.short2bytes(speed)))

    def encoder_motor_move(self, slot, speed, distance, callback):
        device_id = 62
        ext_id = ((slot << 4) + device_id) & 0xff
        self.__do_callback(ext_id, callback)
        self.__write_package(bytearray([0xff, 0x55, 0x0b, ext_id, 0x02, device_id, 0x01, slot]
                                       + self.long2bytes(distance)
                                       + self.short2bytes(speed)))

    def encoder_motor_move_to(self, slot, speed, distance, callback):
        device_id = 62
        ext_id = ((slot << 4) + device_id) & 0xff
        self.__do_callback(ext_id, callback)
        self.__write_package(bytearray([0xff, 0x55, 0x0b, ext_id, 0x02, device_id, 0x06, slot]
                                       + self.long2bytes(distance)
                                       + self.short2bytes(speed)))

    def encoder_motor_set_current_position_zero(self, slot):
        device_id = 62
        self.__write_package(bytearray([0xff, 0x55, 0x05, 0x00, 0x02, device_id, 0x04, slot]))

    def encoder_motor_position(self, slot, callback):
        device_id = 61
        ext_id = ((slot << 4) + device_id) & 0xff
        self.__do_callback(ext_id, callback)
        self.__write_package(bytearray([0xff, 0x55, 0x06, ext_id, 0x01, device_id, 0x00, slot, 0x01]))

    def encoder_motor_speed(self, slot, callback):
        device_id = 61
        ext_id = ((slot << 4) + device_id) & 0xff
        self.__do_callback(ext_id, callback)
        self.__write_package(bytearray([0xff, 0x55, 0x06, ext_id, 0x01, device_id, 0x00, slot, 0x02]))

    def stepper_motor_run(self, slot, speed):
        device_id = 76
        self.__write_package(bytearray([0xff, 0x55, 0x07, 0x00, 0x02, device_id, 0x02, slot]
                                       + self.short2bytes(speed)))

    def stepper_motor_move(self, port, speed, distance, callback):
        device_id = 76
        ext_id = ((port << 4) + device_id) & 0xff
        self.__do_callback(ext_id, callback)
        self.__write_package(bytearray([0xff, 0x55, 0x0b, ext_id, 0x02, device_id, 0x01, port]
                                       + self.long2bytes(distance)
                                       + self.short2bytes(speed)))

    def stepper_motor_move_to(self, port, speed, distance, callback):
        device_id = 76
        ext_id = ((port << 4) + device_id) & 0xff
        self.__do_callback(ext_id, callback)
        self.__write_package(bytearray([0xff, 0x55, 0x0b, ext_id, 0x02, device_id, 0x06, port]
                                       + self.long2bytes(distance)
                                       + self.short2bytes(speed)))

    def stepper_motor_set_current_position_zero(self, port):
        device_id = 76
        self.__write_package(bytearray([0xff, 0x55, 0x05, 0x00, 0x02, device_id, 0x04, port]))

    def rgb_led_display(self, port, slot, index, red, green, blue):
        self.__write_package(bytearray([0xff, 0x55, 0x9, 0x0, 0x2, 0x8, port, slot, index, int(red), int(green),
                                        int(blue)]))

    def rgb_led_show(self, port, slot):
        self.__write_package(bytearray([0xff, 0x55, 0x5, 0x0, 0x2, 19, port, slot]))

    def seven_segment_display(self, port, value):
        self.__write_package(bytearray([0xff, 0x55, 0x8, 0x0, 0x2, 9, port]
                                       + self.float2bytes(value)))

    def led_matrix_message(self, port, x, y, message):
        arr = list(message)
        for i in range(len(arr)):
            arr[i] = ord(arr[i])
        self.__write_package(bytearray([0xff, 0x55, 8 + len(arr), 0, 0x2, 41, port, 1, self.char2byte(x),
                                        self.char2byte(7 - y), len(arr)] + arr))

    def led_matrix_display(self, port, x, y, buffer):
        self.__write_package(bytearray([0xff, 0x55, 7 + len(buffer), 0, 0x2, 41, port, 2, x, 7 - y] + buffer))

    def shutter_on(self, port):
        self.__write_package(bytearray([0xff, 0x55, 0x5, 0, 0x3, 20, port, 1]))

    def shutter_off(self, port):
        self.__write_package(bytearray([0xff, 0x55, 0x5, 0, 0x3, 20, port, 2]))

    def focus_on(self, port):
        self.__write_package(bytearray([0xff, 0x55, 0x5, 0, 0x3, 20, port, 3]))

    def focus_off(self, port):
        self.__write_package(bytearray([0xff, 0x55, 0x5, 0, 0x3, 20, port, 4]))

    def on_parse(self, byte):
        value = 0
        self.buffer += [byte]
        buffer_length = len(self.buffer)
        if buffer_length >= 2:
            if self.buffer[buffer_length - 1] == 0x55 and self.buffer[buffer_length - 2] == 0xff:
                self.isParseStart = True
                self.isParseStartIndex = buffer_length - 2
            if self.buffer[buffer_length - 1] == 0xa and self.buffer[buffer_length - 2] == 0xd and self.isParseStart:
                self.isParseStart = False
                position = self.isParseStartIndex + 2
                ext_id = self.buffer[position]
                position += 1
                parse_type = self.buffer[position]
                position += 1
                # 1 byte 2 float 3 short 4 len+string 5 double
                if parse_type == 1:
                    value = self.buffer[position]
                if parse_type == 2:
                    value = self.read_float(position)
                if value < -512 or value > 1023:
                    value = 0
                if parse_type == 3:
                    value = self.read_short(position)
                if parse_type == 4:
                    value = self.read_string(position)
                if parse_type == 5:
                    value = self.read_double(position)
                if parse_type == 6:
                    value = self.read_long(position)
                if parse_type <= 6:
                    self.response_value(ext_id, value)
                self.buffer = []

    def read_float(self, position):
        v = [self.buffer[position], self.buffer[position + 1], self.buffer[position + 2], self.buffer[position + 3]]
        return struct.unpack('<f', struct.pack('4B', *v))[0]

    def read_short(self, position):
        v = [self.buffer[position], self.buffer[position + 1]]
        return struct.unpack('<h', struct.pack('2B', *v))[0]

    def read_string(self, position):
        position_range = self.buffer[position]
        position += 1
        s = ""
        for i in range(position_range):
            s += self.buffer[position + i].charAt(0)
        return s

    def read_double(self, position):
        v = [self.buffer[position], self.buffer[position + 1], self.buffer[position + 2], self.buffer[position + 3]]
        return struct.unpack('<f', struct.pack('4B', *v))[0]

    def read_long(self, position):
        v = [self.buffer[position], self.buffer[position + 1], self.buffer[position + 2], self.buffer[position + 3]]
        return struct.unpack('<l', struct.pack('4B', *v))[0]

    def response_value(self, ext_id, value):
        self.__selectors["callback_" + str(ext_id)](value)

    def __do_callback(self, ext_id, callback):
        self.__selectors["callback_" + str(ext_id)] = callback

    @staticmethod
    def float2bytes(float_value):
        val = struct.pack("f", float_value)
        return [val[0], val[1], val[2], val[3]]

    @staticmethod
    def long2bytes(long_value):
        val = struct.pack("=l", long_value)
        return [val[0], val[1], val[2], val[3]]

    @staticmethod
    def short2bytes(short_value):
        val = struct.pack("h", short_value)
        return [val[0], val[1]]

    @staticmethod
    def char2byte(char_value):
        val = struct.pack("b", char_value)
        return val[0]
