#
#   Filename: crates.py
#   Authors: Joel Mathew Cinosh             | Avirbhav Dubey
#   E-mails: jademountainacademy0@gmail.com | avirbhavdubey@gmail.com
#   Iterpreter startup: python crates.py 
#   Brief: The main file that orchestrates the entire robot, the robot being CR8S, the patient collection robot.
#   Date: 28-09-2026
#   Version: 1.0.2a
#   License: The MIT Lisence
#   Github URL: https://github.com/nightshade4235/TearX/blob/main/crates.py
#


import time
import math
import board
import busio
import adafruit_tcs34725
import adafruit_vl53l0x
from gpiozero import PWMOutputDevice, DigitalOutputDevice, Button


WHEEL_DIAMETER_MM = 42.0
WHEEL_CIRCUMFERENCE_MM = math.pi * WHEEL_DIAMETER_MM
ENCODER_COUNTS_PER_REV = 0

WALL_STOP_DISTANCE_MM = 25
LINE_TIMEOUT = 8.0
TOF_TIMEOUT = 3.0
HEADING_TOLERANCE = 2.0

LEFT_FRONT_PWM = None
LEFT_FRONT_DIR = None
LEFT_FRONT_ENCODER = None

LEFT_REAR_PWM = None
LEFT_REAR_DIR = None
LEFT_REAR_ENCODER = None

RIGHT_FRONT_PWM = None
RIGHT_FRONT_DIR = None
RIGHT_FRONT_ENCODER = None

RIGHT_REAR_PWM = None
RIGHT_REAR_DIR = None
RIGHT_REAR_ENCODER = None

LEFT_IR_PINS = []
RIGHT_IR_PINS = []

DRUM_STEP_PIN = None
DRUM_DIR_PIN = None
DRUM_HOME_PIN = None
DRUM_STEPS_PER_REV = 0

TOF_FRONT_XSHUT = None
TOF_SIDE_XSHUT = None

SLOT_RED = 1
SLOT_YELLOW_PCC1 = 2
SLOT_YELLOW_PCC2 = 3
SLOT_GREEN = 4

DRUM_CURRENT_SLOT = 1
YELLOW_TOGGLE = 0


def require_configuration():
    values = [
        LEFT_FRONT_PWM,
        LEFT_FRONT_DIR,
        LEFT_FRONT_ENCODER,
        LEFT_REAR_PWM,
        LEFT_REAR_DIR,
        LEFT_REAR_ENCODER,
        RIGHT_FRONT_PWM,
        RIGHT_FRONT_DIR,
        RIGHT_FRONT_ENCODER,
        RIGHT_REAR_PWM,
        RIGHT_REAR_DIR,
        RIGHT_REAR_ENCODER,
        DRUM_STEP_PIN,
        DRUM_DIR_PIN,
        DRUM_HOME_PIN
    ]

    if any(value is None for value in values):
        raise RuntimeError("Hardware GPIO configuration is incomplete")

    if ENCODER_COUNTS_PER_REV <= 0:
        raise RuntimeError("ENCODER_COUNTS_PER_REV must be configured")

    if DRUM_STEPS_PER_REV <= 0:
        raise RuntimeError("DRUM_STEPS_PER_REV must be configured")


class Encoder:
    def __init__(self, pin):
        self.count = 0
        self.sensor = Button(pin, pull_up=True)
        self.sensor.when_pressed = self.increment

    def increment(self):
        self.count += 1

    def reset(self):
        self.count = 0

    def distance_mm(self):
        revolutions = self.count / ENCODER_COUNTS_PER_REV
        return revolutions * WHEEL_CIRCUMFERENCE_MM


class Motor:
    def __init__(self, pwm_pin, direction_pin, encoder_pin):
        self.pwm = PWMOutputDevice(pwm_pin, frequency=1000)
        self.direction = DigitalOutputDevice(direction_pin)
        self.encoder = Encoder(encoder_pin)

    def set(self, speed):
        speed = max(-1.0, min(1.0, speed))

        if speed >= 0:
            self.direction.on()
            self.pwm.value = speed
        else:
            self.direction.off()
            self.pwm.value = abs(speed)

    def stop(self):
        self.pwm.value = 0

    def reset_encoder(self):
        self.encoder.reset()

    def distance_mm(self):
        return self.encoder.distance_mm()


class DriveSystem:
    def __init__(self):
        self.left_front = Motor(
            LEFT_FRONT_PWM,
            LEFT_FRONT_DIR,
            LEFT_FRONT_ENCODER
        )

        self.left_rear = Motor(
            LEFT_REAR_PWM,
            LEFT_REAR_DIR,
            LEFT_REAR_ENCODER
        )

        self.right_front = Motor(
            RIGHT_FRONT_PWM,
            RIGHT_FRONT_DIR,
            RIGHT_FRONT_ENCODER
        )

        self.right_rear = Motor(
            RIGHT_REAR_PWM,
            RIGHT_REAR_DIR,
            RIGHT_REAR_ENCODER
        )

    def set_left(self, speed):
        self.left_front.set(speed)
        self.left_rear.set(speed)

    def set_right(self, speed):
        self.right_front.set(speed)
        self.right_rear.set(speed)

    def set(self, left, right):
        self.set_left(left)
        self.set_right(right)

    def forward(self, speed=0.45):
        self.set(speed, speed)

    def reverse(self, speed=0.45):
        self.set(-speed, -speed)

    def turn_left(self, speed=0.35):
        self.set(-speed, speed)

    def turn_right(self, speed=0.35):
        self.set(speed, -speed)

    def stop(self):
        self.set(0, 0)

    def reset_encoders(self):
        self.left_front.reset_encoder()
        self.left_rear.reset_encoder()
        self.right_front.reset_encoder()
        self.right_rear.reset_encoder()

    def left_distance(self):
        return (
            self.left_front.distance_mm()
            + self.left_rear.distance_mm()
        ) / 2

    def right_distance(self):
        return (
            self.right_front.distance_mm()
            + self.right_rear.distance_mm()
        ) / 2

    def distance(self):
        return (
            self.left_distance()
            + self.right_distance()
        ) / 2


class LineSensors:
    def __init__(self):
        self.left = [
            Button(pin, pull_up=True)
            for pin in LEFT_IR_PINS
        ]

        self.right = [
            Button(pin, pull_up=True)
            for pin in RIGHT_IR_PINS
        ]

    def left_values(self):
        return [sensor.is_pressed for sensor in self.left]

    def right_values(self):
        return [sensor.is_pressed for sensor in self.right]

    def all_high(self):
        values = self.left_values() + self.right_values()
        return len(values) > 0 and all(values)

    def active_count_left(self):
        return sum(self.left_values())

    def active_count_right(self):
        return sum(self.right_values())


class ToFSensors:
    def __init__(self):
        self.i2c = busio.I2C(board.SCL, board.SDA)

        self.front = adafruit_vl53l0x.VL53L0X(self.i2c)
        self.side = adafruit_vl53l0x.VL53L0X(self.i2c)

    def front_mm(self):
        try:
            return self.front.range
        except Exception:
            return 0

    def side_mm(self):
        try:
            return self.side.range
        except Exception:
            return 0

    def front_contact(self):
        distance = self.front_mm()

        if distance <= 0:
            return True

        return distance <= WALL_STOP_DISTANCE_MM

    def side_contact(self):
        distance = self.side_mm()

        if distance <= 0:
            return True

        return distance <= WALL_STOP_DISTANCE_MM


class ColorSensor:
    def __init__(self):
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.sensor = adafruit_tcs34725.TCS34725(self.i2c)
        self.sensor.integration_time = 50
        self.sensor.gain = 4

    def read(self):
        red = 0
        green = 0
        blue = 0

        for _ in range(5):
            r, g, b, _ = self.sensor.color_raw()
            red += r
            green += g
            blue += b
            time.sleep(0.02)

        red /= 5
        green /= 5
        blue /= 5

        total = red + green + blue

        if total <= 0:
            return None

        r = red / total
        g = green / total
        b = blue / total

        if r > 0.42 and r > g * 1.25 and r > b * 1.35:
            return "red"

        if r > 0.30 and g > 0.30 and b < 0.20:
            return "yellow"

        if g > 0.40 and g > r * 1.15 and g > b * 1.20:
            return "green"

        return None


class Drum:
    def __init__(self):
        self.step = DigitalOutputDevice(DRUM_STEP_PIN)
        self.direction = DigitalOutputDevice(DRUM_DIR_PIN)
        self.home = Button(DRUM_HOME_PIN, pull_up=True)
        self.current_slot = DRUM_CURRENT_SLOT

    def pulse(self):
        self.step.on()
        time.sleep(0.001)
        self.step.off()
        time.sleep(0.001)

    def home_drum(self):
        self.direction.off()

        start = time.monotonic()

        while not self.home.is_pressed:
            self.pulse()

            if time.monotonic() - start > 10:
                raise RuntimeError("Drum homing timeout")

        self.current_slot = 1

    def rotate_steps(self, steps):
        if steps == 0:
            return

        self.direction.on()

        for _ in range(steps):
            self.pulse()

    def rotate_to(self, target_slot):
        delta = (target_slot - self.current_slot) % 4

        if delta == 0:
            return

        steps_per_slot = DRUM_STEPS_PER_REV // 4
        self.rotate_steps(delta * steps_per_slot)

        self.current_slot = target_slot

    def load(self):
        time.sleep(0.3)

    def unload(self, slot):
        self.rotate_to(slot)
        time.sleep(0.3)


class IMU:
    def __init__(self):
        self.heading = 0.0

    def read_heading(self):
        return self.heading

    def reset_heading(self):
        self.heading = 0.0

    def available(self):
        return False


class Navigation:
    def __init__(self, drive, lines, tof, imu):
        self.drive = drive
        self.lines = lines
        self.tof = tof
        self.imu = imu

    def reset_checkpoint(self):
        self.drive.reset_encoders()

    def line_checkpoint(self):
        start = time.monotonic()
        detected = False

        while time.monotonic() - start < LINE_TIMEOUT:
            if self.lines.all_high():
                detected = True
                break

            self.drive.forward(0.35)
            time.sleep(0.005)

        self.drive.stop()

        if not detected:
            raise RuntimeError("Line checkpoint timeout")

        self.reset_checkpoint()
        time.sleep(0.15)

    def wall_checkpoint(self):
        start = time.monotonic()

        while time.monotonic() - start < TOF_TIMEOUT:
            if self.tof.front_contact():
                self.drive.stop()
                self.reset_checkpoint()
                return

            self.drive.forward(0.3)
            time.sleep(0.01)

        self.drive.stop()
        raise RuntimeError("ToF checkpoint timeout")

    def drive_distance(self, distance_mm, speed=0.4):
        self.drive.reset_encoders()

        while self.drive.distance() < distance_mm:
            left = self.drive.left_distance()
            right = self.drive.right_distance()

            error = left - right
            correction = error * 0.01

            self.drive.set(
                speed - correction,
                speed + correction
            )

            time.sleep(0.005)

        self.drive.stop()
        self.reset_checkpoint()

    def turn_to(self, target):
        if not self.imu.available():
            self.drive.stop()
            return

        while True:
            current = self.imu.read_heading()
            error = (target - current + 180) % 360 - 180

            if abs(error) <= HEADING_TOLERANCE:
                break

            speed = min(0.5, max(0.18, abs(error) * 0.01))

            if error > 0:
                self.drive.turn_right(speed)
            else:
                self.drive.turn_left(speed)

            time.sleep(0.01)

        self.drive.stop()
        time.sleep(0.15)


class PatientRobot:
    def __init__(self):
        require_configuration()

        self.drive = DriveSystem()
        self.lines = LineSensors()
        self.tof = ToFSensors()
        self.color = ColorSensor()
        self.drum = Drum()
        self.imu = IMU()

        self.navigation = Navigation(
            self.drive,
            self.lines,
            self.tof,
            self.imu
        )

    def slot_for_color(self, color):
        global YELLOW_TOGGLE

        if color == "red":
            return SLOT_RED

        if color == "green":
            return SLOT_GREEN

        if color == "yellow":
            if YELLOW_TOGGLE == 0:
                slot = SLOT_YELLOW_PCC1
            else:
                slot = SLOT_YELLOW_PCC2

            YELLOW_TOGGLE = 1 - YELLOW_TOGGLE
            return slot

        return None

    def collect_patient(self):
        color = None

        for _ in range(5):
            color = self.color.read()

            if color is not None:
                break

            time.sleep(0.1)

        if color is None:
            raise RuntimeError("Unable to identify patient color")

        slot = self.slot_for_color(color)

        if slot is None:
            raise RuntimeError("Invalid patient color")

        self.drum.rotate_to(slot)
        self.drum.load()

    def collect_all_patients(self):
        for _ in range(12):
            self.navigation.drive_distance(200)
            self.collect_patient()

    def unload(self, slot):
        self.drum.unload(slot)

    def run(self):
        try:
            self.drum.home_drum()
            self.navigation.line_checkpoint()
            self.navigation.turn_to(90)

            self.collect_all_patients()

            self.navigation.line_checkpoint()
            self.navigation.turn_to(180)

            self.navigation.line_checkpoint()

            self.navigation.turn_to(90)
            self.navigation.wall_checkpoint()
            self.unload(SLOT_YELLOW_PCC1)

            self.drive.reverse()
            self.navigation.turn_to(180)

            self.drive.reverse()
            self.navigation.turn_to(90)

            self.drive.forward()
            self.unload(SLOT_RED)

            self.navigation.turn_to(270)
            self.navigation.wall_checkpoint()

            self.navigation.turn_to(90)
            self.drive.forward()
            self.unload(SLOT_YELLOW_PCC2)

            self.drive.reverse()
            self.navigation.line_checkpoint()

            self.navigation.turn_to(90)
            self.navigation.wall_checkpoint()

            self.navigation.turn_to(180)
            self.drive.forward()
            self.unload(SLOT_GREEN)

        finally:
            self.drive.stop()


def main():
    robot = PatientRobot()
    robot.run()


if __name__ == "__main__":
    main()
