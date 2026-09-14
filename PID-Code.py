from machine import Pin, PWM, I2C
import time
import math
from imu import MPU6050 # imu is premade object used for extracted MPU6050 data
from time import sleep

# IMU Setup -------------------------------------------------------------
i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400000)
imu = MPU6050(i2c)
imu.accel_range = 2      # +/-8g range
imu.filter_range = 3     # Low pass filter against vibration, filters 41 Hz

def read_a(axis): #reads the linear acceleration of the rocket
    return getattr(imu.accel, axis)
def read_w(axis): #reads the angular velocity of the rocket
    return getattr(imu.gyro, axis)

# Servo Setup -------------------------------------------------------
servo_xy = PWM(Pin(17))   # xy plane = top servo
servo_zy = PWM(Pin(16))   # zy plane = bottom servo
servo_xy.freq(50)
servo_zy.freq(50)
gimbal_limit = 5.0 # degrees
servo_center_us = 1500
us_per_degree = 500/90

def clamp(num, limit):
    if num > limit:
        num = limit
    elif num < -limit:
        num = -limit
    return num

def set_gimbal(servo, degree):
    degree = clamp(degree, gimbal_limit)
    pulse_ns = (servo_center_us + degree*us_per_degree) * 1000 # calculates degree in microseconds then convert to nano
    servo.duty_ns(int(pulse_ns))

# Gyro calibration -------------------------------------------------
wxsum, wzsum = 0.0, 0.0
sample = 500
for n in range(sample): # takes a sample of angular velocities over sample number of iterations, then finds average bias for gyroscope
    wxsum += read_w("x")
    wzsum += read_w("z")
    time.sleep_ms(2)
bias_x = wxsum/sample 
bias_z = wzsum/sample
    
# Complementary Filter Alpha
alpha = 1 # gyro vs accel weight for PID (ONLY USING GYRO FOR POWERED FLIGHT, alpha = 1)

# PID Class for xy/zy ---------------------------------------------
setpoint = 0.0
class PID:
    def __init__(self, kp, ki, kd, i_limit):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.integral = 0.0
        self.prev_error = 0.0
        self.i_limit = i_limit
        
    def update(self, setpoint, measured, dt):
        # Proportional
        error = setpoint - measured
        p = self.kp * error
        
        # Integral
        self.integral += error * dt
        self.integral = clamp(self.integral, self.i_limit)
        i = self.ki * self.integral
        
        # Derivative
        derivative = (error-self.prev_error)/dt
        d = self.kd * derivative
        
        self.prev_error = error
        return p+i+d
    
pid_xy = PID(kp=0.5, ki=0.2, kd=0.1, i_limit = 10) # PID gains are placeholders for hold-down testing
pid_zy = PID(kp=0.5, ki=0.2, kd=0.1, i_limit = 10)

# Main Loop ----------------------------------------------
led = Pin("LED", Pin.OUT)
led_last = time.ticks_ms()
led_interval = 500

last = time.ticks_us()
state = "IDLE"
threshold = 3.0 # g's - threshold for launch state activation

ax, ay, az = read_a("x"), read_a("y"), read_a("z")
prelaunch_angle_xy = math.degrees(math.atan2(ax, ay))
prelaunch_angle_zy = math.degrees(math.atan2(az, ay))
angle_xy = prelaunch_angle_xy
angle_zy = prelaunch_angle_zy

while True:
    if time.ticks_diff(time.ticks_ms(), led_last) >= led_interval: # blinks the led without putting microcontroller to sleep
        led.toggle()
        led_last = time.ticks_ms()
    now = time.ticks_us()
    dt = time.ticks_diff(now, last)/1_000_000 # gives loop time in seconds
    last = now 
    if dt <= 0:
        continue
    
    # Angular velocities and linear accelerations
    ax, ay, az = read_a("x"), read_a("y"), read_a("z")
    wx = read_w("x") - bias_x
    wz = read_w("z") - bias_z
    
    # Angles calculated using the gyroscope and accelerometer data
    gyro_xy = angle_xy + wz*dt
    accel_xy = math.degrees(math.atan2(ax, ay))
    gyro_zy = angle_zy + wx*dt
    accel_zy = math.degrees(math.atan2(az, ay))
    
    # Angle estimates - Complementary Filter for sensor fusion in case I need to switch back to it. alpha = 1 so only gyroscope values taken into account
    angle_xy = (1-alpha)*accel_xy + alpha*gyro_xy
    angle_zy = (1-alpha)*accel_zy + alpha*gyro_zy
    
    if state == "IDLE":
        set_gimbal(servo_xy, 0)
        set_gimbal(servo_zy, 0)
        angle_xy = accel_xy
        angle_zy = accel_zy
        print("state", state, "angle_xy", angle_xy, "angle_zy", angle_zy) # angle estimate from accelerometer before launch
        if math.sqrt(ax**2 + ay**2 + az**2) >= threshold: # detect launch
            state = "ACTIVE"
            
    elif state == "ACTIVE":
        new_xy = pid_xy.update(setpoint, angle_xy, dt)
        new_zy = pid_zy.update(setpoint, angle_zy, dt)
        set_gimbal(servo_xy, -new_xy) # negative due to servo setup / gimbal geometry
        set_gimbal(servo_zy, new_zy)
        print("state", state, "angle_xy", angle_xy, "angle_zy", angle_zy, "new_xy", new_xy, "new_zy", new_zy)
    
        
        
    

