#!/usr/bin/python3
import RPi.GPIO as GPIO
import time

# Motor direction
STOP  = 0
FORWARD  = 1
BACKWARD = 2

#PWM PIN
ENA = 16  # BCM25 36 pin for DC Motor
ENS = 14  # BCM12 32 pin for Servo

#GPIO PIN
IN1 = 20   # BCM20 38 pin
IN2 = 21   # BCM21 40 pin

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False) # disable warning

GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(ENS, GPIO.OUT)

# Hocar Class
class Hocar:
    def __init__(self, enAPin, inPin1, inPin2, enSPin, init_speed, init_angle):
        # GPIO pin configuration 
        self.enAPin_ = enAPin
        self.inPin1_ = inPin1
        self.inPin2_ = inPin2
        self.enSPin_ = enSPin
        
        self.dc_pwm = GPIO.PWM(self.enAPin_, 1000)
        self.dc_pwm.start(0)
        self.servo_pwm = GPIO.PWM(self.enSPin_, 50)
        self.servo_pwm.start(0)
        
        self.speed_ = init_speed 
        self.angle_ = init_angle

    def change_speed(self, speed):
        self.speed_ = speed
        if self.speed_ < 0:
            self.speed_ = 0
        self.dc_pwm.ChangeDutyCycle(self.speed_)
       
    def setMotorControl(self, speed, status):
        self.change_speed(speed)
        if status == FORWARD:
            GPIO.output(self.inPin1_, True)
            GPIO.output(self.inPin2_, False)
            # print('forward...')
            
        elif status == BACKWARD:
            GPIO.output(self.inPin1_, False)
            GPIO.output(self.inPin2_, True)
            # print('backward...')
            
        elif status == STOP:
            GPIO.output(self.inPin1_, False)
            GPIO.output(self.inPin2_, False)
            print('stop')
            
    def setServoAngle(self, angle):
        self.angle_ = angle
        duty = self.angle_ / 18 + 2.5
        GPIO.output(self.enSPin_, True)
        self.servo_pwm.ChangeDutyCycle(duty)
        time.sleep(0.1) # delay
        GPIO.output(self.enSPin_, False)
        
    def __del__(self):
        # stop
        self.speed_ = 0
        self.dc_pwm.stop()
        self.servo_pwm.stop()
        GPIO.cleanup()

# Test
if __name__ == "__main__":
    mt = Hocar(ENA, IN1, IN2, ENS, 0, 90)
    
    for i in range(15):
        mt.setMotorControl(80, FORWARD)
        mt.setServoAngle(120)
        time.sleep(0.2)

    mt.setMotorControl(0, STOP)
    time.sleep(2.0)

    for i in range(15):
        mt.setMotorControl(80, BACKWARD)
        mt.setServoAngle(55)
        time.sleep(0.2)

    mt.setMotorControl(0, STOP)
    mt.setServoAngle(90)
    time.sleep(1.0)
    
    GPIO.cleanup()
