import RPi.GPIO as GPIO
import math
import xbox

GPIO_SERVO_PIN = 25
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup(GPIO_SERVO_PIN, GPIO.OUT)

def updateServo(pwm, angle):
    duty = float(angle) / 10.0 + 2.5
    pwm.ChangeDutyCycle(duty)

def angleFromCoords(x,y):
    angle = 0.0
    if x == 0.0 and y == 0.0:
        angle = 90.0
    elif x>= 0.0 and y >= 0.0:
        angle = math.degrees(math.atan(y/x)) if x!=0.0 else 90.0
    elif x<0.0 and y>=0.0:
        angle = math.degrees(math.atan(y/x))
        angle += 180.0
    elif x<0.0 and y<0.0:
        angle = math.degrees(math.atan(y/x))
        angle += 180.0
    elif x>=0.0 and y<0.0:
        angle = math.degrees(math.atan(y/x)) if x!=0.0 else -90.0
        angle += 360.0
    return angle

if __name__ == '__main__':
    joy = xbox.Joystick()
    pwm = GPIO.PWM(GPIO_SERVO_PIN, 100)
    pwm.start(5)

    while not joy.Back():
        x, y = joy.leftStick()
        angle = angleFromCoords(x,y)
        if angle > 180 and angle < 270:
            angle = 180
        elif angle >= 270:
            angle = 0
        updateServo(pwm, angle)

    joy.close()
    pwm.stop()
