import RPi.GPIO as GPIO
class SignalsController:
    def __init__(self):
        GPIO.setmode(GPIO.BOARD)
        self.pwm = GPIO.PWM(18, 100)
        self.pwm.start(50)
        self.left = 13
        self.right = 15
        GPIO.setup(self.left, GPIO.OUT)
        GPIO.setup(self.right, GPIO.OUT)

    def steer(self, control):
        if control >= 0:
            GPIO.output(self.left, False)
            GPIO.output(self.right, True)
        elif control <= 0:
            GPIO.output(self.left, True)
            GPIO.output(self.right, False)
        self.pwm.ChangeDutyCycle(abs(control))
