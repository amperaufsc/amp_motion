import RPi.GPIO as GPIO
class SignalsController:
    def __init__(self, left, right, PWM):
        GPIO.setmode(GPIO.BOARD)
        self.pwm = GPIO.PWM(PWM, 100)
        self.pwm.start(0)
        self.left = left
        self.right = right
        GPIO.setup(self.left, GPIO.OUT)
        GPIO.setup(self.right, GPIO.OUT)
        self.x = 0

    def steer(self, control):
        self.pwm.ChangeDutyCycle(abs(control))
        if control > 0:
            GPIO.output(self.left, False)
            GPIO.output(self.right, True)  
        elif control < 0:
            GPIO.output(self.left, True)
            GPIO.output(self.right, False)
        else:
            GPIO.output(self.left, False)
            GPIO.output(self.right, False)

    def shutdown(self):
        GPIO.cleanup()
