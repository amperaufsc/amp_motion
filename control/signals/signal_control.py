import RPi.GPIO as GPIO
class SignalsController:
    def __init__(self):
        GPIO.setmode(GPIO.BOARD)
        self.pwm = GPIO.PWM(18, 100)
        self.pwm.start(50)
        GPIO.setup(13, GPIO.OUT)

    def change_duty(self, control):
        if control == 0:
            self.pwm.ChangeDutyCycle(50)
        elif control > 0:
            self.pwm.ChangeDutyCycle(100)
        elif control < 0:
            self.pwm.ChangeDutyCycle(0)

