# This program demonstrates how to control an ultrasound distance sensor

# Import the relevant libraries
import RPi.GPIO as GPIO
import time
from adafruit_servokit import ServoKit

# Initialize ServoKit for the PWA board.
kit = ServoKit(channels=16)
channel_servo1 = 0
kit.servo[channel_servo1].set_pulse_width_range(300,2400)

# GPIO Mode (BOARD / BCM)
# GPIO.setmode(GPIO.BOARD)

# set GPIO Pins
TriggerPin  = 18
EchoPin     = 24
# ServoPin = 7
# set GPIO Pins
GPIO_Ain1 = 17
GPIO_Ain2 = 27
GPIO_Apwm = 22
GPIO_Bin1 = 5
GPIO_Bin2 = 6
GPIO_Bpwm = 13

# Set GPIO direction (IN / OUT)
GPIO.setup(GPIO_Ain1, GPIO.OUT)
GPIO.setup(GPIO_Ain2, GPIO.OUT)
GPIO.setup(GPIO_Apwm, GPIO.OUT)
GPIO.setup(GPIO_Bin1, GPIO.OUT)
GPIO.setup(GPIO_Bin2, GPIO.OUT)
GPIO.setup(GPIO_Bpwm, GPIO.OUT)

# Both motors are stopped 
GPIO.output(GPIO_Ain1, False)
GPIO.output(GPIO_Ain2, False)
GPIO.output(GPIO_Bin1, False)
GPIO.output(GPIO_Bin2, False)

# Set PWM parameters
pwm_frequency = 50

# Create the PWM instances
pwmA = GPIO.PWM(GPIO_Apwm, pwm_frequency)
pwmB = GPIO.PWM(GPIO_Bpwm, pwm_frequency)

# Set the duty cycle (between 0 and 100)
# The duty cycle determines the speed of the wheels
pwmA.start(100)
pwmB.start(100)

# set GPIO direction (IN / OUT)
GPIO.setup(TriggerPin, GPIO.OUT)
GPIO.setup(EchoPin, GPIO.IN)

#GPIO.setup(ServoPin, GPIO.OUT)

# Wait for sensor to settle
GPIO.output(TriggerPin, False)
print("Waiting for sensor to settle")
time.sleep(2)
print("Start sensing")

pwm_frequency = 50
duty_min = 2.5 * float(pwm_frequency) / 50.0
duty_max = 12.5 * float(pwm_frequency) / 50.0


def set_duty_cycle(angle):
    return ((duty_max - duty_min) * float(angle) / 180.0 + duty_min)

    # Create a PWM instance
#pwm_servo = GPIO.PWM(ServoPin, pwm_frequency)
# Helper function to get the distance from the ultrasound sensor.
# It returns the measured distance in cm or -1 if it doesn't detect anything nearby.
# The function can take up to 0.25 seconds to execute.
# The details are not important; you should not modify this code
# --- Start of the ultrasound sensor helper function ---
def distance():
    
    # Create a pulse on the trigger pin
    # This activates the sensor and tells it to send out an ultrasound signal
    GPIO.output(TriggerPin, True)
    time.sleep(0.00001)
    GPIO.output(TriggerPin, False)

    # Wait for a pulse to start on the echo pin
    # The response is not valid if it takes too long, and we should break the loop
    valid = True
    RefTime = time.time()
    StartTime = RefTime
    while (GPIO.input(EchoPin) == 0) and (StartTime-RefTime < 0.1):
        StartTime = time.time()
    if (StartTime-RefTime >= 0.1):
        valid = False
        
    # Wait for a pulse to end on the echo pin
    # The response is not valid if it takes too long, and we should break the loop
    if (valid):
        RefTime = time.time()
        StopTime = time.time()
        while (GPIO.input(EchoPin) == 1) and (StopTime-RefTime < 0.1):
            StopTime = time.time()
        if (StopTime-RefTime >= 0.1):
            valid = False
        
    # If we received a complete pulse on the echo pin (i.e., valid == True)
    # Calculate the distance based on the length of the echo pulse and
    # the speed of sound (34300 cm/s)
    if (valid):
        EchoPulseLength = StopTime - StartTime
        return (EchoPulseLength * 34300) / 2        # Divide by 2 because we are calculating based on a reflection, so the travel time there and back
    else:
        return -1

def senseForward(angle):
    # rotate the servo is necessary
    if angle != 85:
        channel = channel_servo1
        angle = 85
        kit.servo[channel].angle = angle
        time.sleep(1)
    # Read from the distance sensor
    dist = distance()
    print("Measured Distance = {0} cm".format(dist))
    return dist

def senseLeft(angle):
    # rotate the servo is necessary
    print("Rotating servo to the left")
    if angle != 0:
        channel = channel_servo1
        angle = 0
        kit.servo[channel].angle = angle
        time.sleep(3)
    print("Servo done rotating to the right") 
    # Read from the distance sensor
    dist = distance()
    print("Measured Distance = {0} cm".format(dist))
    return dist

def senseRight(angle):
    # rotate the servo is necessary
    if angle != 160:
        channel = channel_servo1
        angle = 160
        kit.servo[channel].angle = angle
        time.sleep(3)
    # Read from the distance sensor
    dist = distance()
    print("Measured Distance = {0} cm".format(dist))
    return dist

# --- End of the ultrasound sensor helper function ---

machine_state = 0 # 0 = running, 1 = stop
obstacle_dist = 40 # the minimum distance between the robot and the obstacle
leftTurnDuration = 2.5 # amount of time to take a left turn
rightTurnDuration = 1 # amount of time to take a right turn

def turnRight():
    GPIO.output(GPIO_Ain1, False)
    GPIO.output(GPIO_Ain2, True)
    GPIO.output(GPIO_Bin1, True)
    GPIO.output(GPIO_Bin2, False)
    pwmA.ChangeDutyCycle(60)                # duty cycle between 0 and 100
    pwmB.ChangeDutyCycle(100)

def turnLeft():
    GPIO.output(GPIO_Ain1, True)
    GPIO.output(GPIO_Ain2, False)
    GPIO.output(GPIO_Bin1, False)
    GPIO.output(GPIO_Bin2, True)
    pwmA.ChangeDutyCycle(80)                # duty cycle between 0 and 100
    pwmB.ChangeDutyCycle(80)

def moveForward():
    GPIO.output(GPIO_Ain1, True)
    GPIO.output(GPIO_Ain2, False)
    GPIO.output(GPIO_Bin1, True)
    GPIO.output(GPIO_Bin2, False)
    pwmA.ChangeDutyCycle(80)                # duty cycle between 0 and 100
    pwmB.ChangeDutyCycle(20)                # duty cycle between 0 and 100

def stopMotor():
    # stop the robot when obstacle is detected
    GPIO.output(GPIO_Ain1, False)
    GPIO.output(GPIO_Ain2, False)
    GPIO.output(GPIO_Bin1, False)
    GPIO.output(GPIO_Bin2, False)
    print ("Stop")

# Main program
try:
    # setup
    channel = channel_servo1
    angle = 90
    kit.servo[channel].angle = angle
    time.sleep(1)
    machine_state = 0
    stopMotor()
    # loop    
    # This code repeats forever
    while True:
        if machine_state == 0: # keep going forward until sense obstable
            # keep running until sense obstacles
            moveForward()
            print ("Forward half speed")
            dist = senseForward(angle)
            angle = 90
            if dist >= obstacle_dist:
                machine_state = 0 # remain in moving state
            else:
                machine_state = 1 # change to stop state
        elif machine_state == 1:
            stopMotor()
            dist = senseLeft(angle)
            angle = 0
            if dist < obstacle_dist:
                # If there is an obstacle on the left, sense right instead
                dist = senseRight(angle)
                angle = 180
                if dist < obstacle_dist:
                    stopMotor()
                    # should not happen
                    print("Finshed!")
                    raise KeyboardInterrupt
                else:
                    turnRight()
                    time.sleep(rightTurnDuration)
            else:
                turnLeft()
                time.sleep(leftTurnDuration)
            machine_state = 0 # change machine state back to 0 to star moving again
        else:
            print("unknown machine state!")
            raise KeyboardInterrupt
        time.sleep(0.1)

# Reset by pressing CTRL + C
except KeyboardInterrupt:
        print("Measurement stopped by User")
        pwmA.stop()
        pwmB.stop()
        GPIO.cleanup()
