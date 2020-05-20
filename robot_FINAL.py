import RPi.GPIO as GPIO
import time
import subprocess

import smbus            #import SMBus module of I2C
from time import sleep          #import

import bluetooth

# Neopixel setup
import board
import neopixel
pixels = neopixel.NeoPixel(board.D18, 20)

# bluetooth setup
server_sock=bluetooth.BluetoothSocket(bluetooth.RFCOMM)
port=1
server_sock.bind(('',port)) # LEAVE MAC ADDRESS EMPTY STRING
server_sock.listen(1)

# some MPU6050 registers and their addresses
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47


GPIO.setmode(GPIO.BCM)

#set GPIO distance sensor Pins
GPIO_TRIGGER = 23
GPIO_ECHO = 25

GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)

# servos
GPIO.setup(12, GPIO.OUT)
GPIO.setup(17, GPIO.OUT)

GPIO.setup(24, GPIO.OUT) # LED

GPIO.output(24,GPIO.LOW)


t = True

# initialized frequency and duty cycle
freq = 46.5
dc = 6.98

left = GPIO.PWM(12, freq)
left.start(dc)
right = GPIO.PWM(17, freq)
right.start(dc)

def left_cw():
    global freq
    global dc
    freq = 46.95
    dc = 6.10
    left.ChangeFrequency(freq)
    left.ChangeDutyCycle(dc)

def left_ccw():
    global freq
    global dc
    freq = 46.08
    dc = 7.83
    left.ChangeFrequency(freq)
    left.ChangeDutyCycle(dc)

def left_stop():
    global freq
    global dc
    freq = 46.5
    dc = 6.98
    left.stop()
    left.start(0)

def right_cw():
    global freq
    global dc
    freq = 46.95
    dc = 6.10
    right.ChangeFrequency(freq)
    right.ChangeDutyCycle(dc)

def right_ccw():
    global freq
    global dc
    freq = 46.08
    dc = 7.83
    right.ChangeFrequency(freq)
    right.ChangeDutyCycle(dc)

def right_stop():
    global freq
    global dc
    freq = 46.5
    dc = 6.98
    right.stop() # cut signal to servo
    right.start(0)

# measurement returned by ultrasonic range sensor
def distance():
    # set Trigger to HIGH
    GPIO.output(GPIO_TRIGGER, True)

    # set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)

    StartTime = time.time()
    StopTime = time.time()

    # save StartTime
    while GPIO.input(GPIO_ECHO) == 0:
        StartTime = time.time()

    # save time of arrival
    while GPIO.input(GPIO_ECHO) == 1:
        StopTime = time.time()

    # time difference between start and arrival
    TimeElapsed = StopTime - StartTime
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = (TimeElapsed * 34300) / 2

    return distance

# initializing accelerometer
def MPU_Init():
    bus.write_byte_data(Device_Address, SMPLRT_DIV, 7) #write to sample rate register
    bus.write_byte_data(Device_Address, PWR_MGMT_1, 1) #Write to power management register
    bus.write_byte_data(Device_Address, CONFIG, 0) #Write to Configuration register
    bus.write_byte_data(Device_Address, GYRO_CONFIG, 24) #Write to Gyro configuration register
    bus.write_byte_data(Device_Address, INT_ENABLE, 1) #Write to interrupt enable register

def read_raw_data(addr):
    #Accelero and Gyro value are 16-bit
    high = bus.read_byte_data(Device_Address, addr)
    low = bus.read_byte_data(Device_Address, addr+1)

    #concatenate higher and lower value
    value = ((high << 8) | low)

    #to get signed value from mpu6050
    if(value > 32768):
        value = value - 65536
    return value

# correct direction for straight line movement
def piv_right_corr():
    right_cw()
    left_stop()
    time.sleep(.1)
    right_stop()
    execute_robot_command('1')

# correct direction for straight line movement
def piv_left_corr():
    right_stop()
    left_ccw()
    time.sleep(.1)
    left_stop()
    execute_robot_command('1')

def execute_robot_command(s):
    if (s == "0"): # quit
        t = False
    elif (s == '1'): # indefinite forwards
        right_cw()
        left_ccw()
    elif (s == '2'): # indefinite backwards
        right_ccw()
        left_cw()
    elif (s == '3'): # pivot right finite duration
        right_stop()
        left_ccw()
        time.sleep(1.5)
        left_stop()
    elif (s == '4'): # pivot left finite duration
        right_cw()
        left_stop()
        time.sleep(1.5)
        right_stop()
    elif (s == '5'): # stop
        right_stop()
        left_stop()

def valid_command():
    global command_arr
    return all(x == command_arr[0] for x in command_arr)

# Accelerometer initilizations
bus = smbus.SMBus(1)    # or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x68   # MPU6050 device address
MPU_Init()

# Variable initalizations
prev_Ay = 5
coll_arr = [10] * 8
avg = 0
sum_avg = 0
LED_on = 0
start_ctr = False

# Robot starting conditions
left_stop()
right_stop()
s_prev = ""
prev_comm = ''
panic = False


command_arr = [''] * 6 # ensures that two commands in a row means command was intended


try:
    pixels.fill((0,255,255))
    client_sock,address = server_sock.accept()
    print("accepted connection")

    while(t):

        ### Detect collisions ###
        d = distance()
        if d < 30: # centimeters
            print("collision detected")
            GPIO.output(24, GPIO.HIGH) # turn LED on to indicate collision
            pixels.fill((255,0,0)) # red
            start_ctr = True
            execute_robot_command("5") # panic stop
            time.sleep(1)
            execute_robot_command("2") # go backwards
            time.sleep(1)
            execute_robot_command("5") # stop again
            panic = True


        GPIO.output(24, GPIO.LOW) # debugging

        if (panic):
            pixels.fill((255,0,0)) # red
        else:
            pixels.fill((0,0,255)) # blue
            pixels[16] = (0,255,0)
            pixels[15] = (0,255,0)
            pixels[14] = (0,255,0)


        # don't direction correct if collision is detected
        if s_prev == '1' and not(panic):
            gyro_y = read_raw_data(GYRO_YOUT_H)
            Gy = gyro_y/131.0
            if Gy > 4:
                piv_left_corr()
            elif Gy < -4:
                piv_right_corr()


        ### Listen for next command ###
        data = client_sock.recv(1024)
        #print (data.decode('utf-8'))
        s = data.decode('utf-8')
        if len(s) > 1:
            s = s[:1] # use only first character in string
       # s = '1' # override command for debugging
        if (s == '0' or s == '1' or s == '2' or s == '3' or s == '4' or s == '5'):
            if valid_command() and not(s==prev_comm):
                execute_robot_command(s) # if the command is gibberish (not 0-5), nothing happens
                print(s)
                panic = False
                prev_comm = s
            s_prev = s
            command_arr.insert(0,s)
            junk = command_arr.pop()
            #print(command_arr)


        sum_avg = 0
        avg = 0
        time.sleep(.2)

except:
    client_sock.close()
    server_sock.close()
    pixels.fill((0,0,0))
GPIO.cleanup()
