import RPi.GPIO as GPIO
import time
import subprocess
import board
import neopixel
pixels = neopixel.NeoPixel(board.D18, 30)

GPIO.setmode(GPIO.BCM)

GPIO.setup(12, GPIO.OUT)
GPIO.setup(17, GPIO.OUT)


t = True

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
    print("hereee")

def left_stop():
    global freq
    global dc
    freq = 46.5
    dc = 6.98
    left.ChangeFrequency(freq)
    left.ChangeDutyCycle(dc)

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
    right.ChangeFrequency(freq)
    right.ChangeDutyCycle(dc)

def correct_movement():
    global prev_comm
    ### Correct robot movement ###
    if prev_comm == "1": # only correct if robot is moving forwards
        gyro_x = read_raw_data(GYRO_XOUT_H)
        Gx = gyro_x/131.0
        #print("Gx=%.2f" %Gx)
        if Gx > 6.5:
            #print("correct right")
            piv_right_corr(Gx)
        elif Gx < -4:
            #print("correct left")
            piv_left_corr(Gx)

prev_comm = ""
s_prev = '0'
while(t):
    s = input('To quit, type quit: ')
    correct_movement()
    if (s == "quit"):
        t = False
    elif (s == '1' and not(s==s_prev)): # indefinite forwards
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
    #time.sleep(1000)
    time.sleep(1)
    pixels.fill((0,255,255))
    s_prev = s
GPIO.cleanup()
