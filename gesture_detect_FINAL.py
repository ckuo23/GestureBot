#IMPORT STATEMENTS
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
import imutils
import bluetooth


#DEFINE THE CAMERA
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640,480)) #get frames with specified framerate/resolution


#AVERAGE FRAMES TOGETHER AT BEGINNING OF SCRIPT TO GET BACKGROUND
background = None
back_count = 0
avg_weight = 0.5

time.sleep(0.1)


#SET UP BLUETOOTH
port = 1
piZeroMAC = 'B8:27:EB:B9:6C:D3'
sock=bluetooth.BluetoothSocket( bluetooth.RFCOMM )
sock.connect((piZeroMAC, port))


#VARIABLES TO DETERMINE GESTURE
gesture = 0
gesture_string = ""
debounce_gesture = 0
prev_debounce = 0
  
  
#HELPER FUNCTION TO DEBOUNCE GESTURES  
def debouncer(gest,gesture_str):
    global debounce_gesture
    global prev_debounce
    global gesture
    global gesture_string
    
    if debounce_gesture == 0:
        debounce_gesture = 1
        prev_debounce = gest
    else:
        if prev_debounce != gest:
            debounce_gesture = 1
            prev_debounce = gest
        else:
            gesture = gest
            gesture_string = gesture_str
            
            
#ANALYZE ONE FRAME AT A TIME
for frame in camera.capture_continuous(rawCapture, format='bgr', use_video_port=True):
    frm = frame.array #change frame into array so we can use openCV
    
    #INITIAL FILTERING
    gray = cv2.cvtColor(frm,cv2.COLOR_BGR2GRAY) #change from BGR to grayscale
    blur = cv2.GaussianBlur(gray,(5,5),0) #smooth image
    
    #FIND AVERAGE BACKGROUND IMAGE
    if back_count < 20: #if the script has just started, gather background frames
        # 20 is arbitrary, can change if it gives better averaging
        if background is None: #if first frame
            background = blur.copy().astype("float") #copy the filtered frame array
        else: #or else if frame 2 to 20
            cv2.accumulateWeighted(blur,background,avg_weight) #add frame to average
        back_count = back_count + 1
        gesture = 0
        gesture_string = "none"
        
    #FIND HAND AGAINST BACKGROUND    
    else: #if you already have the average background
        if back_count == 20:
            print ("Done averaging background! Please place hand back in frame")
            back_count = 21
        
        #calculate difference between each pixel in background and each pixel in this frame
        find_foreground = cv2.absdiff(background.astype("uint8"),blur)
        
        #if the difference is larger than the threshold, call this pixel part of the foreground
        #and then threshold the foreground vs background
        ret,thresh = cv2.threshold(find_foreground,25,255,cv2.THRESH_BINARY)
        
        #find contours in binary image
        heirarchy, contours, heirarchy = cv2.findContours(thresh,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        
        #create image that will have drawing of contours and defects to display later
        drawing = np.zeros(frm.shape,np.uint8)
        
        #if there is at least one contour, find the biggest one (and call it the hand)
        if len(contours) != 0:
            
            largest_contour = max(contours, key=cv2.contourArea) #find largest contour (this is the hand)
            cv2.drawContours(drawing,[largest_contour],0,(255,255,0),2) #draw hand contour
            
            #find approx contour shape and convex hull
            cnt = cv2.approxPolyDP(largest_contour,0.01*cv2.arcLength(largest_contour,True),True)
            hull = cv2.convexHull(cnt,returnPoints = False) #no return points so correct format for finding defects
            
            #find convexity defects (these are gaps between fingers)
            defects = cv2.convexityDefects(cnt,hull)
            
            if defects is not None: #if defects are found, draw each one as a circle
                for i in range(defects.shape[0]):
                    s,e,f,d = defects[i,0]
                    far = tuple(cnt[f][0])
                    cv2.circle(drawing,far,5,[0,0,255],-1)
                #use number of defects to determine gesture
                if i < 2:
                    debouncer(0,"none") #debounce so must get gesture twice for it to be sent
                elif i == 5:
                    debouncer(1,"forwards")
                elif i == 4:
                    debouncer(2,"backwards")
                elif i == 3:
                    debouncer(3,"right")
                elif i == 2:
                    debouncer(4,"left")
                else:
                    debouncer(5,"stop")
                #print((i+1)) #print number of defects
                i=0 #reset number of defects for next frame
                
            cv2.imshow("output",drawing) #show convex hull and defects in new display
            
        else: #if no defects, no gesture
            gesture = 0
            gesture_string = "none"
            
    cv2.imshow("input",frm) #display original frame
    print (gesture_string)
    sock.send(str(gesture)) #send gesture over bluetooth
    
    key = cv2.waitKey(10) #check for keypress
    rawCapture.truncate(0) #reset for next frame
    
    if key == ord("a"):
        background = None
        back_count = 0
        print ("Re-averaging background... please remove hand from frame")
    if key == ord("q"): #if keypress is "q", quit
        sock.close()
        break