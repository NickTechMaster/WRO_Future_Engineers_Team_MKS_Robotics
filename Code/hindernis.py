#Bibiliotheken importieren
import RPi.GPIO as GPIO
import time
import cv2
import numpy as np
from gpiozero import Button


#GPIO Pins festlegen
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)


GPIO.setup(26, GPIO.OUT)
servo = GPIO.PWM(26, 50)


GPIO.setup(19, GPIO.OUT)
GPIO.output(19, GPIO.HIGH)

GPIO.setup(13, GPIO.OUT)
pwm = GPIO.PWM(13, 50)
pwm.start(0)
GPIO.output(13, GPIO.HIGH)


GPIO.setup(5, GPIO.OUT)

GPIO.setup(6, GPIO.OUT)


GPIO.setup(21, GPIO.OUT)
GPIO.setup(20, GPIO.IN)

GPIO.setup(16, GPIO.OUT)
GPIO.setup(12, GPIO.IN)

GPIO.setup(1, GPIO.OUT)
GPIO.setup(0, GPIO.IN)


lower_green = np.array([42, 30, 20])
upper_green = np.array([68, 250, 255])

lower_red = np.array([1, 70, 20])
upper_red = np.array([6, 190, 255])


vid = cv2.VideoCapture(0) 

time.sleep(4)


#Ultraschall Sensoren ansteuern
def ultraschall(mode):

    if mode == 0:
        GPIO_TRIGGER = 21
        GPIO_ECHO = 20
    elif mode == 1:
        GPIO_TRIGGER = 16
        GPIO_ECHO = 12
    else:
        GPIO_TRIGGER = 1
        GPIO_ECHO = 0
    distanz_all = 0

    GPIO.output(GPIO_TRIGGER, True)

    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)

    StartZeit = time.time()
    StopZeit = time.time()

    while GPIO.input(GPIO_ECHO) == 0:
        StartZeit = time.time()

    while GPIO.input(GPIO_ECHO) == 1:
        StopZeit = time.time()


    TimeElapsed = StopZeit - StartZeit
    distanz = (TimeElapsed * 34300) / 2

    return(round(distanz))


#Motor ansteuern
def motor(mode, speed):
    if mode == "stop":
        pins = { 19:GPIO.HIGH, 5:GPIO.HIGH, 6:GPIO.HIGH, 13:GPIO.HIGH }
        for p in list(pins.keys()):
            GPIO.output(p, pins[p])

    elif mode == "forward":
        pins = { 19:GPIO.HIGH, 5:GPIO.HIGH, 6:GPIO.LOW, 13:GPIO.HIGH }
        for p in list(pins.keys()):
            GPIO.output(p, pins[p])
        pwm.ChangeDutyCycle(speed)

    elif mode == "backward":
        pwm.start(0)
        pins = { 19:GPIO.HIGH, 5:GPIO.LOW, 6:GPIO.HIGH, 13:GPIO.HIGH }
        for p in list(pins.keys()):
            GPIO.output(p, pins[p])
        pwm.ChangeDutyCycle(speed)

direction = ""

while True:
    motor("forward", 100)

    #Ultraschall Sensoren definieren
    rechts = ultraschall(0) 

    links = ultraschall(2)


    if links < 450 and rechts < 450:


        ret, frame = vid.read()


        area = 0
        area_green = 0
        area_red = 0

        
        #bild aufnehmen und häller machen
        Intensity_Matrix=np.ones(frame.shape, dtype = "uint8") * 0

        frame = cv2.add(frame, Intensity_Matrix)
        
        
        height, width = frame.shape[:2]
        area_frame = height * width

        lowest_green = height + 1
        lowest_red = height + 1

        image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        #erkennen ob grün oder rot erkannt wird
        mask_green = cv2.inRange (image, lower_green, upper_green)
        contours_green, hierarchy = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        mask_red = cv2.inRange (image, lower_red, upper_red)
        contours_red, hierarchy = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours_green) != 0:
            for contour in contours_green:
                if cv2.contourArea(contour) > 500:
                    print("color_green_detected")
                    x, y, w, h = cv2.boundingRect(contour)

                    cv2.rectangle(frame, (x,y), (x + w, y + h), (0, 255, 0), 3)
                    area = h * w
                    area_green = area_green + area 
                    if y < lowest_green:
                        lowest_green = y

        if len(contours_red) != 0:
            for contour in contours_red:
                if cv2.contourArea(contour) > 500:
                    print("color_red_detected")
                    x, y, w, h = cv2.boundingRect(contour)
                    cv2.rectangle(frame, (x,y), (x + w, y + h), (0, 0, 255), 2)
                    area = h * w
                    area_red = area_red + area 
                    if y < lowest_red:
                        lowest_red = y
        ready = True


        if round(100 * area_green / area_frame) > 12 and round(100 * area_red / area_frame) > 12:
            if lowest_green > lowest_red:
                print("control green p")

            elif lowest_red > lowest_green:
                print("control red p")
  
        elif round(100 * area_green / area_frame) > 1:
            print("control green")
            rechts = rechts + 72
        elif round(100 * area_red / area_frame) > 10:
            print("control red")
            direction = "innen"
 
        else:
            control = False
            direction = ""


        #lenkung ausrechenen und ausweichmanöver einleiten

        mittelpunkt = (links + rechts) / 2


        if direction == "außen":
            rechts = rechts * 2.6 + 29
        elif direction == "innen":
            links = links * 2.4 + 30


        #links
        if links <= rechts:
            lenkung = round(100 - (100 * links / mittelpunkt))
            servo.start(7 - lenkung * 0.025)


        #rechts

        elif rechts <= links:

            lenkung = round(100 - (100 * rechts / mittelpunkt))
            servo.start(7 + lenkung * 0.025)


    #motor stoppen wenn programm manuel beendet wird
    try:
        pass
    except KeyboardInterrupt:
        pins = { 19:GPIO.HIGH, 5:GPIO.HIGH, 6:GPIO.HIGH, 13:GPIO.HIGH }
        for p in list(pins.keys()):
            GPIO.output(p, pins[p])
