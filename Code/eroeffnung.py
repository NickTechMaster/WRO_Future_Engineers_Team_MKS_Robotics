#Bibiliotheken importieren
import RPi.GPIO as GPIO
import time
import cv2
import numpy as np
import threading
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


counter_start = False
button = Button(9)

#Zähler für Ende und blockade für eine gewisse Zeit für blau erkennen
def counter():
    global blue_counter
    global counter_start
    global exit_program
    global counter_ende
    while True:
        if counter_ende == "Ende":
                time.sleep(1.5)
                pins = { 19:GPIO.HIGH, 5:GPIO.HIGH, 6:GPIO.HIGH, 13:GPIO.HIGH }
                for p in list(pins.keys()):
                    GPIO.output(p, pins[p])
                exit_program = False
                exit()
        elif counter_start == True:
            blue_counter = False
            time.sleep(2)
            blue_counter = True
            counter_start = False
       

#zähler starten
thread_counter = threading.Thread(target=counter, args = ())


global blue_counter 
global end_counter 
global exit_program
global counter_ende
counter_ende = False

exit_program = True
end_counter = 0
vid = cv2.VideoCapture(0) 
time.sleep(3)
pins = { 19:GPIO.HIGH, 5:GPIO.HIGH, 6:GPIO.HIGH, 13:GPIO.HIGH }
for p in list(pins.keys()):
    GPIO.output(p, pins[p])
print("start")
button.wait_for_press()
print("start_2")
thread_counter.start()


#farbbereich von blau definieren
lower_blue = np.array([90, 100, 20])
upper_blue = np.array([130, 250, 255])


blue_counter = True


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


motor("forward", 100)
while exit_program == True:
    #startet end zähler nach 12 blau erkannten linien
    if end_counter >= 12:
        counter_ende = "Ende"


    #Ultraschall Sensoren definieren
    rechts = ultraschall(0) 

    links = ultraschall(2)



    #lenkung ausrechnen
    if links < 450 and rechts < 450:

        mittelpunkt = (links + rechts) / 2

        #links
        if links <= rechts:
            lenkung = round(100 - (100 * links / mittelpunkt))

            servo.start(7 - lenkung * 0.017)



        #rechts

        elif rechts <= links:
            lenkung = round(100 - (100 * rechts / mittelpunkt))
            servo.start(7 + lenkung * 0.017)


    area = 0
    area_green = 0
    area_red = 0

    #bild aufnehmen und häller machen
    ret, frame = vid.read()

    Intensity_Matrix=np.ones(frame.shape, dtype = "uint8") * 100

    frame = cv2.add(frame, Intensity_Matrix)
    height, width = frame.shape[:2]

    image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
    #überprüfen ob blau erkannt wird
    mask_green = cv2.inRange (image, lower_blue, upper_blue)
    contours_green, hierarchy = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)  

    if len(contours_green) != 0:
        for contour in contours_green:
            if cv2.contourArea(contour) > 500:
                if blue_counter == True:
                    blue_counter = False
                    end_counter = end_counter + 1
                    print("blue")
                    counter_start = True
        

    #motor stoppen wenn programm manuel beendet wird
    try:
        pass
    except KeyboardInterrupt:
        pins = { 19:GPIO.HIGH, 5:GPIO.HIGH, 6:GPIO.HIGH, 13:GPIO.HIGH }
        for p in list(pins.keys()):
            GPIO.output(p, pins[p])
