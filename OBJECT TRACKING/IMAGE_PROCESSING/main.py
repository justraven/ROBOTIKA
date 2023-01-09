import cv2 as cv
import numpy as np

import serial
import time 

face_data = cv.CascadeClassifier('/home/alimsatria/Documents/MATERI KULIAH/ROBOTIKA/IMAGE_PROCESSING/trained_data/haarcascade_frontalcatface.xml')
video = cv.VideoCapture(2)

arduino = serial.Serial(port = '/dev/ttyUSB0', baudrate = 9600, timeout=1)

#frame resolution is 640x480
center_frame_x = 640/2 #pick center of x frame
center_frame_y = 480/2 #pick center of y frame


while(1) :

    ret,frame = video.read()
    frame = cv.flip(frame,1)
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    face_detection = face_data.detectMultiScale(gray, 1.5, 3)

    for (x,y,w,h) in face_detection :
        cv.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
        roi_gray  = gray[y:y+h, x:x+w]
        roi_color = frame[y:y+h, x:x+w]

        x_min = x
        x_max = x + w
        y_min = y
        y_max = y + h
        x_center = (x_min+x_max)/2
        y_center = (y_min+y_max)/2

        error_x = center_frame_x - x_center;
        error_y = center_frame_y - y_center;
        
        # data = '{0:f};{1:f}\n'.format(error_x,error_y) 
        data = '{0:f};{1:f}\n'.format(x_center,y_center)
        arduino.write(data.encode('ascii'))
        print(data);

        time.sleep(50/1000)

        # print(data)
        # print('center_x : ', x_center , '\tcenter_y : ', y_center)

    cv.imshow('frame',frame)

    if cv.waitKey(1) & 0xFF == ord('q'):
        break

video.release()
cv.destroyAllWindows()