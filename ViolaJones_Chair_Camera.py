import numpy as np
import cv2

face_cascade = cv2.CascadeClassifier('C:\\Users\\VLAD\\Desktop\\PROJECT\\Train_Classifier\\opencv-haar-classifier-training\\trained_classifiers\\chairs3.xml')
cap = cv2.VideoCapture(0)

while(True):
    ret, frame = cap.read()
    # Our operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    faces = face_cascade.detectMultiScale(gray, 2, 2)
    for (x,y,w,h) in faces:
        cv2.rectangle(frame,(x,y),(x+w,y+h),(255,0,0),2)
        break

    # Display the resulting frame
    cv2.imshow('img',frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()

#cv2.imshow('img',img)
cv2.destroyAllWindows()
