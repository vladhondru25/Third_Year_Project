import numpy as np
import cv2

face_cascade = cv2.CascadeClassifier('C:\\Users\\VLAD\\Desktop\\PROJECT\\Train_Classifier\\opencv-haar-classifier-training\\trained_classifiers\\chairs3.xml')
#cap = cv2.VideoCapture(0)

#img = cv2.imread('C:\\Users\\VLAD\\Desktop\\test.jpg')
img = cv2.imread('C:\\Users\\VLAD\\Desktop\\PROJECT\\CNN\\101_ObjectCategories\\chair2\\'+"classic-cafe-upholstered-dining-chair-o.jpg")

#gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

#faces = face_cascade.detectMultiScale(gray, 1.3, 5)

#for (x,y,w,h) in faces:
#    cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
#    roi_gray = gray[y:y+h, x:x+w]
#    roi_color = img[y:y+h, x:x+w]
#    eyes = eye_cascade.detectMultiScale(roi_gray)
#    for (ex,ey,ew,eh) in eyes:
#        cv2.rectangle(roi_color,(ex,ey),(ex+ew,ey+eh),(0,255,0),2)


#ret, frame = cap.read()
# Our operations on the frame come here
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

faces = face_cascade.detectMultiScale(gray, 2, 2)
for (x,y,w,h) in faces:
    cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)

    # Display the resulting frame
cv2.imshow('img',img)

# When everything done, release the capture
#cap.release()

#cv2.imshow('img',img)
cv2.waitKey(0)
cv2.destroyAllWindows()
