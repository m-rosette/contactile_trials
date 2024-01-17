import cv2

cap = cv2.VideoCapture(0) 
ret,frame = cap.read()
cv2.imwrite('measured_yaxis2' + '.jpg', frame)
cap.release()
