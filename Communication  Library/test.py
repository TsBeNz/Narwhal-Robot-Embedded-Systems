import cv2
cv = cv2.VideoCapture(0)
  
while(True):   
    ret, frame = cv.read()
    cv2.imshow('frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
  
cv.release()
cv2.destroyAllWindows()