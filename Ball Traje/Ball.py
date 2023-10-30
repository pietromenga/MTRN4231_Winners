import cv2
import numpy as np

kf = cv2.KalmanFilter(4,2)
kf.measurementMatrix = np.array([[1,0,0,0],[0,1,0,0]], np.float32)
kf.transitionMatrix = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]], np.float32)

def Estimate(x,y):
    measured = np.array([[np.float32(x)], [np.float32(y)]])
    kf.correct(measured)
    predicted = kf.predict()

    return predicted

def findingCountours(frame):
    x,y,w,h = 0,0,0,0
    contours,_=cv2.findContours(frame,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        area = cv2.contourArea(contour)

        if area > 500:
            peri = cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour,0.02*peri, True)
            (x,y),radius = cv2.minEnclosingCircle(approx)
    
    return x, y

def Masking(frame):
    lower = np.array([4,60,115])
    upper = np.array([30,255,255])
    mask = cv2.inRange(frame, lower, upper)

    k = np.ones((10,10))
    maskDilated = cv2.dilate(mask, k)
    maskC = cv2.bitwise_and(frame, frame,mask-maskDilated)
    maskC = cv2.resize(maskC,(800,800))
    
    return maskDilated, maskC

def drawing(frame, points):
    for i in range(1,len(points)):
        cv2.line(frame, points[i-1], points[i], (250,0,0), 2)

vid = cv2.VideoCapture('vid (1).mp4')
points = []
predicted = np.zeros((2,1), np.float32)
count = 0

while (vid.isOpened()):
    ret, frame = vid.read()

    if (ret == True):
        mask,_ = Masking(frame)
        _,maskC = Masking(frame)
        
        nextPoint = []
        x,y = findingCountours(mask)
        nextPoint.append((int(x), int(y)))

        for i in nextPoint:
            points.append(i)
        
        predicted = Estimate(x,y)

        if (0,0) in points:
            points.remove((0,0))

        if len(points) > 15:
            del points[0]
        drawing(frame, points)

        # Draw actual coordinates
        cv2.circle(frame, (int(x), int(y)), 20, [250,0,0], 2, 7)
        cv2.line(frame, (int(x), int(y + 20)), (int(x+50), int(y)), [0,0,0], 2, 7)
        cv2.putText(frame, "Actual", (int(x+50), int(y+20)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, [250,0,0])

        # Drawing kalman filter predicted output
        cv2.circle(frame, (int(predicted[0]), int(predicted[1])), 20, [0,0,255], 2, 7)
        cv2.line(frame, (int(predicted[0] + 16), int(predicted[1] - 15)), (int(predicted[0] + 50), int(predicted[1]- 30)), [0,0,0], 2, 7)
        cv2.putText(frame, "Predicted", (int(predicted[0]+50), int(predicted[1]-30)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, [0,0,255])
        frame = cv2.resize(frame, (800, 800))

        cv2.imshow('final', frame)
        k = cv2.waitKey(100)

        if cv2.waitKey(10) & 0xFF == ord('q'):
            break
    else:
        break

print("finsih")
vid.release()
cv2.destroyAllWindows()

