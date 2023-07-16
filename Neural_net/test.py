
from tensorflow.keras.models import load_model


model = load_model('arrow_detection.h5')


model.summary()


import cv2
import numpy as np


cam=cv2.VideoCapture(0)
while True:
    _,frame=cam.read()
    raw = cv2.resize(frame,(100,100))
    raw = raw/255
    li = [raw]
    li = np.array(li)
    print(model.predict(li))
    cv2.imshow("Video",frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cam.release()
cv2.destroyAllWindows()






