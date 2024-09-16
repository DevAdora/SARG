import cv2
import threading


thres = 0.45  # Threshold to detect object

# Load class names from coco.names file
classFile = '/home/qwerty/sarg/object_detection/coco.names'
configPath = '/home/qwerty/sarg/object_detection/ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt'
weightsPath = '/home/qwerty/sarg/object_detection/frozen_inference_graph.pb'

classNames = []
with open(classFile, 'rt') as f:
    classNames = f.read().rstrip('\n').split('\n')


# Initialize the detection model
net = cv2.dnn_DetectionModel(weightsPath, configPath)
net.setInputSize(320, 320)
net.setInputScale(1.0 / 127.5)
net.setInputMean((127.5, 127.5, 127.5))
net.setInputSwapRB(True)
def getObjects(img, draw=True):
    classIds, confs, bbox = net.detect(img, confThreshold=thres)
    objectInfo=[]
    if len(classIds) != 0:
        for classId, confidence, box in zip(classIds.flatten(), confs.flatten(), bbox):
            className = classNames[classId - 1]
            objectInfo.append([box, className])
            if (draw):
                    
                cv2.rectangle(img, box, color=(0, 255, 0), thickness=2)
                cv2.putText(img,className.upper(), (box[0] + 10, box[1] + 30),
                            cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 2)
                cv2.putText(img, str(round(confidence * 100, 2)), (box[0] + 200, box[1] + 30),
                            cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 2)
    return img, objectInfo

class VideoStream:
    def __init__(self, src=0):
        self.cap = cv2.VideoCapture(src)
        self.stopped = False
        self.frame = None

    def start(self):
        threading.Thread(target=self.update, args=()).start()
        return self

    def update(self):
        while not self.stopped:
            if not self.cap.isOpened():
                continue
            (self.grabbed, self.frame) = self.cap.read()

    def read(self):
        return self.frame

    def stop(self):
        self.stopped = True

if __name__ == "__main__":
    stream = VideoStream(0).start()  # Or use GStreamer

    frame_count = 0
    while True:
        img = stream.read()
        if img is None:
            continue

        result, objectInfo = getObjects(img, False)
        print(objectInfo)
        cv2.imshow("Live Video", img)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    stream.stop()
    cv2.destroyAllWindows()
 