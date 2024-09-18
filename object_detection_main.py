import cv2
import threading
import numpy as np
import RPi.GPIO as GPIO

# Thresholds and GPIO Pins
thres = 0.45  # Threshold to detect object
nms_threshold = 0.2
button_pin_raigen = 23
button_pin_raint = 22
terminate_button = 16
focal = 450  # Adjust according to your setup
width = 4  # Example width of object for distance calculation (adjust as necessary)

# Load class names from coco.names file
classFile = '/home/qwerty/sarg/object_detection/coco.names'
configPath = '/home/qwerty/sarg/object_detection/ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt'
weightsPath = '/home/qwerty/sarg/object_detection/frozen_inference_graph.pb'

classNames = []
with open(classFile, 'rt') as f:
    classNames = f.read().rstrip('\n').split('\n')

# Initialize GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(button_pin_raigen, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(button_pin_raint, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(terminate_button, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Initialize the detection model
net = cv2.dnn_DetectionModel(weightsPath, configPath)
net.setInputSize(320, 320)
net.setInputScale(1.0 / 127.5)
net.setInputMean((127.5, 127.5, 127.5))
net.setInputSwapRB(True)

def get_dist(box, image):
    w = box[2] 
    if w > 0: 
        dist = (width * focal) / w  
        print(f"Bounding box width: {w}, Calculated Distance: {dist:.2f} cm")  # Debug: Print the distance
        image = cv2.putText(image, f'Distance: {dist:.2f} cm', (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    else:
        print("No valid bounding box for distance calculation")
    return image

def getObjects(img, draw=True):
    classIds, confs, bbox = net.detect(img, confThreshold=thres)
    
    # Ensure bbox, confs, and classIds are lists
    bbox = list(bbox)
    confs = list(np.array(confs).reshape(1, -1)[0])
    classIds = list(np.array(classIds).flatten())
    
    indices = cv2.dnn.NMSBoxes(bbox, confs, thres, nms_threshold)
    objectInfo = []
    
    if len(indices) > 0:
        for i in indices.flatten():  # Flatten the indices
            box = bbox[i]
            classId = classIds[i]
            confidence = confs[i]
            className = classNames[classId - 1]  # Adjust classId for zero-based index
            
            objectInfo.append([box, className])
            
            if draw:
                x, y, w, h = box
                cv2.rectangle(img, (x, y), (x + w, y + h), color=(0, 255, 0), thickness=2)
                cv2.putText(img, className.upper(), (x + 10, y + 30), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 2)
                cv2.putText(img, f'{round(confidence * 100, 2)}%', (x + 200, y + 30), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 2)

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

# Main function
if __name__ == "__main__":
    stream = VideoStream(0).start()
    frame_count = 0

    while True:
        img = stream.read()
        if img is None:
            continue

        frame_count += 1
        if frame_count % 3 == 0: 
            result, objectInfo = getObjects(img, True)
            print(objectInfo)
            if objectInfo:
                box, className = objectInfo[0]  # Get the first detected object
                img = get_dist(box, img)  # Calculate and display distance

        # Display the video feed
        cv2.imshow("Live Video", img)

        # Exit on 'q' key press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Cleanup
    stream.stop()
    cv2.destroyAllWindows()
