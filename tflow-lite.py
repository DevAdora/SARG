import cv2
import numpy as np
import tflite_runtime.interpreter as tflite
import threading
import RPi.GPIO as GPIO
import time


button_pin_raigen = 23
button_pin_raint = 22
terminate_button = 16
thres = 0.5  
nms_threshold = 0.5 
BUZZER = 26
focal = 450  
width = 4  

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(BUZZER, GPIO.OUT)
GPIO.setup(button_pin_raigen, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(button_pin_raint, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(terminate_button, GPIO.IN, pull_up_down=GPIO.PUD_UP)

interpreter = tflite.Interpreter(model_path="detect.tflite")
interpreter.allocate_tensors()

input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

with open("labelmap.txt", 'r') as f:
    labels = [line.strip() for line in f.readlines()]

input_size = (300, 300) 

threshold = 0.5 
nms_threshold = 0.3

frame = None
ret = False

def capture_frame(cap):
    global frame, ret
    while True:
        ret, frame = cap.read()
        if not ret:
            break

cap = cv2.VideoCapture(0)

capture_thread = threading.Thread(target=capture_frame, args=(cap,))
capture_thread.daemon = True 
capture_thread.start()

while True:
    if ret: 
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        resized_frame = cv2.resize(frame_rgb, input_size)
        input_data = np.expand_dims(np.array(resized_frame, dtype=np.uint8), axis=0)

        interpreter.set_tensor(input_details[0]['index'], input_data)

        interpreter.invoke()

        boxes = interpreter.get_tensor(output_details[0]['index'])[0]  # Bounding box coordinates
        classes = interpreter.get_tensor(output_details[1]['index'])[0]  # Class index of detected objects
        scores = interpreter.get_tensor(output_details[2]['index'])[0]  # Confidence of detected objects
        num_detections = int(interpreter.get_tensor(output_details[3]['index'])[0])  # Number of detections

        bbox = []
        confs = []
        classIds = []

        for i in range(num_detections):
            if scores[i] > threshold:  
                ymin, xmin, ymax, xmax = boxes[i]
                x = int(xmin * frame.shape[1])
                y = int(ymin * frame.shape[0])
                w = int((xmax - xmin) * frame.shape[1])
                h = int((ymax - ymin) * frame.shape[0])

                bbox.append([x, y, w, h])  
                confs.append(float(scores[i])) 
                classIds.append(int(classes[i])) 

        indices = cv2.dnn.NMSBoxes(bbox, confs, threshold, nms_threshold)
        num_objects_detected = len(indices)
        print(f"Objects detected: {num_objects_detected}")
        if len(indices) > 0:
            for i in indices.flatten():
                x, y, w, h = bbox[i]
                classId = classIds[i]
                confidence = confs[i]

                label = labels[classId]

                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

                font_scale = max(0.5, min(1.5, w / 300.0))
                if w > 0:
                    dist = (width * focal) / w

                    if dist < 4:
                        GPIO.output(BUZZER, GPIO.HIGH)
                        time.sleep(0.1)
                        GPIO.output(BUZZER, GPIO.LOW)
                        time.sleep(0.1)
                    else:
                        GPIO.output(BUZZER, False)

                label_text = f"{label}: {confidence:.2f}"
                cv2.putText(frame, label_text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 0, 0), 2)
                print(f"{label} - {dist * 100:.2f}cm")

        cv2.imshow('Object Detection', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
