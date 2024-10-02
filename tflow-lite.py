import cv2
import tflite_runtime.interpreter as tflite
import numpy as np
import threading
import RPi.GPIO as GPIO
import time

# GPIO pin setup
button_pin_raigen = 23
button_pin_raint = 22
terminate_button = 16
thres = 0.5  # Detection threshold
nms_threshold = 0.4 # Non-Maximum Suppression threshold
BUZZER = 26
focal = 450  # Focal length (calibration needed)
width = 4  # Width of object for distance calculation

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(BUZZER, GPIO.OUT)
GPIO.setup(button_pin_raigen, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(button_pin_raint, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(terminate_button, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Load TFLite model and allocate tensors
interpreter = tflite.Interpreter(model_path="detect.tflite")
interpreter.allocate_tensors()

# Get input and output details from the model
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

input_shape = input_details[0]['shape'][1:3]  # Model input shape (height, width)

# Load label map
with open("labelmap.txt", 'r') as f:
    labels = [line.strip() for line in f.readlines()]

# Initialize global variables for video frame capture
frame = None
ret = False

# Threaded function to continuously capture frames from the video stream
def capture_frame(cap):
    global frame, ret
    while True:
        ret, frame = cap.read()
        if not ret:
            break

# Function to terminate the program gracefully
def terminate_program():
    print("Shutdown button pressed, terminating the program...")
    cap.release()
    cv2.destroyAllWindows()
    GPIO.cleanup()
    exit(0)

# Callback function to switch to RAINT mode
def callback_raint():
    from main import RAINT
    if GPIO.input(button_pin_raint) == GPIO.LOW:
        print("Switching to RAINT...")
        cap.release()
        cv2.destroyAllWindows()
        RAINT()
        GPIO.cleanup()
    elif GPIO.input(terminate_button) == GPIO.LOW:
        terminate_program()

previous_boxes = [] 
previous_n = 3 

def smooth_boxes(box, previous_boxes, n=3):
    previous_boxes.append(box)
    if len(previous_boxes) > n:
        previous_boxes.pop(0)

    # Calculate the average bounding box coordinates
    avg_box = np.mean(previous_boxes, axis=0).astype(int)
    return avg_box


cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FPS, 30)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
cap.set(cv2.CAP_PROP_BRIGHTNESS, 0.5)  # Adjust brightness
cap.set(cv2.CAP_PROP_CONTRAST, 0.5)    # Adjust contrast
cap.set(cv2.CAP_PROP_EXPOSURE, 0.1)    # Adjust exposure (if supported)

capture_thread = threading.Thread(target=capture_frame, args=(cap,))
capture_thread.start()

while True:
    if ret:
        # Preprocess the frame for object detection
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        resized_frame = cv2.resize(frame_rgb, (input_shape[1], input_shape[0]))  # Resize frame for model input
        input_data = np.expand_dims(np.array(resized_frame, dtype=np.uint8), axis=0)

        # Set the input tensor for the model
        interpreter.set_tensor(input_details[0]['index'], input_data)
        interpreter.invoke()

        # Get model outputs: bounding boxes, class IDs, and confidence scores
        boxes = interpreter.get_tensor(output_details[0]['index'])[0]
        classes = interpreter.get_tensor(output_details[1]['index'])[0]
        scores = interpreter.get_tensor(output_details[2]['index'])[0]
        num_detections = int(interpreter.get_tensor(output_details[3]['index'])[0])

        bbox = []
        confs = []
        classIds = []

        for i in range(num_detections):
            if scores[i] > thres:  # Only consider detections above the threshold
                ymin, xmin, ymax, xmax = boxes[i]
                x = int(xmin * frame.shape[1])
                y = int(ymin * frame.shape[0])
                w = int((xmax - xmin) * frame.shape[1])
                h = int((ymax - ymin) * frame.shape[0])

                bbox.append([x, y, w, h])  # Append the bounding box
                confs.append(float(scores[i]))  # Append the confidence score
                classIds.append(int(classes[i]))  # Append the class ID

        indices = cv2.dnn.NMSBoxes(bbox, confs, thres, nms_threshold)

        class_to_box = {}
        num_objects_detected = len(indices)
        print(f"Objects detected: {num_objects_detected}")
        if len(indices) > 0:
            for i in indices.flatten():
                box = bbox[i]
                classId = classIds[i]
                confidence = confs[i]

                if classId not in class_to_box or confidence > class_to_box[classId][1]:
                    class_to_box[classId] = (box, confidence) 

        for classId, (box, confidence) in class_to_box.items():
            smoothed_box = smooth_boxes(box, previous_boxes, previous_n)
            x, y, w, h = smoothed_box

            if w > 0:
                dist = (width * focal) / w

                if dist < 4:
                    GPIO.output(BUZZER, GPIO.HIGH)
                    time.sleep(0.1)
                    GPIO.output(BUZZER, GPIO.LOW)
                    time.sleep(0.1)
                else:
                    GPIO.output(BUZZER, False)

                # Draw the bounding box and labels
                cv2.rectangle(frame, (x, y), (x + w, y + h), color=(0, 255, 0), thickness=2)
                label = f"{labels[classId]}: {confidence:.2f}"
                cv2.putText(frame, label, (x, y + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.putText(frame, f'{round(confidence * 100)}%', (x, y + 45), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0), 2)
                cv2.putText(frame, f'Distance: {dist * 100:.2f} cm', (x, y + 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                print(f"{label} - {dist * 100:.2f}cm")

        # Display the resulting frame
        cv2.imshow('Object Detection', frame)

    # Check for user input to break the loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    if GPIO.input(button_pin_raint) == GPIO.LOW:
        callback_raint()
        break
    elif GPIO.input(terminate_button) == GPIO.LOW:
        terminate_program()


# Clean up resources
cap.release()
cv2.destroyAllWindows()
GPIO.cleanup()
