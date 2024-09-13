import threading
import board
import busio
import adafruit_ssd1306
from PIL import Image, ImageDraw, ImageFont
from gpiozero import DistanceSensor
import speech_recognition as sr
import RPi.GPIO as GPIO
import time
from gpiozero.pins.pigpio import PiGPIOFactory
import subprocess
import queue

current_function = None
stop_flag = False

def run_file():
    subprocess.call(["/run_myfile.sh"])


button_pin_raigen = 24  # GPIO pin for RAIGEN button  
button_pin_raint = 23   # GPIO pin for RAINT button
RAIGEN = 'raigen'
RAINT = 'raint'

trig_pin = 6
echo_pin = 5
factory = PiGPIOFactory()

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(button_pin_raigen, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(button_pin_raint, GPIO.IN, pull_up_down=GPIO.PUD_UP)

r = sr.Recognizer()
mic = sr.Microphone()

i2c = busio.I2C(board.SCL, board.SDA)
oled = adafruit_ssd1306.SSD1306_I2C(128, 64, i2c, addr=0x3c)

# for text_display scroll
OLED_WIDTH = 128
OLED_HEIGHT = 64
LINE_HEIGHT = 12
y_offset = 0

oled.fill(0)
oled.show()

font_path = '/home/qwerty/sarg/font/Roboto/Roboto-Black.ttf'
font_size = 16
font = ImageFont.truetype(font_path, font_size)

image = Image.new('1', (oled.width, oled.height))
draw = ImageDraw.Draw(image)

def debounce_button(button_pin):
    last_state = GPIO.input(button_pin)
    time.sleep(0.01)
    current_state = GPIO.input(button_pin)
    if last_state == GPIO.HIGH and current_state == GPIO.LOW:
        return True
    return False

def clear_display():
    oled.fill(0)
    oled.show()

# Text wrap in OLED
# def add_line_breaks_input(text, max_length=16):
#     return '\n'.join([text[i:i+max_length] for i in range(0, len(text), max_length)])


def add_line_breaks_input(text, max_width):
    words = text.split()
    lines = []
    current_line = []
    
    for word in words:
        test_line = current_line + [word]
        bbox = draw.textbbox((0, 0), ' '.join(test_line), font=font)
        if bbox[2] - bbox[0] <= max_width:
            current_line.append(word)
        else:
            lines.append(' '.join(current_line))
            current_line = [word]
    
    if current_line:
        lines.append(' '.join(current_line))
    
    return lines

def add_line_breaks(text, max_width, font, draw):
    lines = []
    words = text.split(' ')
    current_line = ""
    
    for word in words:
        test_line = f"{current_line} {word}".strip()
        bbox = draw.textbbox((0, 0), test_line, font=font)
        text_width = bbox[2] - bbox[0]
        
        if text_width <= max_width:
            current_line = test_line
        else:
            lines.append(current_line)
            current_line = word
            
    if current_line:
        lines.append(current_line)
    
    return "\n".join(lines)
    
def display_text_with_animation(text, delay_between_chars=0.01):
    image = Image.new('1', (oled.width, oled.height))
    draw = ImageDraw.Draw(image)
    
    draw.rectangle((0, 0, oled.width, oled.height), outline=0, fill=0)
    
    wrapped_text = add_line_breaks(text, oled.width, font, draw)
    
    y = 0
    for line in wrapped_text.split('\n'):
        x = 0
        for char in line:
            bbox = draw.textbbox((x, y), char, font=font)
            text_width = bbox[2] - bbox[0]
            
            draw.text((x, y), char, font=font, fill=255)
            oled.image(image)
            oled.show()
            
            x += text_width
            
            time.sleep(delay_between_chars)
        
        line_height = font.getbbox('A')[3]  
        y += line_height  

    time.sleep(1)

accumulated_words = []
words_lock = threading.Lock()

def display_words():
    global y_offset
    last_words = ""

    while True:
        with words_lock:
            if accumulated_words:
                words_to_display = ' '.join(accumulated_words)
                
                # Only update if the text has changed
                if words_to_display != last_words:
                    last_words = words_to_display

                    draw.rectangle((0, 0, OLED_WIDTH, OLED_HEIGHT), outline=0, fill=0)

                    wrapped_lines = add_line_breaks_input(words_to_display, OLED_WIDTH)
                    total_text_height = len(wrapped_lines) * LINE_HEIGHT

                    if total_text_height > OLED_HEIGHT:
                        y_offset = max(OLED_HEIGHT - total_text_height, -abs(y_offset))
                    else:
                        y_offset = 0

                    for i, line in enumerate(wrapped_lines):
                        draw.text((0, i * LINE_HEIGHT + y_offset), line, font=font, fill=255)

                    oled.image(image)
                    oled.show()

        time.sleep(0.1)  # Reduced sleep time for more responsive updates
        

# SPEECH TO TEXT -----------------------------------
def RAINT():
    global stop_flag
    stop_flag = False 
    if GPIO.input(button_pin_raint) == GPIO.LOW:
        if current_function != RAINT:
            return True

    display_thread = threading.Thread(target=display_words)
    display_thread.daemon = True
    display_thread.start()

    r = sr.Recognizer()
    r.energy_threshold = 300  # Adjust based on your environment
    r.dynamic_energy_threshold = False
    r.pause_threshold = 0.8  # Shorter pause to detect end of speech
    
    audio_queue = queue.Queue()
    text_queue = queue.Queue()

    def audio_producer():
        with sr.Microphone() as source:
            r.adjust_for_ambient_noise(source, duration=0.5)
            print("Adjusted for ambient noise. Now listening...")
            while not stop_flag:  # Check the stop flag before each loop
                try:
                    audio = r.listen(source, phrase_time_limit=3)
                    audio_queue.put(audio)
                except sr.WaitTimeoutError:
                    pass

    def audio_consumer():
        while not stop_flag:  # Check the stop flag in the consumer loop as well
            if GPIO.input(button_pin_raigen) == GPIO.LOW or GPIO.input(button_pin_raint) == GPIO.LOW:
                break
            try:
                audio = audio_queue.get(timeout=1)
                try:
                    text = r.recognize_google(audio, language='en-IN')
                    print(f"Recognized: {text}")
                    text_queue.put(text)
                except sr.UnknownValueError:
                    print("Google Speech Recognition could not understand audio")
                except sr.RequestError as e:
                    print(f"Could not request results from Google Speech Recognition service; {e}")
            except queue.Empty:
                pass

    producer_thread = threading.Thread(target=audio_producer)
    consumer_thread = threading.Thread(target=audio_consumer)
    
    producer_thread.start()
    consumer_thread.start()

    try:
        while True:
            if GPIO.input(button_pin_raigen) == GPIO.LOW or GPIO.input(button_pin_raint) == GPIO.LOW:
                break
            try:
                text = text_queue.get(timeout=0.1)
                with words_lock:
                    accumulated_words.append(text)
            except queue.Empty:
                pass
    finally:
        producer_thread.join(timeout=1)
        consumer_thread.join(timeout=1)


    

# OBJECT DETECTION -----------------------------------

sensor = DistanceSensor(echo=echo_pin, trigger=trig_pin, pin_factory=factory)

def RAIGEN():
    global stop_flag
    stop_flag = False  # Reset the stop flag at the beginning of the function
    while not stop_flag:  # Check the stop flag in the loop
      
        if GPIO.input(button_pin_raint) == GPIO.LOW:
            print("Switching to RAINT...")
            break  

        print(f"Distance: {sensor.distance * 100:.1f} cm")
        time.sleep(1)  

# MAIN -----------------------------------
def main_loop():
    global current_function, stop_flag  

    while True:
        if GPIO.input(button_pin_raigen) == GPIO.LOW:
            if current_function != RAIGEN:
                stop_flag = True
                time.sleep(0.5)  # Wait for the other function to stop
                current_function = 'RAIGEN'
                clear_display()
                print("Switched to RAIGEN")
                RAIGEN()
            
        elif GPIO.input(button_pin_raint) == GPIO.LOW: 
            if current_function != RAINT:
                stop_flag = True
                time.sleep(0.5)  # Wait for the other function to stop
                current_function = 'RAINT'
                welcome_words_raint = "RAINT (Real-time Audio Interpretation and Notation Transcription)"
                display_text_with_animation(welcome_words_raint)
                clear_display()
                RAINT()


        time.sleep(0.1)
            
if __name__ == "__main__":
    try:
        main_loop()
    except KeyboardInterrupt:
        clear_display()
        GPIO.cleanup()
        print("Program terminated")