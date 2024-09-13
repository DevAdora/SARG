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

current_function = None

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

def add_line_breaks_input(text, max_width, font, draw):
    words = text.split(' ')
    lines = []
    current_line = ""

    for word in words:
        # Check if adding this word will exceed the max width
        test_line = f"{current_line} {word}".strip()
        bbox = draw.textbbox((0, 0), test_line, font=font)
        text_width = bbox[2] - bbox[0]
        
        if text_width <= max_width:
            current_line = test_line  # Word fits in the line
        else:
            lines.append(current_line)  # Add current line and start new one
            current_line = word

    if current_line:
        lines.append(current_line)  # Append remaining words

    return lines  # Return a list of lines

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

# GUBA PAMANI PIP DOLLL 

accumulated_words = []
words_lock = threading.Lock()

def display_words():
    global y_offset

    """ Continuously update the OLED display with recognized words """
    while True:
        # Acquire lock to safely access shared word array
        with words_lock:
            if accumulated_words:
                words_to_display = ' '.join(accumulated_words)

                image = Image.new('1', (oled.width, oled.height))
                draw = ImageDraw.Draw(image)

                draw.rectangle((0, 0, oled.width, oled.height), outline=0, fill=0)

                wrapped_lines = add_line_breaks_input(words_to_display, OLED_WIDTH, font, draw)

                total_text_height = len(wrapped_lines) * LINE_HEIGHT

                if total_text_height > OLED_HEIGHT:
                    y_offset -= LINE_HEIGHT

                if abs(y_offset) > total_text_height - OLED_HEIGHT:
                    y_offset = 0

                for i, line in enumerate(wrapped_lines):
                    draw.text((0, i * LINE_HEIGHT + y_offset), line, font=font, fill=255)

                # Clear previous image content
                oled.image(image)
                oled.show()

        time.sleep(0.5)  # Adjust to control display refresh rate
        

# SPEECH TO TEXT -----------------------------------
def RAINT():
    display_thread = threading.Thread(target=display_words)
    display_thread.daemon = True  # Daemon thread will exit when the main program exits
    display_thread.start()

    last_speech_time = time.time()
    speech_timeout = 3  # Timeout in seconds after no speech is detected

    while True:
        if GPIO.input(button_pin_raigen) == GPIO.LOW or GPIO.input(button_pin_raint) == GPIO.LOW:
            break

        with mic as source:          
            r.adjust_for_ambient_noise(source)
            print("Listening .. ")
            try:
                audio = r.listen(source, timeout=5)
                last_speech_time = time.time() 
                words = r.recognize_google(audio, language='en-IN')
                print(words)

                with words_lock:
                    accumulated_words.append(words)

                # words = add_line_breaks_input(words)

                # # Clear previous image content
                # draw.rectangle((0, 0, oled.width, oled.height), outline=0, fill=0)
                
                # # Draw the recognized words on the image
                
                # # stream text instead of 1 overall display
                # draw.text((0, 0), words, font=font, fill=255)

                # # Display the image on the OLED
                # oled.image(image) 
                # oled.show()
                
            except sr.UnknownValueError:
                print("Google Speech Recognition could not understand audio")

        if time.time() - last_speech_time > speech_timeout:
            print("No input detected for a while, processing...")
            last_speech_time = time.time()

        time.sleep(0.1)  


    

# OBJECT DETECTION -----------------------------------

sensor = DistanceSensor(echo=echo_pin, trigger=trig_pin, pin_factory=factory)

def check_distance():
    print(f"Distance: {sensor.distance * 100:.1f} cm")
    time.sleep(1)

def RAIGEN():
  while True:
    if GPIO.input(button_pin_raigen) == GPIO.LOW or GPIO.input(button_pin_raint) == GPIO.LOW:
        break  
    check_distance()
    time.sleep(0.1)

# MAIN -----------------------------------
def main_loop():
    global current_function        

    while True:
        if GPIO.input(button_pin_raigen) == GPIO.LOW:
            if current_function != RAIGEN:
                current_function = RAIGEN
                clear_display()
                print("Switched to RAIGEN")

            current_function()
            
        elif GPIO.input(button_pin_raint) == GPIO.LOW: 
            if current_function != RAINT:
                current_function = RAINT
                welcome_words_raint = "RAINT (Real-time Audio Interpretation and Notation Transcription)"
                display_text_with_animation(welcome_words_raint)
                clear_display()
                
            current_function()


        time.sleep(0.1)
            
if __name__ == "__main__":
    try:
        main_loop()
    except KeyboardInterrupt:
        GPIO.cleanup()
        print("Program terminated")
        