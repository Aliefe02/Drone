# Version of the path follow program that has object avoidence
# Not stable

import time
time.sleep(2)

import sys
import os
import glob

log_directory = '/home/ali/Desktop/path_follow/'

def generate_log_file_name(base_directory, prefix):
    files = glob.glob(os.path.join(base_directory, f'{prefix}*.log'))

    new_file_name = f'{prefix}{len(files) + 1}.log'
    return os.path.join(base_directory, new_file_name)

stdout_file_name = generate_log_file_name(log_directory, 'stdout')
stderr_file_name = generate_log_file_name(log_directory, 'stderr')

sys.stdout = open(stdout_file_name, 'w')
sys.stderr = open(stderr_file_name, 'w')
import json
from dronekit import connect,VehicleMode
import math
import Adafruit_SSD1306
from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont
from SX127x.LoRa import *
from SX127x.board_config import BOARD
from pymavlink import mavutil
import RPi.GPIO as GPIO
import threading
# Set Buzzer
buzzer_pin = 21
GPIO.setmode(GPIO.BCM)
GPIO.setup(buzzer_pin, GPIO.OUT)


targetHeight = 2
def remove_null_bytes(data):
    return data.replace('\x00', '')
max_avoid_meters = 5
targetAltitude = 1
manualArm = False

# LoRa setup

BOARD.setup()
RST = None
DC = 23
SPI_PORT = 0
SPI_DEVICE = 0
disp = Adafruit_SSD1306.SSD1306_128_32(rst=RST)

disp.begin()
disp.clear()
disp.display()

width = disp.width
height = disp.height
image = Image.new('1', (width, height))
draw = ImageDraw.Draw(image)
draw.rectangle((0, 0, width, height), outline=0, fill=0)
font = ImageFont.load_default()

# Ultrasonic Setup
#sensor 1 -> sağ
#sensor 2 -> sol
#sensor 3 -> ön

distance1 = 0
distance2 = 0
distance3 = 0

sensor1_trig = 6
sensor1_echo = 13
sensor2_trig = 0
sensor2_echo = 5
sensor3_trig = 26
sensor3_echo = 19

GPIO.setup(sensor1_trig, GPIO.OUT)
GPIO.setup(sensor1_echo, GPIO.IN)
GPIO.setup(sensor2_trig, GPIO.OUT)
GPIO.setup(sensor2_echo, GPIO.IN)
GPIO.setup(sensor3_trig, GPIO.OUT)
GPIO.setup(sensor3_echo, GPIO.IN)

def buz_error():     # Buzz 3 times and exit the program with error
    for n in range(3):
        for i in range(100):
            GPIO.output(buzzer_pin, GPIO.HIGH)  
            time.sleep(0.001)  
            GPIO.output(buzzer_pin, GPIO.LOW)   
            time.sleep(0.003) 

        GPIO.output(buzzer_pin, GPIO.LOW) 
        time.sleep(1)
    GPIO.cleanup()
    exit(-1)

def get_distance(trig, echo):
    GPIO.output(trig, True)
    time.sleep(0.00001)
    GPIO.output(trig, False)

    start_time = time.time()
    while GPIO.input(echo) == 0:
        start_time = time.time()

    stop_time = time.time()
    while GPIO.input(echo) == 1:
        stop_time = time.time()
        if stop_time - start_time >= 0.04:  
            print("Out of range")
            return 255  # target too far to calculate so send a value larger than the lowest distance value

    # Calculate distance
    elapsed_time = stop_time - start_time
    distance = (elapsed_time * 34300) / 2 

    return distance

def update_distance():
    global distance1
    global distance2
    global distance3
    # Running in a thread, this function updates distances on the global variables
    try:
        while True:
            distance1 = get_distance(sensor1_trig, sensor1_echo)
            distance2 = get_distance(sensor2_trig, sensor2_echo)
            distance3 = get_distance(sensor3_trig, sensor3_echo)
    except:
        buz_error()


def buz():
    for i in range(100):
        GPIO.output(buzzer_pin, GPIO.HIGH)  
        time.sleep(0.001)  
        GPIO.output(buzzer_pin, GPIO.LOW)  
        time.sleep(0.003)  
    GPIO.output(buzzer_pin, GPIO.LOW)  


def buz_time(x):
    for n in range(int(x)):
        for i in range(100):
            GPIO.output(buzzer_pin, GPIO.HIGH) 
            time.sleep(0.001)  
            GPIO.output(buzzer_pin, GPIO.LOW)  
            time.sleep(0.003)  

        GPIO.output(buzzer_pin, GPIO.LOW) 
        time.sleep(1)  

def draw_on_display(data):
    try:
        global data_write
        global test
        draw.rectangle((0, 0, width, height), outline=0, fill=0)
        draw.text((5, 10), "> "+data, font=font, fill=255)
        rotated_image = image.rotate(180)  # Rotate the image by 180 degrees
        disp.image(rotated_image)
        disp.display()
    except:
        buz_error()

def draw_movement(x,y,z,yaw):
    try:
        global data_write
        global test
        draw.rectangle((0, 0, width, height), outline=0, fill=0)
        draw.text((5, 10), " "+str(x)+" "+str(y)+" "+str(z)+" "+str(yaw), font=font, fill=255)
        rotated_image = image.rotate(180)  # Rotate the image by 180 degrees
        disp.image(rotated_image)
        disp.display()
    except:
        buz_error()

def connectMyCopter():
    print("Starting the connection")
    connection_string = "/dev/ttyACM0"
    baud_rate = 57600

    vehicle = connect(connection_string,baud=baud_rate,wait_ready=True)
    print("Conected")
    return vehicle

def move(x, y, z,yaw):
    try:
        draw_movement(x,y,z,yaw)
        msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            0b0000011111000111,
            0, 0, 0,
            x, y, z,
            0, 0, 0,
            0, math.radians(yaw))

        vehicle.send_mavlink(msg)
        time.sleep(1)
    except:
        buz_error()

def rotate(rotation,direction):
    try:
        print(str(rotation),str(direction))
        i = 0
        while i < int(rotation/30):
            print("Turn")
            msg = vehicle.message_factory.set_position_target_local_ned_encode(
                0,
                0, 0,
                mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                0b0000011111000111,
                0, 0, 0,
                0, 0, 0,
                0, 0, 0,
                0, math.radians(int(30*(direction))))

            vehicle.send_mavlink(msg)
            time.sleep(1)
            i += 1
    except:
        buz_error()

def take_off(height):
    try:
        print('Arming Vehicle')

        vehicle.mode = VehicleMode("GUIDED")

        while vehicle.mode != 'GUIDED':
            print("Waiting for drone to enter GUIDED mode")
            time.sleep(.5)
        print("Vehicle in GUIDED mode")

        if manualArm == False:
            vehicle.armed = True
            while vehicle.armed == False:
                print("Waiting for vehicle to become armed")
                time.sleep(1)
        else:
            if vehicle.armed == False:
                print("Exiting script. Failed when arming")
                return None
        print("Vehicle is ARMED")
        vehicle.simple_takeoff(int(height))
        draw_on_display("Flying")
        while True:
            print("Current Altitude: "+str(vehicle.location.global_relative_frame.alt))
            if vehicle.location.global_relative_frame.alt >= .95*int(height):
                break
            time.sleep(0.5)
        print("Target altitude reached!")
        return None
    except:
        buz_error()


def point_forward(degree):
    rotation = 0
    direction = 1
    if degree != 0:
        if degree == 90:
            rotation = 90
            direction = -1
        elif degree == 270:
            rotation = 90
        elif degree == 180:
            rotation = 180
        rotate(rotation,direction)
    return 0

def point_right(degree):
    rotation = 0
    direction = 1
    if degree != 90:
        if degree == 0:
            rotation = 90
        elif degree == 180:
            rotation = 90
            direction = -1
        elif degree == 270:
            rotation = 180
        rotate(rotation,direction)
    return 90

def point_backwards(degree):
    rotation = 0
    direction = 1
    if degree != 180:
        if degree == 0:
            rotation = 180
        elif degree == 90:
            rotation = 90
        elif degree == 270:
            rotation = 90
            direction = -1
        rotate(rotation,direction)
    return 180

def point_left(degree):
    rotation = 0
    direction = 1
    if degree != 270:
        if degree == 0:
            rotation = 90
            direction = -1
        elif degree == 90:
            rotation = 180
        elif degree == 180:
            rotation = 90
        rotate(rotation,direction)
    return 270


def go_forward():
    move(1,0,0,0)

def stop():
    move(0,0,0,0)

def land():
    buz()
    vehicle.mode = VehicleMode("LAND")
    while vehicle.mode!='LAND':
        time.sleep(1)
        draw_on_display("Landing")
        print("Waiting for drone to land")
    draw_on_display("Drone Landed")

def land_exit(): # Land the drone and exit the program
    land()
    exit()

def follow_path(path,height,cell_length):
    global distance1
    global distance2
    global distance3
    
    try:
        draw_on_display("Drone Starting!!!")                #  x  +  > forward
        take_off(height)                                    #  y  +  > go right
                                # 0 -> x                       z  +  > go down
                                # 1 -> y                      yaw 60 > turn right 60 degrees, 60 degrees in 1 second
        degree = 0
        draw_on_display("Starting Route")
        print(path)
        skip_count = 0
        coordinate_count = 0
        # for i in range(len(path)):
        while coordinate_count < len(path):
            right_shift = 0
            left_shift = 0
            forward_shift = 0
            prev_cord = path[coordinate_count]
            cord = path[coordinate_count+1]
            if skip_count > 0:
                skip_count -= 1
                continue
            if cord[0] == prev_cord[0] and cord[1] < prev_cord[1]:          # Go forward
                print("Forward")
                degree = point_forward(degree)
                if distance3 < 150:     # Obstacle ahead
                    stop()
                    if distance1 > 150:     # Turn right
                        degree = point_right(degree)
                        stop()
                        time.sleep(0.1)
                        while distance2 <= 150:
                            if right_shift >= max_avoid_meters or distance3 <= 150:
                                land_exit()
                            go_forward()
                            right_shift += 1
                        degree = point_forward(degree)
                        while distance2 <= 150:
                            if right_shift >= max_avoid_meters or distance3 <= 150:
                                land_exit()
                            go_forward()
                            forward_shift += 1
                            time.sleep(1)
                            stop()      #go 1 m, stop and calculate distance, go again
                            time.sleep(0.1)
                        degree = point_left(degree)
                        for i in range(right_shift):
                            if distance3 <= 150:
                                land_exit()
                            go_forward()
                        degree = point_forward(degree)
                        coordinate_count += forward_shift
                    elif distance2 > 150:   # Turn left
                        degree = point_left(degree)
                        while distance1 <= 150:
                            if left_shift >= max_avoid_meters or distance3 <= 150:
                                land_exit()
                            go_forward()
                            left_shift += 1
                            time.sleep(1)
                            stop()      #go 1 m, stop and calculate distance, go again
                            time.sleep(0.1)
                        degree = point_forward(degree)
                        while distance1 <= 150:
                            if left_shift >= max_avoid_meters or distance3 <= 150:
                                land_exit()
                            go_forward()
                            forward_shift += 1
                            time.sleep(1)
                            stop()      #go 1 m, stop and calculate distance, go again
                            time.sleep(0.1)
                        degree = point_left(degree)
                        for i in range(left_shift):
                            if distance3 <= 150:
                                land_exit()
                            go_forward()
                        degree = point_forward(degree)
                        coordinate_count += forward_shift
                    else:
                        # Land
                        land()
                        break
                else:    # No obstacle ahead
                    go_forward()
            # Done
            elif cord[0] == prev_cord[0] and cord[1] > prev_cord[1]:        # Go backwards
                print("Backwards")
                degree = point_backwards(degree)
                go_forward()
            elif cord[0] < prev_cord[0] and cord[1] == prev_cord[1]:        # Turn left, go 1 forward
                print("Left")
                degree = point_left(degree)
                go_forward()
            elif cord[0] > prev_cord[0] and cord[1] == prev_cord[1]:        # Turn right, go 1 forward
                print("Right")
                degree = point_right(degree)
                go_forward()
            elif cord[0] > prev_cord[0] and cord[1] < prev_cord[1]:         # Go forward 1, right 1
                print("Right Forward")
                degree = point_forward(degree)
                go_forward()
                degree = point_right(degree)
                go_forward()
            elif cord[0] < prev_cord[0] and cord[1] < prev_cord[1]:         # Go forward 1, left 1
                print("Left Forward")
                degree = point_forward(degree)
                go_forward()
                degree = point_left(degree)
                go_forward()
            elif cord[0] > prev_cord[0] and cord[1] > prev_cord[1]:         # Go backward 1, right 1
                print("Right Backwards")
                degree = point_backwards(degree)
                go_forward()
                degree = point_right(degree)
                go_forward()
            elif cord[0] < prev_cord[0] and cord[1] > prev_cord[1]:         # Go backward 1, left 1
                print("Left Backwards")
                degree = point_backwards(degree)
                go_forward()
                degree = point_left(degree)
                go_forward()
            coordinate_count += 1
        land()
    except:
        buz_error()

class LoRaRcvCont(LoRa):
    def __init__(self, verbose=False):
        super(LoRaRcvCont, self).__init__(verbose)
        self.set_mode(MODE.SLEEP)
        self.set_dio_mapping([0, 0, 0, 0, 0, 0])

    def start(self):
        self.reset_ptr_rx()
        self.set_mode(MODE.RXCONT)

        while True:
            time.sleep(.5)
            status = self.get_modem_status()

    def on_rx_done(self):
        try:
            self.clear_irq_flags(RxDone=1)
            self.set_modem_config_1()
            payload = self.read_payload(nocheck=True)
            data =remove_null_bytes(bytearray(payload).decode('utf-8', 'ignore').strip())

            if 'correct' in data: # Correct data is received and path is ready
                data = json.loads(data)
                draw_on_display("Setting Up!")
                distance_measure_thread = threading.Thread(target=update_distance,args=())  # Thread for reading distances without blockingthe main part
                distance_measure_thread.start()
                follow_path(data['path'],data['height'],data["cell_length"])    # Start route

            self.set_mode(MODE.SLEEP)
            self.reset_ptr_rx()
            self.set_mode(MODE.RXCONT)
        except:
            buz_error()



draw_on_display("Connecting..")
try:
    vehicle = connectMyCopter()
except:
    draw_on_display("ERROR!")
    buz_time(3)
    GPIO.cleanup()
    exit(0)

try:
    lora = LoRaRcvCont(verbose=False)
    lora.set_mode(MODE.STDBY)
    lora.set_pa_config(pa_select=1)

    draw_on_display("READY!")
    lora.start()
    buz()
except KeyboardInterrupt:
    print("\nKeyboardInterrupt")
except:
    draw_on_display("ERROR LoRa!")
    buz_time(3)
    GPIO.cleanup()
    exit(-1)
finally:
    lora.set_mode(MODE.SLEEP)
    BOARD.teardown()
