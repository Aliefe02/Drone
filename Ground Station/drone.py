
import time
time.sleep(2)

import sys
import os
import glob

# Define the directory where the log files are stored
log_directory = '/home/ali/Desktop/path_follow/'

# Function to count files and generate new file name
def generate_log_file_name(base_directory, prefix):
    # List all files matching the pattern (e.g., stdout*.log or stderr*.log)
    files = glob.glob(os.path.join(base_directory, f'{prefix}*.log'))
    
    # Generate the new file name based on the number of existing files
    new_file_name = f'{prefix}{len(files) + 1}.log'
    return os.path.join(base_directory, new_file_name)

# Generate new file names for stdout and stderr
stdout_file_name = generate_log_file_name(log_directory, 'stdout')
stderr_file_name = generate_log_file_name(log_directory, 'stderr')

# Redirect stdout and stderr to the new files
sys.stdout = open(stdout_file_name, 'w')
sys.stderr = open(stderr_file_name, 'w')
import json
from dronekit import connect,VehicleMode #,LocationGlobalRelative,APIException
#import socket
import math
#import argparse
#import Adafruit_GPIO.SPI as SPI
import Adafruit_SSD1306
from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont
from SX127x.LoRa import *
from SX127x.board_config import BOARD
from pymavlink import mavutil

targetHeight = 2
def remove_null_bytes(data):
    return data.replace('\x00', '')

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


def draw_on_display(data):
    global data_write
    global test
    draw.rectangle((0, 0, width, height), outline=0, fill=0)
    draw.text((5, 10), "> "+data, font=font, fill=255)
    rotated_image = image.rotate(180)  # Rotate the image by 180 degrees
    disp.image(rotated_image)
    disp.display()

def draw_movement(x,y,z,yaw):
    global data_write
    global test
    draw.rectangle((0, 0, width, height), outline=0, fill=0)
    draw.text((5, 10), " "+str(x)+" "+str(y)+" "+str(z)+" "+str(yaw), font=font, fill=255)
    rotated_image = image.rotate(180)  # Rotate the image by 180 degrees
    disp.image(rotated_image)
    disp.display()



def connectMyCopter():
    print("Starting the connection")
    connection_string = "/dev/ttyACM0"
    baud_rate = 57600

    vehicle = connect(connection_string,baud=baud_rate,wait_ready=True)
    print("Conected")
    return vehicle

def arm():
    print("Arming")
    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode != 'GUIDED':
        time.sleep(1)
        print("Waiting for drone to enter GUIDED mode")
    vehicle.armed = True
    while vehicle.armed== False:
        print("Waiting for drone to become armed..")
        vehicle.armed = True
        time.sleep(1)
    print("\n[Vehicle is now armed]")
    print("[Props are spinning]")
    return None

def loiter():
    print('Arming Vehicle')

    vehicle.mode = VehicleMode("GUIDED")

    while vehicle.mode != 'GUIDED':
        print("Waiting for drone to enter GUIDED mode")
        time.sleep(1)
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
    draw_on_display("Flying")
    vehicle.simple_takeoff(2)

    while True:
        print("Current Altitude: "+str(vehicle.location.global_relative_frame.alt))
        if vehicle.location.global_relative_frame.alt >= .95*targetHeight:
            break
        time.sleep(0.5)
    print("Target altitude reached!")
    vehicle.mode = VehicleMode("LOITER")
    return None


def hover():
    print('Arming Vehicle')

    vehicle.mode = VehicleMode("GUIDED")

    while vehicle.mode != 'GUIDED':
        print("Waiting for drone to enter GUIDED mode")
        time.sleep(1)
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
    draw_on_display("Flying")
    vehicle.simple_takeoff(2)

    while True:
        print("Current Altitude: "+str(vehicle.location.global_relative_frame.alt))
        if vehicle.location.global_relative_frame.alt >= .95*targetHeight:
            break
        time.sleep(0.5)
    print("Target altitude reached!")

    return None

def move(x, y, z,yaw):
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

def take_off(height):
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
        if vehicle.location.global_relative_frame.alt >= .95*2:
            break
        time.sleep(0.5)
    print("Target altitude reached!")

    return None

def point_forward(degree):
    rotation = 0
    if degree != 0:
        if degree == 90:
            rotation = -90
        elif degree == 270:
            rotation = 90
        elif degree == 180:
            rotation = 180
        move(0,0,0,rotation)
    return 0  
  
def point_right(degree):
    rotation = 0
    if degree != 90:
        if degree == 0:     
            rotation = 90
        elif degree == 180:
            rotation = -90
        elif degree == 270:
            rotation = 180
        
        move(0,0,0,rotation)
    return 90

def point_backwards(degree):
    rotation = 0
    if degree != 180:
        if degree == 0:
            rotation = 180
        elif degree == 90:
            rotation = 90
        elif degree == 270:
            rotation = -90
        move(0,0,0,rotation)
    return 180

def point_left(degree):
    rotation = 0
    if degree != 270:
        if degree == 0:
            rotation = -90
        elif degree == 90:
            rotation = 180
        elif degree == 180:
            rotation = 90
        move(0,0,0,rotation)
    return 270

def go_forward():
    move(1,0,0,0)

def land():
    vehicle.mode = VehicleMode("LAND")
    while vehicle.mode!='LAND':
        time.sleep(1)
        draw_on_display("Landing")
        print("Waiting for drone to land")
    draw_on_display("Drone Landed")

def follow_path(path,height,cell_length):
    draw_on_display("Drone Starting!!!")                #  x  +  > forward
    take_off(height)                                    #  y  +  > go right
                            # 0 -> x                       z  +  > go down
    prev_cord = path[0]     # 1 -> y                      yaw 60 > turn right 60 degrees
    degree = 0
    draw_on_display("Starting Route")
    for cord in path[1:]:
        if cord[0] == prev_cord[0] and cord[1] > prev_cord[1]:          # Go forward
            degree = point_forward(degree)
            go_forward()
        elif cord[0] == prev_cord[0] and cord[1] < prev_cord[1]:        # Go backwards
            degree = point_backwards(degree)
            go_forward()
        elif cord[0] < prev_cord[0] and cord[1] == prev_cord[1]:        # Turn left, go 1 forward
            degree = point_left(degree)
            go_forward()
        elif cord[0] > prev_cord[0] and cord[1] == prev_cord[1]:        # Turn right, go 1 forward
            degree = point_right(degree)
            go_forward()
        elif cord[0] > prev_cord[0] and cord[1] > prev_cord[1]:         # Go forward 1, right 1
            degree = point_forward(degree)
            go_forward()
            degree = point_right(degree)
            go_forward()
        elif cord[0] < prev_cord[0] and cord[1] > prev_cord[1]:         # Go forward 1, left 1
            degree = point_forward(degree)
            go_forward()
            degree = point_left(degree)
            go_forward()
        elif cord[0] > prev_cord[0] and cord[1] < prev_cord[1]:         # Go backward 1, right 1
            degree = point_backwards(degree)
            go_forward()
            degree = point_right(degree)
            go_forward()
        elif cord[0] < prev_cord[0] and cord[1] < prev_cord[1]:         # Go backward 1, left 1
            degree = point_backwards(degree)
            go_forward()
            degree = point_left(degree)
            go_forward()

        prev_cord = cord    # set new prev_cord
    land()

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
        self.clear_irq_flags(RxDone=1)
        self.set_modem_config_1()
        payload = self.read_payload(nocheck=True)
        data =remove_null_bytes(bytearray(payload).decode('utf-8', 'ignore').strip())

        if 'correct' in data: # Correct data is received and path is ready
            data = json.loads(data)
            draw_on_display("Setting Up!")
            follow_path(data['path'],data['height'],data["cell_length"])    # Start route

        self.set_mode(MODE.SLEEP)
        self.reset_ptr_rx()
        self.set_mode(MODE.RXCONT)



draw_on_display("Connecting..")
try:
    vehicle = connectMyCopter()
except:
    draw_on_display("ERROR!")
    exit(0)

try:
    lora = LoRaRcvCont(verbose=False)
    lora.set_mode(MODE.STDBY)
    lora.set_pa_config(pa_select=1)

    draw_on_display("READY!")
    lora.start()
except KeyboardInterrupt:
    print("\nKeyboardInterrupt")
except:
    draw_on_display("ERROR LoRa!")
    exit(0)
finally:
    lora.set_mode(MODE.SLEEP)
    BOARD.teardown()
