from time import sleep
sleep(2)
from dronekit import connect,VehicleMode,LocationGlobalRelative,APIException
import socket
import math
import argparse
import Adafruit_GPIO.SPI as SPI
import Adafruit_SSD1306
from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont
from SX127x.LoRa import *
from SX127x.board_config import BOARD
from pymavlink import mavutil
import math


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


def draw_on_display(x,y,z,yaw,command):
    global data_write
    global test
    draw.rectangle((0, 0, width, height), outline=0, fill=0)
    draw.text((5,0), str(command),font=font,fill=255)
    draw.text((0, 10), str(x)+" "+str(y)+" "+str(z)+" "+str(yaw)+" ", font=font, fill=255)
    rotated_image = image.rotate(180)  # Rotate the image by 180 degrees
    disp.image(rotated_image)
    disp.display()

def draw_info(data):
    global data_write
    global test
    draw.rectangle((0, 0, width, height), outline=0, fill=0)
    draw.text((5,10), " > "+str(data),font=font,fill=255)
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
        sleep(1)
        print("Waiting for drone to enter GUIDED mode")
    vehicle.armed = True
    while vehicle.armed== False:
        print("Waiting for drone to become armed..")
        vehicle.armed = True
        sleep(1)
    print("\n[Vehicle is now armed]")
    print("[Props are spinning]")
    return None

def loiter():
    print('Arming Vehicle')

    vehicle.mode = VehicleMode("GUIDED")

    while vehicle.mode != 'GUIDED':
        print("Waiting for drone to enter GUIDED mode")
        sleep(1)
    print("Vehicle in GUIDED mode")

    if manualArm == False:
        vehicle.armed = True
        while vehicle.armed == False:
            print("Waiting for vehicle to become armed")
            sleep(1)
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
        sleep(0.5)
    print("Target altitude reached!")
    vehicle.mode = VehicleMode("LOITER")
    return None


def hover():
    print('Arming Vehicle')

    vehicle.mode = VehicleMode("GUIDED")

    while vehicle.mode != 'GUIDED':
        print("Waiting for drone to enter GUIDED mode")
        sleep(1)
    print("Vehicle in GUIDED mode")

    if manualArm == False:
        vehicle.armed = True
        while vehicle.armed == False:
            print("Waiting for vehicle to become armed")
            sleep(1)
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
        sleep(0.5)
    print("Target altitude reached!")

    return None


def take_off_and_land():
    print('Arming Vehicle')

    vehicle.mode = VehicleMode("GUIDED")

    while vehicle.mode != 'GUIDED':
        print("Waiting for drone to enter GUIDED mode")
        sleep(.5)
    print("Vehicle in GUIDED mode")

    if manualArm == False:
        vehicle.armed = True
        while vehicle.armed == False:
            print("Waiting for vehicle to become armed")
            sleep(1)
    else:
        if vehicle.armed == False:
            print("Exiting script. Failed when arming")
            return None
    print("Vehicle is ARMED")
    vehicle.simple_takeoff(2)
    draw_on_display("Flying")
    while True:
        print("Current Altitude: "+str(vehicle.location.global_relative_frame.alt))
        if vehicle.location.global_relative_frame.alt >= .95*2:
            break
        sleep(0.5)
    print("Target altitude reached!")

    return None

def move(x, y, z,yaw):
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


class LoRaRcvCont(LoRa):
    def __init__(self, verbose=False):
        super(LoRaRcvCont, self).__init__(verbose)
        self.set_mode(MODE.SLEEP)
        self.set_dio_mapping([0, 0, 0, 0, 0, 0])

    def start(self):
        self.reset_ptr_rx()
        self.set_mode(MODE.RXCONT)

        while True:
            sleep(.5)
            status = self.get_modem_status()

    def on_rx_done(self):
        self.clear_irq_flags(RxDone=1)
        self.set_modem_config_1()
        payload = self.read_payload(nocheck=True)
        data =remove_null_bytes(bytearray(payload).decode('utf-8', 'ignore').strip())
        
        if "L" in data:
            hover()
            return

        # x positive is forward
        # y positive is right
        # z positive is down

        x,y,z,yaw = 0,0,0,0
        if "W" in data:
            x = 1
        if "S" in data:
            x = -1
        if "D" in data:
            y = 1
        if "A" in data:
            y = -1
        if "E" in data:
            yaw = 5
        if "Q" in data:
            yaw = -5
        if "U" in data:
            z = -1
        if "J" in data:
            z = 1
        draw_on_display(x,y,z,yaw,data)
        move(x,y,z,yaw)

        self.set_mode(MODE.SLEEP)
        self.reset_ptr_rx()
        self.set_mode(MODE.RXCONT)



draw_info("Connecting..")
try:
   vehicle = connectMyCopter()
except:
    draw_info("ERROR!")
    exit(0)

try:
    lora = LoRaRcvCont(verbose=False)
    lora.set_mode(MODE.STDBY)
    lora.set_pa_config(pa_select=1)
    
    draw_info("READY!")
    #hover()
    lora.start()
except KeyboardInterrupt:
    print("\nKeyboardInterrupt")
except:
    draw_info("ERROR LoRa!")
    exit(0)
finally:
    lora.set_mode(MODE.SLEEP)
    BOARD.teardown()
