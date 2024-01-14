import serial
import keyboard
import time

ser = serial.Serial('COM10', 9600)  

while True:
    command = ""
    try:
        if keyboard.is_pressed("l"):    # Launch
            command += "L"
        if keyboard.is_pressed("w"):    # Forward
            command += "W"
        elif keyboard.is_pressed("s"):  # Backward
            command += "S"
        if keyboard.is_pressed("a"):    # Left
            command += "A"
        elif keyboard.is_pressed("d"):  # Right
            command += "D"
        if keyboard.is_pressed("e"):    # Yaw CW
            command += "E"
        elif keyboard.is_pressed("q"):  # Yaw CCW
            command += "Q"
        if keyboard.is_pressed("u"):    # Ascend
            command += "U"
        elif keyboard.is_pressed("j"):  # Descend
            command += "J"

        print(command)
        #print(command,end='\r')       
    except KeyboardInterrupt:
        break
    except Exception as e:
        print(e)
        continue
    

    ser.write(command.encode())
    ser.write(b'\n')  

    time.sleep(.3)
