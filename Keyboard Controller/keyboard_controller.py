import serial
import keyboard
import time

ser = serial.Serial('COM10', 9600)  

while True:
    command = ""
    try:
        if keyboard.is_pressed("w"):
            command += "W"
        elif keyboard.is_pressed("s"):
            command += "S"
        if keyboard.is_pressed("a"):
            command += "A"
        elif keyboard.is_pressed("d"):
            command += "D"
        if keyboard.is_pressed("e"):
            command += "E"
        elif keyboard.is_pressed("q"):
            command += "Q"
        if keyboard.is_pressed("u"):
            command += "U"
        elif keyboard.is_pressed("j"):
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