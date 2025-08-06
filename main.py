import os
import argparse
from unittest.mock import right

import serial
import math
import pygame
import time

ser = None

def create_parser():
    parser = argparse.ArgumentParser(description="Robotic Arm CLI")
    parser.add_argument("-a","--about",action="store_true",help="about_us")
    parser.add_argument("-s","--settings",action="store_true",help="Creates the Settings.txt file which stores the robotic arms settings")
    parser.add_argument("-d", "--debug" , action="store_true",help="debug")
    parser.add_argument("-ho","--home" , action="store_true",help="sends homing command threw serial to arduino")
    parser.add_argument("-p","--position",nargs=4,type=float,help="go into a position using inverse kinnematiks")
    parser.add_argument("-sg","--sgcode",nargs=1,type=str ,help="sends gcode for keyframing and not only")
    parser.add_argument("-j","--jog",nargs=5,type=float,help="joging the arm")
    parser.add_argument("-z","--zero",action="store_true",help="Move Rob_Arm to zeor/homed position")

    parser.add_argument("-c","--controller" , action="store_true",help="euns the inverse kinnematics using a wireless controller")


    return parser

def initialize_serial():
    global ser
    config = settings()
    com = config.get('COM')
    baund = config.get('BAUND')
    ser = serial.Serial("com"+str(com),baund)

def home():

    initialize_serial()
    ser.write("$H\n".encode())
    ser.write("G10 L20 P1 X0 Y0 Z0 A0\n".encode())
    ser.write("G1 F2000 X171.5 Y85 Z104 A76\n ".encode())
    ser.write("G10 L20 P1 X0 Y0 Z0 A0\n".encode())
    response = ser.readline().decode().strip()
    print("Response from Arduino:", response)


def debug():
    print("DEBUG_MODE")
    config = settings()
    com = config.get('COM')
    print(com)

def create_settings():
 if os.path.exists("settings.txt"):
     resp = input("File Exists, want overwirte?")
     if (resp == "Y" or "Yes" or "y" or "yes"):
         os.remove("settings.txt")
         print("File removed, Creating New")


 with open("settings.txt", "a") as file:
     file.write("---------------------------------------------------------" + "\n")
     file.write("This is the settings file for your robotic arm" + "\n")
     file.write("---------------------------------------------------------" + "\n")
     file.write("COM = 17" + "\n")
     file.write("BAUND = 115200" + "\n")
     file.write("xjoint = 300" +"\n")
     file.write("yjoint = 300" + "\n")
     file.write("zjoint = 100"+"\n")

     file.write("yjoint_xaxis_transform = 80" + "\n")
     file.write("yjoint_zaxis_transform = 224" + "\n")
     file.write("controller_dead_zone = 0.2")
     print("Settings File created succesfully")

def settings():
  config = {}

  try:
   with open('settings.txt', 'r') as file:
       for line in file:
          line = line.strip()
          if line and '=' in line:
              key, value = line.split('=',1)
              key = key.strip()
              value = value.strip()

              if value.isdigit():
                  value = int(value)
              elif value.lower() == 'true':
                  value =  True
              elif value.lower() == 'false':
                  value = False
              config[key] = value

  except FileNotFoundError:
      print(f"File 'settings.txt' not found.")
  except Exception as e:
      print(f"An error occurred while reading the file: {e}")

  return config

def about_us():

    print("")
    print("----------------------------------------------------------------------------------------------")
    print(r"           _           _   _                        _             _ _                   ____  ")
    print(r"  _ __ ___ | |__   ___ | |_(_) ___    ___ ___  _ __ | |_ _ __ ___ | | | ___ _ __  __   _|___ ")
    print(r" | '__/ _ \| '_ \ / _ \| __| |/ __|  / __/ _ \| '_ \| __| '__/ _ \| | |/ _ \ '__| \ \ / / __) |")
    print(r" | | | (_) | |_) | (_) | |_| | (__  | (_| (_) | | | | |_| | | (_) | | |  __/ |     \ V / / __/")
    print(r" |_|  \___/|_.__/ \___/ \__|_|\___|  \___\___/|_| |_|\__|_|  \___/|_|_|\___|_|      \_/ |_____|")
    print("----------------------------------------------------------------------------------------------")
    print("This is a student project for controlling HollyV2 and other rob_arms with inverse kinnematics")
    print("Property of HOLLY_INDUSTRIES.CC")
    print("Compatable with Holly V2 Robotic Arm")
    print("")
    print("")
    print("")

def inversekinnematics():
    config = settings()
    parser = create_parser()
    args = parser.parse_args()
    x, y, z,f = args.position

    xjoint = config.get('xjoint')
    yjoint = config.get('yjoint')
    zjoint = config.get('zjoint')

    x_transform = config.get('yjoint_xaxis_transform')
    z_transform = config.get('yjoint_zaxis_transform')

# offsets gia x kai y

    ypot = ((math.sqrt((x * x) + (y * y))))
    xdeg = math.degrees(math.asin(y / ypot))

    ypot1 = ypot - x_transform
    ynew = ypot1 * (y / ypot1)
    xnew = ypot1 * math.cos(math.radians(xdeg))
    ypot2 = ypot1 - x_transform

    zt = z - z_transform
#------------------------------------------------
    ypoteinousa1 = ((math.sqrt((xnew * xnew) + (ynew * ynew))))

    ypoteinousa2 = math.sqrt((ypoteinousa1 * ypoteinousa1) + (zt * zt))

    undetriangle = math.degrees(math.asin(zt / ypoteinousa2))

    upertriangle = math.degrees(math.acos(
        ((xjoint * xjoint) + (ypoteinousa2 * ypoteinousa2) - (yjoint * yjoint)) /
        (2 * xjoint * ypoteinousa2)
    ))

    ydeg =  upertriangle + undetriangle

    zdeg = math.degrees( math.acos((((yjoint * yjoint) + (xjoint * xjoint) - (ypoteinousa2 * ypoteinousa2)) / (2 * xjoint * yjoint))))


    print("-------------------------------")
    print(f"xdeg: {xdeg}" )
    print(f"ydeg: {ydeg}")
    print(f"zdeg:  {zdeg}")
    print("-------------------------------")

    initialize_serial()
    global ser

    print("SENT TO ARDUINO")
    command = ("G1 f{} X{} Y{} Z{}\n".format(f, round(xdeg, 2), round(90-ydeg, 2), round(180-zdeg ,2)).encode())
    ser.write(command)
    print(command)
    response = ser.readline().decode().strip()
    print("Response from Arduino:", response)
    print("")
    print("")


def gcode_sender():

 global ser

 parser = create_parser()
 args = parser.parse_args()
 filename = args.sgcode
 print(filename)

 try:
    initialize_serial()
    with open(filename, 'r')as file:
        for line in file:
            gcode = line.strip().split(';')[0].strip()
            if gcode:
                ser.write((gcode+'\n').encode('utf-8'))
                print(f"Sent: {gcode}")

                response = ser.readline().decode('utf-8').strip()
                print(f"Arduino: {response}")



 except FileNotFoundError:
        print(f"Error: The file {filename} does not exist.")

 except Exception as e:
        print(f"Unexpected error: {e}")


def jog():

    initialize_serial()
    parser = create_parser()
    args = parser.parse_args()
    xmot, ymot, zmot, emot,ff = args.jog
    print(f"jog to potition X:{xmot} Y:{ymot} Z:{zmot} E:{emot}  FORCE:{ff}")

    jcom = command = ("G1 X{} Y{} Z{} E{} F{} \n".format(xmot,ymot,zmot,emot,ff)).encode()
    initialize_serial()
    print(jcom)
    ser.write(jcom)
    response = ser.readline().decode().strip()
    print("Response from Arduino:", response)
    print("")


def arm_zero():

    initialize_serial()
    ser.write("G1 F3000 X0 Y0 Z0 A0\n".encode())
    print("Robotic Arm going to zero")
    response = ser.readline().decode().strip()
    print("Response from Arduino:", response)

def controller():

    print("Controller Mode Initialised")
    pygame.init()
    pygame.joystick.init()

    # Check if any joystick is connected
    if pygame.joystick.get_count() == 0:
        print("No joystick connected.")
        return

    # Get the joystick object
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    x = 300
    y = 300
    z = 400

    print(f"Joystick detected: {joystick.get_name()}")
    print(f"Number of axes: {joystick.get_numaxes()}")

    initialize_serial()
    global ser

    try:
        while True:
            # Pump events to update the state
            pygame.event.pump()

            deadzone = 0.2

            def apply_deadzone(value, deadzone):
                if abs(value) < deadzone:
                    return 0
                return value

            # Get joystick inputs and apply the deadzone
            left_x = apply_deadzone(round(joystick.get_axis(0) * 1, 2), deadzone)
            left_y = apply_deadzone(round(joystick.get_axis(1) *-1, 2), deadzone)
            right_x = apply_deadzone(round(joystick.get_axis(2), 2), deadzone)
            right_y = apply_deadzone(round(joystick.get_axis(3) * -1, 2), deadzone)

            # Update x and y positions based on joystick inputs
            x = x + left_y * 10
            y = y + left_x * 10
            z = z + right_y * 7
            f=10000


            print("x:" + str(x) + " y:" +str(y) +" z:" +str(z))

            config = settings()




            xjoint = config.get('xjoint')
            yjoint = config.get('yjoint')
            zjoint = config.get('zjoint')

            x_transform = config.get('yjoint_xaxis_transform')
            z_transform = config.get('yjoint_zaxis_transform')

            # offsets gia x kai y

            ypot = ((math.sqrt((x * x) + (y * y))))
            xdeg = math.degrees(math.asin(y / ypot))

            ypot1 = ypot - x_transform
            ynew = ypot1 * (y / ypot1)
            xnew = ypot1 * math.cos(math.radians(xdeg))
            ypot2 = ypot1 - x_transform

            zt = z - z_transform
            # ------------------------------------------------
            ypoteinousa1 = ((math.sqrt((xnew * xnew) + (ynew * ynew))))

            ypoteinousa2 = math.sqrt((ypoteinousa1 * ypoteinousa1) + (zt * zt))

            undetriangle = math.degrees(math.asin(zt / ypoteinousa2))

            upertriangle = math.degrees(math.acos(
                ((xjoint * xjoint) + (ypoteinousa2 * ypoteinousa2) - (yjoint * yjoint)) /
                (2 * xjoint * ypoteinousa2)
            ))

            ydeg = upertriangle + undetriangle

            zdeg = math.degrees(math.acos(
                (((yjoint * yjoint) + (xjoint * xjoint) - (ypoteinousa2 * ypoteinousa2)) / (2 * xjoint * yjoint))))


            command = (
                "G1 f{} X{} Y{} Z{}\n".format(f, round(xdeg, 2), round(90 - ydeg, 2), round(180 - zdeg, 2)).encode())
            ser.write(command)

            time.sleep(0.05)




    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        # Quit pygame
        pygame.quit()



def main():
        parser = create_parser()
        args = parser.parse_args()

        # Check if none of the arguments are provided
        if not (args.about or args.settings or args.debug or args.home or args.position or args.sgcode or args.jog or args.zero or args.controller):
            parser.print_help()
            return

        if args.about:
            about_us()

        if args.settings:
            create_settings()

        if args.debug:
            debug()

        if args.home:
            home()

        if args.position:
            inversekinnematics()

        if args.sgcode:
            gcode_sender()

        if args.jog:
            jog()

        if args.zero:
            arm_zero()

        if args.controller:
            controller()




if __name__ == "__main__":
        main()




