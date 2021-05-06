import pygame
import math
import struct
import numpy as np
import serial
import sys
import keyboard





pygame.init()
joysticks = []
clock = pygame.time.Clock()
motion = [0, 0]
angle = 0
modulus = 0
data_to_send = np.array([0, 0, 0, 0])  #      0:modulus     1:angle     2:nb appui A/B      3:appui RB
display_data_controller = False
port = None
if len(sys.argv) > 1:
    port_bluetooth = sys.argv[1]
else:
    port_bluetooth = 'com11'

if len(sys.argv) > 2:  # affichage des donnees du controlleur, et plus d'informations
    display_data_controller = True
    print("mode affichages des infos")



def init_communication(port):
    print('Connecting to port {}'.format(port))

    try:
        port_temp = serial.Serial(port, timeout=0.5)
        print("Connected !")
        if display_data_controller:
            print("Port connecté : ", port_temp)
        return port_temp
    except:
        print('Cannot connect to the e-puck2')
        sys.exit(0)



def sendFloatSerial(port, data_for_robot):
    if display_data_controller:
        print("data to send : ", data_for_robot)

    data = data_for_robot.astype(np.int16)

    # to convert to int16 we need to pass via numpy
    size = np.array([data.size], dtype=np.int16)

    send_buffer = bytearray([])

    i = 0
    while (i < size[0]):
        send_buffer += struct.pack('<h', data[i])
        i = i + 1

    port.write(b'ABCD')
    port.write(struct.pack('<h', 2 * size[0]))  #2 car short = 2 bytes
    port.write(send_buffer)
    if display_data_controller:
        print("size : ", struct.pack('<h', 2 * size[0]))
        print("buffer : ", send_buffer)
    print('sent !')



def close_all():
    sys.exit(0)




"""
Boucle infinie de controle : restart, stop, connection...
"""
port = init_communication(port_bluetooth)
while True:
    print("(re)Start")
    input_restart = ""

    # for al the connected joysticks
    for i in range(0, pygame.joystick.get_count()):
        # create an Joystick object in our list
        joysticks.append(pygame.joystick.Joystick(i))
        # initialize them all (-1 means loop forever)
        joysticks[-1].init()

    """
    Boucle infinie recuperation de donnees du controlleur : amplitude, angle, nb appuis A/B
    """
    while True:
        if keyboard.is_pressed(" "): # condition arret de la boucle de donnees
            break
        clock.tick(15)  #15 boucles par seconde
        for event in pygame.event.get():
            if event.type == pygame.JOYAXISMOTION:
                if event.axis < 2:
                    motion[event.axis] = event.value * 100
                    motion[event.axis] = event.value * 100
                    modulus = math.sqrt(pow(motion[0], 2) + pow(motion[1], 2))
                    if modulus > 100:
                        modulus = 100
                    if modulus > 20:
                        angle = math.atan2(motion[1], motion[0]) * 180 / math.pi + 90
                        if angle > 180:
                            angle -= 360
                        angle += 180 #que des angles positifs car on envoie des unsigned int 
                        data_to_send[0] = int(modulus)
                        data_to_send[1] = int(angle)
                    if modulus < 20:
                        data_to_send[0] = 0
                        data_to_send[1] = 0
            elif event.type == pygame.JOYBUTTONDOWN:
                #inversion de A
                if event.button == 0:
                    if data_to_send[2] == 1:
                        data_to_send[2] = 0
                    else:
                        data_to_send[2] = 1
                #détection gachette
                if event.button == 5:
                    data_to_send[3] = 1
            elif event.type == pygame.JOYBUTTONUP:
                if event.button == 5:
                    data_to_send[3] = 2
        # print(clock.get_time())
        if port != None:
            sendFloatSerial(port, data_to_send)
            if data_to_send[3] == 2:
                data_to_send[3] = 0

    while input_restart != "n" or input_restart != "o" or input_restart != " o" or input_restart != " n":
        input_restart = input("restart ?  o / n ->")
        if input_restart == "n" or input_restart == " n":
            close_all()
        elif input_restart == "o" or input_restart == " o":
            break

