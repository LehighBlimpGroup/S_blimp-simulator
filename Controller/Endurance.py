import pygame
import time
import socket
import struct
import math
from simple_pid import PID
from NatNetClient import NatNetClient
from util import quaternion_to_euler
import pandas as pd
import numpy as np
import pygame
from numpy.linalg import norm

from datetime import datetime

# Use current date and time as the file name
currentDateAndTime = datetime.now()
Date = str(currentDateAndTime)
newDate = ''
for l in Date:
    if l == "-" or l == " " or l == ":":
        l = '_'
    newDate += l
anotherNewDate = str(newDate)[0:-7]

#Setup robot PID value
xp = 0.15
xi = 0.002
xd = 0.1

yp = 0.15
yi = 0.002
yd = 0.1

zp = 0.18
zi = 0.0
zd = 0.05/20

#Setup the point we want to do hovering 
waypoint = [[-1.45,-1.9,1.9]]
#Setup the offset of the yaw feed back, to fix the delay problem
yaw_cali = 40 #degrees
#Setup the path and file name for data collection
name = input("Additional Test Info:")
f = open('/home/jiawei/Desktop/TrackingData10082023/globalTrack' + str(anotherNewDate)+"_"+ name+ "_"+".csv","w")

#Initialize the value for our test
l = 0.2 # meters
absz = 0
a_old = 0
a_state = 1
b_old = 0
b_state = 1
tauz = 0
fx = 0
fy = 0
fz = 0
state = 0
count = 0
#Distance to disired position
dist_to_kp = 0
#Feed back of the posiitons and rotations of our robot, from OptiTrack System
positions = {}
rotations = {}
#Setup values for animation
height = 1000
width = 1000

#Get position and rotation data from OptiTrack System
running = True
def receive_rigid_body_frame(id, position, rotation_quaternion):
    # Position and rotation received
    positions[id] = position
    # The rotation is in quaternion. We need to convert it to euler angles
    rotx, roty, rotz = quaternion_to_euler(rotation_quaternion)
    # Store the roll pitch and yaw angles
    rotations[id] = (rotx, roty, rotz)

# Udp and joystick setup for sending commands to robot
UDP_IP = "192.168.0.05"
UDP_PORT = 1234
print("UDP target IP: %s" % UDP_IP)
print("UDP target port: %s" % UDP_PORT)

def udp_init():
    sock = socket.socket(
        socket.AF_INET, # Internet
        socket.SOCK_DGRAM
    ) # UDP
    return sock

def joystick_init():
    pygame.display.init()
    pygame.joystick.init()
    pygame.joystick.Joystick(0).init()

    # Prints the values for axis0
    joystick = pygame.joystick.Joystick(0)
    return joystick

def init():
    sock = udp_init()
    joystick = joystick_init()
    return sock, joystick

def udp_send(sock, ip, port, message):
    sock.sendto(message, (UDP_IP, UDP_PORT))

#Get distance between current position and disired position
def get_distance(p_now,next_waypoint):
    difference = np.subtract(p_now,next_waypoint)
    distance_r3 = norm(difference)
    print("going to point: " + str(next_waypoint) + "distance : " + str(distance_r3))
    return distance_r3

#Drive the robot to disired position by sending Udp package
def goto_point(next_waypoint,p_now,r_now):
    global absz, a_state, b_state, count, a_old, b_old, dist_to_kp, running
    x_pid = PID(xp, xi, xd, setpoint = next_waypoint[0])
    y_pid = PID(yp, yi, yd, setpoint = next_waypoint[1])
    z_pid = PID(zp, zi, zd, setpoint = next_waypoint[2], sample_time=0.01)
    
    # Call PID 
    fx = x_pid(p_now[0])
    fy = y_pid(p_now[1])
    #tauz for height control, tauy is the rotation feed back from OptiTrack System (with the offset)
    tauz = z_pid(p_now[2])
    tauy = math.radians(r_now[2] + yaw_cali)
    fz = 0

    #Get the angle of desire position based on error in x-y plane
    theta = math.atan2(-(next_waypoint[1]- p_now[1]),(next_waypoint[0]- p_now[0])) + math.pi
    #Get the current tau value for Bangbang control 
    if 0 <= (tauy + theta) % (2* math.pi) and  (tauy + theta) % (2* math.pi) < math.pi:
        absz = 0.1 /10
    else:
        absz = -0.1 /10
    #Using Joystick buttons to control the states of robot 
    pygame.event.pump()
    b = joystick.get_button(1)
    a = joystick.get_button(0)
    if a == 1 and a_old == 0:
        a_state = not a_state
    a_old = a
    if b == 1 and b_old == 0:
        b_state = not b_state
    b_old = b
    
    #Print out the values in our command and send to robot Via Udp 
    print(round(fx,2), round(fy,2), round(fz/16,2), round(a_state,2), round(tauy,2), tauz, round(absz,2), round(b_state, 2))
    message = struct.pack('<ffffffff', fx, fy, fz/16, a_state, tauy, tauz, absz, b_state) 
    udp_send(sock, UDP_IP, UDP_PORT, message)
    #Write a record to log file every 20 commands
    if count >=20:
        state = str(time.time() - start_t)+ " "+ str(positions[robot_id][0])+ " "+str(positions[robot_id][1]) +" " +str(positions[robot_id][2])+" " + str(rotations[robot_id][0])+" " + str(rotations[robot_id][1])+" " + str(rotations[robot_id][2])+ " "+ str(fx)+ " "+ str(fy) + " " +str(fz) + " " + str(absz) + " " + str(next_waypoint[0]) + " " + str(next_waypoint[1]) + " " + str(next_waypoint[2]) + " " + str(dist_to_kp) +"\n"
        f.write(state)
        count = 0

    count+=1

    #Animation of the robot position and desired position
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    #Use different color to show rotot and desired position, with trajectory
    goals = pygame.Surface((5, 5))
    goal = pygame.Surface((10, 10))
    bot = pygame.Surface((5,5))
    goals.fill((0,200,0))
    goal.fill((255-(time.time() * 100 % 255), (255 - time.time() * 100 % 255),time.time() % 255))
    bot.fill((20*time.time() % 255, 0, 0))
    rect = goal.get_rect()
    for p in waypoint:
        screen.blit(goals,(width/2 + p[0] * 100, height/2 + p[1] * 100))
    screen.blit(goal,(width/2 + next_waypoint[0] * 100, height/2 + next_waypoint[1] * 100))
    screen.blit(bot,(width/2 - 2 + p_now[0]  *100, height/2 -2 + p_now[1] * 100))
    pygame.display.update()
    if running == False:
        pygame.quit()
    #Stop the program if press a button on joystick
    if a_state == 0:
        f.close()

    
if __name__ == "__main__":
    #Initialize variables and log file 
    sock, joystick = init()
    f.write("time"+ " "+ "x"+ " "+"y" +" " +"z"+" " + "pitch"+" " +"roll"+" " + "yaw"+ " "+ "fx"+ " "+ "fy" + " " +"fz" + " " + "absz" + " " + "waypoint_x" + " " + "waypoint_y" + " " + "waypoint_z" + " " + "Distance_to_r3" +"\n")
    count = 0
    start_t = time.time()
    running = True
    #Initialize Pygame window
    pygame.init()
    screen = pygame.display.set_mode([width, height ])
    screen.fill((255, 255, 255))

    while True:
        #Get position and rotation data from Optitrack System
        clientAddress = "192.168.0.38"
        optitrackServerAddress = "192.168.0.4"
        robot_id = 382
        # This will create a new NatNet client
        streaming_client = NatNetClient()
        streaming_client.set_client_address(clientAddress)
        streaming_client.set_server_address(optitrackServerAddress)
        streaming_client.set_use_multicast(True)
        # Configure the streaming client to call our rigid body handler on the emulator to send data out.
        streaming_client.rigid_body_listener = receive_rigid_body_frame

        # Start up the streaming client now that the callbacks are set up.
        # This will run perpetually, and operate on a separate thread.
        is_running = streaming_client.run()
        
        while is_running:
            if robot_id in positions:
                num_point = len(waypoint)
                #Try to go to our desired waypoints 
                for t_index in range(0,num_point):
                    p_now = [positions[robot_id][0], positions[robot_id][1],positions[robot_id][2]]
                    next_waypoint = waypoint[t_index]
                    dist_to_kp = get_distance(p_now,next_waypoint)
                    #Keep going to the desired point 
                    while dist_to_kp > 0.01 :
                        p_now = [positions[robot_id][0], positions[robot_id][1],positions[robot_id][2]]
                        r_now = [rotations[robot_id][0], rotations[robot_id][1],rotations[robot_id][2]]
                        dist_to_kp = get_distance(p_now,next_waypoint)
                        time_now = time.time()
                        time_diff  = time_now-start_t
                        goto_point(next_waypoint,p_now,r_now)
