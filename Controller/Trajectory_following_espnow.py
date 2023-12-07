import serial
import time
import numpy as np
import sys
import pygame
import time
import socket
import struct
import math
import numpy as np
from simple_pid import PID
from NatNetClient import NatNetClient
from util import quaternion_to_euler
from numpy.linalg import norm

#Setup desired trajectory
Triangle = True
Lissajous = False 

# Lissajous trajectory features
w = 0.11#rad/s
radius = 2 #m

# Triangle trajectory features
v = 0.30#m/s
side_length = 3 #m

#Setup for ESPNow comunication
PORT = '/dev/ttyACM0'
joynum = 0

#Setup robot PID value
xp = 0.1 * 2
xi = 0.0006
xd = 0.0001

yp = 0.1 
yi = 0.0006
yd = 0.0001

zp = 0.2 /10
zi = 0.02
zd = 0.2 

#Setup the offset of the yaw feed back, to fix the delay problem
yaw_cali = 40 #degrees

#Setup for caculation of yaw rate
k_yaw =  10 * 2*math.pi
yaw_rate = 2*math.pi
last_yaw = 0
start_T =0
last_time = 0
last_yawrate_time = 0
last_point = [0,0,0]
last_point_time = 0

from datetime import datetime
currentDateAndTime = datetime.now()
Date = str(currentDateAndTime)
newDate = ''
for l in Date:
    if l == "-" or l == " " or l == ":":
        l = '_'
    newDate += l
anotherNewDate = str(newDate)[0:-7]

#Setup the path and file name for data collection
f = open('/home/jiawei/Desktop/TrackingData06092023/globalTrack' + str(anotherNewDate)+"_"+ str(yaw_cali)+ "_"+".csv","w")

#Feed back of the posiitons and rotations of our robot, from OptiTrack System
positions = {}
rotations = {}

#Desired center point of the trajectory
z_height = 1.5 #m
x_point = -5. #m in optitrack space
y_point = 0 #m in optitrack space
groundz = 1.5

#Setup values for animation
height = 1000
width = 1000

#Get position and rotation data from OptiTrack System
def receive_rigid_body_frame(id, position, rotation_quaternion):
    # Position and rotation received
    positions[id] = position
    # The rotation is in quaternion. We need to convert it to euler angles
    rotx, roty, rotz = quaternion_to_euler(rotation_quaternion)
    # Store the roll pitch and yaw angles
    rotations[id] = (rotx, roty, rotz)

#ESPNow comunication setup and Joystick setup
class Control_Input:
    def __init__(self, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13):
        self.p1 = p1
        self.p2 = p2
        self.p3 = p3
        self.p4 = p4
        self.p5 = p5
        self.p6 = p6
        self.p7 = p7
        self.p8 = p8
        self.p9 = p9
        self.p10 = p10
        self.p11 = p11
        self.p12 = p12
        self.p13 = p13

    def __str__(self) -> str:
        return (
            '<'
            + str(self.p1)
            + '|'
            + str(self.p2)
            + '|'
            + str(self.p3)
            + '|'
            + str(self.p4)
            + '|'
            + str(self.p5)
            + '|'
            + str(self.p6)
            + '|'
            + str(self.p7)
            + '|'
            + str(self.p8)
            + '|'
            + str(self.p9)
            + '|'
            + str(self.p10)            
            + '|'
            + str(self.p11)
            + '|'
            + str(self.p12)
            + '|'
            + str(self.p13)
            + '>'
        )


def espnow_init():
    ser = serial.Serial(PORT, 115200)
    return ser

def esp_now_send(ser, input):
    try:
        # NOTE - The daley here need to match the delay in the ESP32 receiver code
        message = str(input)
        ser.write(message.encode())
        try:
            incoming = ser.readline().decode(errors='ignore').strip()
            print("Received Data: " + incoming)
        except UnicodeDecodeError:
            print("Received malformed data!")
    except KeyboardInterrupt:
        print("Exiting Program")
        ser.close()

def joystick_init():
    pygame.display.init()
    pygame.joystick.init()
    pygame.joystick.Joystick(joynum).init()
    # Prints the values for axis0
    joystick = pygame.joystick.Joystick(joynum)
    return joystick

def init():
    joystick = joystick_init()
    return joystick

# Update the yawrate
def get_yaw_rate(r_now):
    global yaw_rate,last_yawrate_time,last_yaw
    if last_yawrate_time == 0:
        last_yawrate_time = time.time()
        last_yaw = r_now[2]
        return yaw_rate
    t = time.time()
    err_yaw = math.radians(-(r_now[2] - last_yaw))
    err_yaw = max(0.001,err_yaw)
    # get ave of yawrate
    yaw_rate = yaw_rate * 0.98 + (err_yaw / (t - last_yawrate_time)) * 0.02
    last_yaw = r_now[2]
    last_yawrate_time = t
    return yaw_rate

#Get distance error 
def get_curve(p1,p2,t):
    err = [p2[0]-p1[0],p2[1]-p1[1],p2[2]-p1[2]]

#Get distance between current position and disired position
def get_distance(p_now,next_waypoint):
    difference = np.subtract(p_now,next_waypoint)
    distance_r3 = norm(difference)
    return distance_r3

#Generate the lassajous trajectory base on current time
def get_waypoint():
    global theta, last_time, last_point    
    if last_time == 0:
        last_time = time.time() - start_t
    t = time.time() - start_t
    
    vel_mag = get_velocity()
    theta = w * (t + (t - last_time) / vel_mag) #% (2 * math.pi)
    
    #lisarous
    if (t - last_time < 0.1):
        return last_point
    else:
        next_waypoint = [ ((x_point) + 2*radius*math.sin(theta + math.pi/2)),
                          ((y_point) + radius*math.sin(2* theta)),
                          z_height]

        last_time = t
        last_point = next_waypoint
        return next_waypoint

#Get velocity of the trajectory
def get_velocity():
    global theta
    vel_mag = math.sqrt((theta * radius*math.cos(theta + math.pi/2))**2+(2**2 * 2 * theta * radius * math.cos(2* theta))**2)
    return vel_mag

#Generate the triangle trajectory base on current time
def get_triangle():
    global last_time, last_point, start_T
    T = side_length / v
    t = time.time() - start_T 
    #Update every 0.1s
    if (t - last_time < 0.1):
        return last_point
    # get three sides depends on time
    if 0 <= t < T:
        next_waypoint = [(x_point) + v*t,(y_point) + 0,z_height]
    elif T <= t < (2*T):
        next_waypoint = [(x_point) + v*T - 0.5*v*(t-T),(y_point) +(math.sqrt(3)/2)*v*(t-T),z_height]
    elif (2* T) <= t < (3*T):
        next_waypoint = [(x_point) + 0.5*v*T- 0.5*v*(t-2*T),(y_point) + (math.sqrt(3)/2)*v*T -(math.sqrt(3)/2)*v*(t-2*T),z_height]
    if t>3*T:
        start_T = start_T + 3*T
        return get_triangle()
    return next_waypoint

#Drive the robot to disired position by sending ESPNow package
def goto_point(next_waypoint,p_now,r_now):
    global absz, a_state, b_state, count, a_old, b_old, dist_to_kp,running, last_point, last_point_time, theta, plot_x, yaw_rate, yaw_cali
    z_pid = PID(zp, zi, zd, setpoint = next_waypoint[2], sample_time=0.01)
    tauy = z_pid(p_now[2])
    #update yaw offset base on current yawrate
#     yaw_rate = get_yaw_rate(r_now)
#     yaw_cali = 20 + (k_yaw / yaw_rate)
    taux = zp
    # send current facing angle of the robot
    tauz = math.radians(r_now[2] + yaw_cali)

    fx = 0
    fy = 0
    fz = 0
    dis = get_distance(p_now,next_waypoint)
    pygame.event.pump()
    
    #Using Joystick buttons to control the states of robot 
    b = joystick.get_button(1)
    a = joystick.get_button(0)
    if a == 1 and a_old == 0:
        a_state = not a_state
    a_old = a
    if b == 1 and b_old == 0:
        b_state = not b_state
    b_old = b

    #Send the error in x-y plane, robot will generate desired angle 
    err_x = (next_waypoint[0]- p_now[0]) 
    err_y = -(next_waypoint[1]- p_now[1]) 
    err_z = (next_waypoint[2] - p_now[2])
    absz = z_height - groundz          
    snap = 0
    #Print out the values in our command and send to robot Via ESPNow 
    print(round(err_x,2), round(err_y,2), round(fz/16,2), round(a_state,2), round(tauz,2), tauy, round(absz,2), round(b_state, 2))
    esp_now_input = Control_Input(0,err_x, err_y, fz, taux, tauz, tauy,  absz, b_state, snap, 0, 0, 0)
    esp_now_send(sock, esp_now_input)
    #Write a record to log file every 10 commands
    if count >=10:
        state = str(time.time() - start_t)+ " "+ str(positions[robot_id][0])+ " "+str(positions[robot_id][1]) +" " +str(positions[robot_id][2])+" " + str(rotations[robot_id][0])+" " + str(rotations[robot_id][1])+" " + str(rotations[robot_id][2])+ " "+ str(fx)+ " "+ str(fy) + " " +str(fz) + " " + str(absz) + " " + str(next_waypoint[0]) + " " + str(next_waypoint[1]) + " " + str(next_waypoint[2]) + " " + str(yaw_rate) +" " + str(dist_to_kp) +"\n"
        f.write(state)
        count = 0
    count+=1
    
    #Animation of the robot position and desired position
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    #Use different color to show rotot and desired position, with trajectory        
    goal = pygame.Surface((5, 5))
    goals = pygame.Surface((5, 5))
    bot = pygame.Surface((5,5))
    goals.fill((0,200,0))
    goal.fill((255-(time.time() * 100 % 255), (255 - time.time() * 100 % 255),time.time() % 255))
    bot.fill((200, 0, 0))
    rect = goal.get_rect()
    for p in waypoint:
        screen.blit(goals,(width/2 -2 + (p[0] - x_point)* 100, height/2 -2 + (p[1] - y_point) * 100))
    screen.blit(goal,(width/2 - 2 + (next_waypoint[0] - x_point) *100, height/2 - 2 + (next_waypoint[1] - y_point)* 100))
    screen.blit(bot,(width/2 - 2 + (p_now[0] -x_point) *100, height/2 -2 + (p_now[1] - y_point) * 100))
    pygame.draw.line(screen,(0,200,0),(plot_x, 100),(plot_x, 100 + 100* err_z), 1)
    plot_x = plot_x + 1
    if plot_x > width:
        plot_x = 0
        pygame.draw.line(screen,(255,255,255),(0,0),(0, 200), 2000)
    
    pygame.display.update()
    if running == False:
        pygame.quit()
    
    #Stop the program if press a button on joystick
    if a_state == 0:
        f.close()

if __name__ == "__main__":
    #Initialize variables and log file 
    sock = espnow_init()
    joystick = init()
    l = 0.2 # meters
    absz = 0
    b_old = 0
    b_state = 1
    a_old = 0
    a_state = 1
    f.write("time"+ " "+ "x"+ " "+"y" +" " +"z"+" " + "pitch"+" " +"roll"+" " + "yaw"+ " "+ "fx"+ " "+ "fy" + " " +"fz" + " " + "absz" + " " + "waypoint_x" + " " + "waypoint_y" + " " + "waypoint_z" + " " + "yaw_rate" + " " + "Distance_to_r3" +"\n")
    count = 0
    start_t = time.time()
    start_T = start_t
    theta = start_t

    #Initialize Pygame window
    running = True
    pygame.init()
    screen = pygame.display.set_mode([width, height])
    screen.fill((255, 255, 255))
    plot_x = 0
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
                tic = time.time()
                #Get position and rotation feed back from Optitrack system
                p_now = [positions[robot_id][0], positions[robot_id][1],positions[robot_id][2]]
                r_now = [rotations[robot_id][0], rotations[robot_id][1],rotations[robot_id][2]]
                #Generate the trajectory and go to desired position 
                if Triangle == True:
                    next_waypoint = get_triangle()
                elif Lissajous == True:
                    next_waypoint = get_waypoint()
                dist_to_kp = get_distance(p_now,next_waypoint)
                goto_point(next_waypoint,p_now,r_now)




