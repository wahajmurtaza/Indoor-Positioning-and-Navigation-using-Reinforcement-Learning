import pygame
import serial
from statistics import mean
from math import *
import matplotlib.pyplot as plt

import urllib.request
import http
import threading
from time import time
from time import sleep
from aitwolayer import Dqn


base = "http://192.168.43.160/"
g_angle = 0 #reset
follow_angle = 999
follow_angle_dummy = 999

brain = Dqn(7,3,0.9)
brain.load()





tank_sprite =  pygame.image.load(r'tank1.png')
tank_rect = tank_sprite.get_rect()

#value buffer
B_arr = [2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,5]
C_arr = [2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,5]
D_arr = [2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,5]


#sensors
line1_pointa = (0,0)
line2_pointa = (0,0)
line3_pointa = (0,0)
line1_pointb = (0,0)
line2_pointb = (0,0)
line3_pointb = (0,0)
line1_pointc = (0,0)
line2_pointc = (0,0)
line3_pointc = (0,0)

#colours
blue = (0,0,255)
red = (255,0,0)
green = (0,255,0)
white = (255,255,255)
black = (0,0,0)

#screen
(width, height) = (1280, 720)


#anchor parameters
B_pos = (0,573)
C_pos = (705,36)
D_pos = (710,822)

B_height , C_height, D_height = 233-20, 231-20 ,233-20
B_rad, C_rad, D_rad = 20, 20, 20

screen_height = 0
screen_width = 0

game_position = [800,700]
orientation = 0
'''
goal1= [100,100]
goal2 = [800,700]

#for right
'''
goal1= [1200,300]
goal2 = [250,600]

action = 0
stopped = 1




def transfer(my_url):   #use to send and receive data
    try:
        n = urllib.request.urlopen(base + my_url).read()
        n = n.decode("utf-8")
        return n

    except http.client.HTTPException as e:
        return e

def trackme(x1,y1,r1,x2,y2,r2,x3,y3,r3):
  A = 2*x2 - 2*x1
  B = 2*y2 - 2*y1
  C = r1**2 - r2**2 - x1**2 + x2**2 - y1**2 + y2**2
  D = 2*x3 - 2*x2
  E = 2*y3 - 2*y2
  F = r2**2 - r3**2 - x2**2 + x3**2 - y2**2 + y3**2
  x = (C*E - F*B) / (E*A - B*D)
  y = (C*D - A*F) / (B*D - A*E)
  x1=int(x)
  y1=int(y)
  return (x1,y1)

def draw_circles(screen1,rad1, rad2, rad3):
    pygame.draw.circle(screen1, (255, 0, 0), B_pos, rad1, 1)
    pygame.draw.circle(screen1, (255, 0, 0), B_pos, 5, 0)

    pygame.draw.circle(screen1, (0, 255, 0), C_pos, rad2, 1)
    pygame.draw.circle(screen1, (80, 255, 80), C_pos, 5, 0)

    pygame.draw.circle(screen1, (255, 150, 0), D_pos, rad3, 1)
    pygame.draw.circle(screen1, (255, 150, 0), D_pos, 5, 0)

def make_map(screen2):

    O1  = pygame.draw.rect(screen2, black, (0, 0, 1656, 908), 7)
    O2  = pygame.draw.rect(screen2, blue, (5, 0, 695, 64), 4)
    O3  = pygame.draw.rect(screen2, blue, (700, 0, 277, 36), 4)
    O4  = pygame.draw.rect(screen2, blue, (978, 0, 648, 61), 4)
    O5  = pygame.draw.rect(screen2, blue, (1438, 61, 216, 182), 4)
    O6  = pygame.draw.rect(screen2, blue, (1107, 138, 116, 56), 4)
    O7  = pygame.draw.rect(screen2, blue, (135, 341, 491, 163), 4)
    O8  = pygame.draw.rect(screen2, blue, (1026, 345, 491, 163), 4)
    O9  = pygame.draw.rect(screen2, blue, (139, 672, 491, 163), 4)
    O10  = pygame.draw.rect(screen2, blue, (1028, 688, 491, 163), 4)
    O11  = pygame.draw.rect(screen2, blue, (698, 822, 287, 87), 4)

    return O1,[O2,O3,O4,O5,O6,O7,O8,O9,O10,O11]

def correct_angle(angle):
    if angle >360:
        return angle-360
    if angle <0:
        return  angle + 360
    return angle

def calculate_new_xy(old_xy,speed,d_angle):
    d_angle = d_angle - 90
    angle = radians(d_angle)
    #print(angle,d_angle)
    #print(old_xy[0],old_xy[1])
    #print((speed*cos(angle)))
    new_x = old_xy[0] + (speed * cos(angle))
    new_y = old_xy[1] + (speed * sin(angle))
    #print(new_x, new_y)
    return [int(new_x), int(new_y)]

def turn_right():
    global orientation
    orientation = orientation + 30
    orientation = correct_angle(orientation)

def turn_left():
    global orientation
    orientation = orientation - 30
    orientation = correct_angle(orientation)


def average_filter(val):
    global B_rad,C_rad,D_rad
    z=len(B_arr)
    #B_rad = int(mean(B_arr[z-val:z]) * 100)
    #C_rad = int(mean(C_arr[z-val:z]) * 100)
    #D_rad = int(mean(D_arr[z-val:z]) * 100)
    B_rad = int(B_arr[z-1]*100)
    C_rad = int(C_arr[z-1] * 100)
    D_rad = int(D_arr[z-1] * 100)
    #print("average",B_rad,C_rad,D_rad)

def arduino_map( x,  in_min,  in_max,  out_min,  out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def error_removal():
    global B_rad, C_rad, D_rad
    z = len(B_arr)
    '''
    B_rad = int(arduino_map(B_rad ,212,421,200,400))
    C_rad = int(arduino_map(C_rad ,212,423,200,400))
    D_rad = int(arduino_map(D_rad ,225,425,200,400))
    
    B_rad = int(arduino_map(B_rad, 260, 829, 256, 836))         #cal,cal,real,real
    C_rad = int(arduino_map(C_rad, 126, 546, 97, 525))
    D_rad = int(arduino_map(D_rad, 205, 577, 257, 685))
    '''

    B_rad = int(arduino_map(B_rad, 296, 864, 256, 836))  # cal,cal,real,real
    C_rad = int(arduino_map(C_rad, 143, 590, 204, 525))
    D_rad = int(arduino_map(D_rad, 181, 614, 257, 578))

    #print("error removed",B_rad,C_rad,D_rad)

def lineRect_collide( x1,  y1,  x2,  y2,  rx,  ry,  rw,  rh) :
    left =   lineLine(x1,y1,x2,y2, rx,ry,rx, ry+rh)
    right =  lineLine(x1,y1,x2,y2, rx+rw,ry, rx+rw,ry+rh)
    top   = lineLine(x1,y1,x2,y2, rx,ry, rx+rw,ry)
    bottom = lineLine(x1,y1,x2,y2, rx,ry+rh, rx+rw,ry+rh)

    if  left or right or top or bottom :
        return True
    return False

def lineLine( x1,  y1,  x2,  y2,  x3,  y3,  x4,  y4) :

  # calculate the direction of the lines
    uA = ((x4-x3)*(y1-y3) - (y4-y3)*(x1-x3)) / ((y4-y3)*(x2-x1) - (x4-x3)*(y2-y1))
    uB = ((x2-x1)*(y1-y3) - (y2-y1)*(x1-x3)) / ((y4-y3)*(x2-x1) - (x4-x3)*(y2-y1))

    if uA >= 0 and uA <= 1 and uB >= 0 and uB <= 1 :
        return True
    return False

def constrain():
    global B_rad, C_rad, D_rad
    if B_rad <10:
        B_rad = 10
    if C_rad <10:
        C_rad = 10
    if D_rad <10:
        D_rad = 10

def parse_distance(msg):
    global B_arr, C_arr, D_arr
    name, dist, _ = msg.split(",")
    if name.upper() == 'B':
        B_arr.append(float(dist))
        del B_arr[0]
    if name.upper() == 'C':
        C_arr.append(float(dist))
        del C_arr[0]
    if name.upper() == 'D':
        D_arr.append(float(dist))
        del D_arr[0]

def height_compensate():
    global B_rad, C_rad, D_rad
    z = len(B_arr)
    try:

        B_rad = sqrt(B_arr[z - 1]*B_arr[z - 1]*100*100 - B_height*B_height)
        B_rad = int(B_rad)
        #print('comensated b= ', B_rad)

        #print(B_rad)
        C_rad = sqrt(C_arr[z - 1] * C_arr[z - 1]*100*100 - C_height * C_height)
        C_rad = int(C_rad)
        #print(C_rad)

        D_rad = sqrt(D_arr[z - 1] * D_arr[z - 1]*100*100 - D_height * D_height)
        D_rad = int(D_rad)
        #print("compenstaed = b" ,C_rad  , D_rad)
        print(B_rad,C_rad,D_rad)

    except:
        print("height error")

class SensorLine():
    def __init__(self, line_angle):
        self.line_angle = line_angle
        self.rangea = 150
        self.rangeb = 100
        self.rangec = 50
        self.ranged = 40
        self.rangee = 30
        self.rangef = 20
        self.rangeg = 10
        self.rangeh = 5
        self.range_arr = [150,100,50,40,30,20,10,5,200]
        self.line_distance_number = 0
        self.distance = 200

    def update(self, game_position1, orientation1, obstacles_array1, screen_rect1):
        self.line_distance_number = 8
        self.pointa = calculate_new_xy(game_position1, self.rangea, orientation1 + self.line_angle)
        self.pointb = calculate_new_xy(game_position1, self.rangeb, orientation1 + self.line_angle)
        self.pointc = calculate_new_xy(game_position1, self.rangec, orientation1 + self.line_angle)
        self.pointd = calculate_new_xy(game_position1, self.ranged, orientation1 + self.line_angle)
        self.pointe = calculate_new_xy(game_position1, self.rangee, orientation1 + self.line_angle)
        self.pointf = calculate_new_xy(game_position1, self.rangef, orientation1 + self.line_angle)
        self.pointg = calculate_new_xy(game_position1, self.rangeg, orientation1 + self.line_angle)
        self.pointh = calculate_new_xy(game_position1, self.rangef, orientation1 + self.line_angle)
        self.array_pt = [self.pointa, self.pointb, self.pointc, self.pointd, self.pointe, self.pointf, self.pointg,
                         self.pointh]

        for number in range(0, 8):
            for ob in obstacles_array1:
                if ob.collidepoint(self.array_pt[number]) or not (screen_rect1.collidepoint(self.array_pt[number])):
                    self.line_distance_number = number

        #print(self.range_arr[self.line_distance_number])
        self.distance = self.range_arr[self.line_distance_number]
sensor1class = SensorLine(0)
sensor2class = SensorLine(30)
sensor3class = SensorLine(-30)



def draw_sensors(screen1):
    global game_position, orientation, line1_pointa, line1_pointb, line1_pointc, line2_pointa, line2_pointb, line2_pointc, line3_pointa, line3_pointb, line3_pointc
    rangea = 50
    rangeb = 100
    rangec =150

    line1_pointa= calculate_new_xy(game_position, rangec,orientation-30)
    line1_pointb = calculate_new_xy(game_position, rangeb, orientation - 30)
    line1_pointc = calculate_new_xy(game_position, rangea, orientation - 30)
    #pygame.draw.line(screen1,red,game_position,line1_pointa,1)

    line2_pointa = calculate_new_xy(game_position, rangec, orientation )
    line2_pointb = calculate_new_xy(game_position, rangeb, orientation)
    line2_pointc = calculate_new_xy(game_position, rangea, orientation)
    #pygame.draw.line(screen1, green, game_position, line2_pointa, 1)

    line3_pointa = calculate_new_xy(game_position, rangec, orientation + 30)
    line3_pointb = calculate_new_xy(game_position, rangeb, orientation + 30)
    line3_pointc = calculate_new_xy(game_position, rangea, orientation + 30)
    #pygame.draw.line(screen1, blue, game_position, line3_pointa, 1)

def find_angle_btw_points(source,destination):
    my_radian = atan2(destination[1] - source[1], destination[0] - source[0])
    return degrees(my_radian)

def find_distance(p1,p2):
    return sqrt(((p1[0] - p2[0]) ** 2) + ((p1[1] - p2[1]) ** 2))

def draw_lines(screen1, x_index, y_index):

    for i in y_vals:
        pygame.draw.line(screen1, red, (0, i), (1656, i))
    for i in x_vals:
        xl4 = pygame.draw.line(screen1, red, (i, 0), (i, 908))

    pygame.draw.line(screen1, green, (x_vals[x_index],0), (x_vals[x_index],908))
    pygame.draw.line(screen1, green, (0, y_vals[y_index]), (1656, y_vals[y_index]))

def draw_sensors(screen1):

    pygame.draw.line(screen1, red, game_position, sensor1class.pointa, 1)
    pygame.draw.line(screen1, red, game_position, sensor2class.pointa, 1)
    pygame.draw.line(screen1, red, game_position, sensor3class.pointa, 1)




next__distance_call = time()
def get_wifi_distances():
    global B_arr, C_arr, D_arr, next__distance_call
    try:
        datax = str(transfer("distances"))
        print("distance",datax)
        dummy_B, dummy_C, dummy_D,_ = datax.split(",")
        if float(dummy_B) is not 2.0 and float(dummy_B) is not 3.0:
           # if float(dummy_B) >2.0 and float(dummy_B) <15.0:
            B_arr.append(float(dummy_B))
        if float(dummy_C) is not 2.0 and float(dummy_C) is not 3.0:
            #if float(dummy_C) > 2.0 and float(dummy_C) < 15.0:
            C_arr.append(float(dummy_C))
        if float(dummy_D) is not 2.0 and float(dummy_D) is not 3.0:
            #if float(dummy_D) > 2.0 and float(dummy_D) < 15.0:
            D_arr.append(float(dummy_D))
    except:
        print("getting distance error")
    next__distance_call = next__distance_call + 1
    threading.Timer(next__distance_call- time(), get_wifi_distances).start()
get_wifi_distances()


next_call = time()
def get_car_angle():             #also send data
    global orientation , next_call
    try:
        dataz = transfer("angle")
        #dataz = transfer("angle")
        #print(dataz)
        new_dataz = str(dataz)
        #print("string",dataz)
        #new_dataz, _ = new_dataz.split("\r")
        #print(new_dataz)
        xz = float(new_dataz)
        xz = int(xz) - g_angle
        orientation = correct_angle(xz)
        print("angle",dataz,orientation)
    except:
        print("angle error")

    global follow_angle
    try:

        if stopped == 1:

            _ = transfer(str(4))
        else:
            _ = transfer(str(action))
    except:
        print("sending data")


    next_call = next_call + 1
    threading.Timer(next_call - time(), get_car_angle).start()
get_car_angle



#Initializing Pygame
running = True
pygame.init()
main_screen = pygame.display.set_mode((width, height))
screen = pygame.Surface((1656, 908))
pygame.transform.scale(screen,(width, height), main_screen)
pygame.display.set_caption("Indoor Positioning System")
screen.fill((255,255,255))






#this is fake valiue
my_position=(150,120)



def loop():
    global screen_height, screen_width, game_position, orientation, goal1,goal2, follow_angle,follow_angle_dummy, stopped, action
    goal_distance , last_goal_distance = 0,0
    last_game_position = game_position
    running = True
    goal = goal1
    start_following = False
    collision=0

    while running is True:

        collision = 0
        stopped = 0

        main_screen.fill((255, 255, 255))
        screen.fill((255, 255, 255))
        screen_rect, obstacles_arr = make_map(screen)

        ev = pygame.event.get()
        for event in ev:

            if event.type == pygame.QUIT:
                running = False
            if event.type ==pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                if event.key == pygame.K_w:
                    screen_height = screen_height+50
                if event.key == pygame.K_s:
                    screen_height = screen_height-50
                if event.key == pygame.K_a:
                    screen_width = screen_width+50
                if event.key == pygame.K_d:
                    screen_width = screen_width-50
                if event.key == pygame.K_UP:
                    game_position = calculate_new_xy(game_position,30,orientation)
                    #game_position = calculate_new_xy(game_position, 10, orientation)
                if event.key == pygame.K_RIGHT:
                    turn_right()
                if event.key == pygame.K_LEFT:
                    turn_left()
                if event.key == pygame.K_z:
                    del path_corners[0]
                if event.key == pygame.K_x:
                    del path_corners[len(path_corners)-2]
                if event.key == pygame.K_f:
                    start_following = not start_following
            if event.type == pygame.MOUSEBUTTONDOWN:
                # 1 is the left mouse button, 2 is middle, 3 is right.
                if event.button == 1 or event.button == 2 or event.button == 3:
                    #print(event.pos)
                    pos = list(event.pos)
                    #print(pos)
                    ratio_x = ((width-20)/screen_rect.width )
                    ratio_y = ((height-20)/screen_rect.height )
                    scaled_pos =[pos[0] / ratio_x, pos[1] / ratio_y]
                    scaled_pos[0] = int(scaled_pos[0]) - 10
                    scaled_pos[1] = int(scaled_pos[1]) - 10
                    goal = scaled_pos
                    print(goal)


        respective_angle = find_angle_btw_points(game_position, goal)
        respective_angle = correct_angle(respective_angle)
        sensor1class.update(game_position, orientation, obstacles_arr, screen_rect)
        sensor2class.update(game_position, orientation, obstacles_arr, screen_rect)
        sensor3class.update(game_position, orientation, obstacles_arr, screen_rect)
        for obstacle in obstacles_arr:
            if tank_rect.colliderect(obstacle):
                collision = 1
        last_signal = [sensor1class.distance, sensor2class.distance, sensor3class.distance,respective_angle
                       -respective_angle, sin(respective_angle),
                       cos(respective_angle), tan(respective_angle)]




        if start_following is True:
            goal_distance = find_distance(game_position, goal)
            if goal_distance<50:
                #del path_corners
                start_following = False
                follow_angle = 999
                del  path_corners
            else:
                path_distance = find_distance(game_position, path_corners[0])
                #print(path_distance)
                follow_angle_dummy = find_angle_btw_points(game_position,path_corners[0])
                follow_angle_dummy = correct_angle(follow_angle_dummy + 90)
                follow_angle = int(correct_angle(follow_angle_dummy + g_angle))
                #print(follow_angle_dummy, follow_angle)
                if(path_distance<50):
                    del path_corners[0]
        if start_following is False:
            follow_angle = 999

        last_reward = -1  # and last_rewad = -0.2
        # last_reward = -steps /1000


        if goal_distance < last_goal_distance:
            last_reward = 0.6
        if game_position[0] < 10 or game_position[0] > 1646 or game_position[1] < 10 or game_position[
            1] > 898 or collision == 1:  # boundaries
            last_reward = -200
        if goal_distance < 50:
            last_reward = 100
            stopped = 1


        action = brain.update(last_reward,last_signal)




        height_compensate()
        error_removal()
        #constrain()
        draw_circles(screen,B_rad,C_rad,D_rad)

        game_position = trackme(B_pos[0], B_pos[1], B_rad, C_pos[0], C_pos[1], C_rad, D_pos[0],
                              D_pos[1], D_rad)



        tank_body = pygame.transform.rotate(tank_sprite, -orientation)
        tank_rect.center = game_position
        draw_sensors(screen)
        screen.blit(tank_body,tank_rect)


        pygame.draw.circle(screen,red,goal,5,0)





        if not(screen_rect.collidepoint(game_position)):
            game_position = last_game_position


        last_goal_distance = goal_distance
        last_game_position = game_position
        collision = 0




        #draw_sensors(screen)
        zzz = pygame.transform.scale(screen, (width-20, height-20))
        main_screen.blit(zzz,(10,10))
        pygame.display.update()
        sleep(0.5)



loop()
