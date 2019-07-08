#Draw Robot Test
#To help figure out the dynamics of drawing the robot in openCV space
import math
import cv2
import numpy as np 

LOOKAHEAD_DISTANCE = 25


##************************************************
## Def: Find Lookahead Point
def FindLookaheadPoint():

    l_x = robot_pos[0] + LOOKAHEAD_DISTANCE/math.sqrt(2)
    l_y = robot_pos[1] + LOOKAHEAD_DISTANCE/math.sqrt(2)
    
    return(  l_x, l_y)


##************************************************
## Def: Draw Robot
## Draws Robot and parameters
def DrawRobot(img):

    ROBOT_WIDTH  = 26
    ROBOT_LENGTH = 24.5
    robot_heading_rad = math.radians(-robot_heading)


    #Slice length and width into x/y components and calculate corner coordinates in inches
    #   4---1        (0,0) *---->  +x            0
    #   | 0 |              |                -90 -+- 90
    #   3---2           +y |                    180
    width_dx  = (ROBOT_WIDTH/2) * math.cos(robot_heading_rad)
    width_dy  = (ROBOT_WIDTH/2) * math.sin(robot_heading_rad)
    length_dx = (ROBOT_LENGTH/2) * math.sin(robot_heading_rad) 
    length_dy = (ROBOT_LENGTH/2) * math.cos(robot_heading_rad) 

    #calculate corner coordinates of robot, convert into pixels and transpose to staring position
    robot_x0 = int(( start_pos[0] + ( robot_pos[0]  ))* PPI )
    robot_y0 = int(( start_pos[1] - ( robot_pos[1]  ))* PPI )

    robot_x2 = int(( start_pos[0] + ( robot_pos[0] + width_dx + length_dx ))* PPI )
    robot_y2 = int(( start_pos[1] - ( robot_pos[1] + width_dy - length_dy ))* PPI )

    robot_x1 = int(( start_pos[0] + ( robot_pos[0] + width_dx - length_dx ))* PPI )
    robot_y1 = int(( start_pos[1] - ( robot_pos[1] + width_dy + length_dy ))* PPI )

    robot_x4 = int(( start_pos[0] + ( robot_pos[0] - width_dx - length_dx ))* PPI )
    robot_y4 = int(( start_pos[1] - ( robot_pos[1] - width_dy + length_dy ))* PPI )

    robot_x3 = int(( start_pos[0] + ( robot_pos[0] - width_dx + length_dx ))* PPI )
    robot_y3 = int(( start_pos[1] - ( robot_pos[1] - width_dy - length_dy ))* PPI )

    #Center of mass
    cv2.circle(img, (robot_x0,  robot_y0), 3, (0,0,255), -1 )

    #Lookahead circle
    cv2.circle(img, (robot_x0,  robot_y0), int(LOOKAHEAD_DISTANCE*PPI), (255,191,0), 1 )

    #Robot Perimeter
    cv2.drawContours(img,  [np.array([  (robot_x1, robot_y1),
                                        (robot_x2, robot_y2),
                                        (robot_x3, robot_y3),
                                        (robot_x4, robot_y4) ]).astype(np.int32)], 
                            0, (150,150,150) , 2 )

    cv2.line(img,   (robot_x0,robot_y0), (robot_x1,robot_y1),   (155,0,0),1)


    #Drive Power Lines
    drive = 20

    #Right Drive
    cv2.line(img,   (robot_x1,robot_y1),
                    ( int ( robot_x1 + drive*2*math.sin(-robot_heading_rad) ), 
                      int ( robot_y1 - drive*2*math.cos(-robot_heading_rad) )  ),
                    (0,0,255),2)

    #Left Drive
    cv2.line(img ,  (robot_x4,robot_y4),
                    ( int ( robot_x4 + drive*math.sin(-robot_heading_rad) ), 
                      int ( robot_y4 - drive*math.cos(-robot_heading_rad) )  ),
                    (0,0,255),2)

    #Place Lookahead point
    lookahead_pt = FindLookaheadPoint()
    if( lookahead_pt != None):
        cv2.circle(img, ( int((start_pos[0] + lookahead_pt[0])*PPI),  int((start_pos[1] - lookahead_pt[1])*PPI)), 3, (0,255,0), -1 )




##############
##   MAIN   ##
##############


####  Draw Results  ####
##  color = (B,G,R)

INCHES_PER_FOOT = 12
FIELD_LENGTH = (27 * INCHES_PER_FOOT)   
FIELD_WIDTH  = (27 * INCHES_PER_FOOT)   

PPI = 2.0       #Pixels per Inch scale factor

start_pos = (int(3*FIELD_WIDTH/4), int(FIELD_LENGTH-12)  )

img = np.zeros((int(FIELD_LENGTH*PPI),int(FIELD_WIDTH*PPI),3), np.uint8)


#Setup
robot_pos     = [ -12 , 24 ]

robot_heading = 0
steps = 8

for num in range(steps+1):

    img2 = img.copy()

    robot_heading = 360*num/steps

    DrawRobot(img2)             

    robot_pos[1] += 12
                                    
    cv2.imshow('image', img2)
    cv2.waitKey(0)

