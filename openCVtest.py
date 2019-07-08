
import cv2
import numpy as np 
import math


print('Hello Kris')


#Create Image Space
field_length = 400
field_width  = 300 
img = np.zeros((field_length,field_width,3), np.uint8)


cv2.circle(img, (  0,  0), 10, (255,255,255), -1 )
cv2.circle(img, ( 50, 50), 10, (255,  0,  0), -1 )
cv2.circle(img, (150,400), 10, (  0,255,  0), -1 )
cv2.circle(img, (299,399), 10, (  0,  0,255), -1 )

robot_x1y1 = (100,100)
robot_x2y2 = (110,110)
robot_color = (150,150,150)

print(robot_x1y1[0])
print(robot_x1y1[0])

#cv2.rectangle(img,  (140,190), (160,210) , (150,150,150), 2 )

cv2.rectangle(img,  robot_x1y1, robot_x2y2, robot_color, 2 )


# circle( img, (x,y), radius , (b,g,r), line_width )
# rectangle( img, (x1,y1), (x2,y2), (b,g,r), line_width)

##  line_width of -1 = filled in.
##  color = (B,G,R)


cv2.imshow('image', img)
##cv2.waitKey(0)              #=wait for key,  >0 = ms wait


####*****************************************************
##ROBOT_WIDTH  = 26
##ROBOT_LENGTH = 24.5
##
##robot_pos     = ( int(field_width/2) ,  int(field_length - ROBOT_LENGTH/2) )   #Center of Robot
##robot_heading = 0
##
##
##robot_x1y1 = ( int(robot_pos[0] - (ROBOT_WIDTH/2)), int(robot_pos[1] + (ROBOT_LENGTH/2)) )
##robot_x2y2 = ( int(robot_pos[0] + (ROBOT_WIDTH/2)), int(robot_pos[1] - (ROBOT_LENGTH/2)) )
##
##cv2.rectangle(img,  robot_x1y1, robot_x2y2, robot_color, 2 )
##
##cv2.imshow('image', img)
##cv2.waitKey(0)              #=wait for key,  >0 = ms wait
##
##
##
##robot_pos = [ int(field_width/2) ,  int(field_length - ROBOT_LENGTH/2) ]
##
##for x in range(10):
##
##    img2 = img.copy()
##
##    robot_pos[1] -= 5
##
##    robot_x1y1 = ( int(robot_pos[0] - (ROBOT_WIDTH/2)), int(robot_pos[1] + (ROBOT_LENGTH/2)) )
##    robot_x2y2 = ( int(robot_pos[0] + (ROBOT_WIDTH/2)), int(robot_pos[1] - (ROBOT_LENGTH/2)) )
##
##    cv2.rectangle(img2,  robot_x1y1, robot_x2y2, robot_color, 2 )
##
##    cv2.imshow('image', img2)
##    cv2.waitKey(100)              #=wait for key,  >0 = ms wait
##
##
##

##*****************************************************
ROBOT_WIDTH  = 26
ROBOT_LENGTH = 24.5

robot_pos     = [ int(field_width/2) ,  int(field_length/2) ]

robot_heading = 0

steps = 8

for num in range(steps+1):

    img2 = img.copy()

    robot_heading = 360*num/steps

    robot_heading_rad = math.radians(robot_heading)


    #Slice length and width into x/y components and calculate corner coordinates
    #   4--1        (0,0) *---->  +x
    #   |  |              |
    #   3--2           +y | 
    width_dx  = (ROBOT_WIDTH/2) * math.cos(robot_heading_rad)
    width_dy  = (ROBOT_WIDTH/2) * math.sin(robot_heading_rad)
    length_dx = (ROBOT_LENGTH/2) * math.sin(robot_heading_rad) 
    length_dy = (ROBOT_LENGTH/2) * math.cos(robot_heading_rad) 

    robot_x1 = int( robot_pos[0] + width_dx + length_dx )
    robot_y1 = int( robot_pos[1] + width_dy - length_dy )

    robot_x2 = int( robot_pos[0] + width_dx - length_dx )
    robot_y2 = int( robot_pos[1] + width_dy + length_dy )

    robot_x3 = int( robot_pos[0] - width_dx - length_dx )
    robot_y3 = int( robot_pos[1] - width_dy + length_dy )

    robot_x4 = int( robot_pos[0] - width_dx + length_dx )
    robot_y4 = int( robot_pos[1] - width_dy - length_dy )

    print(width_dx,width_dy,length_dx,length_dy)

    print("1: (",robot_x1,",",robot_y1,")" )
    print("2: (",robot_x2,",",robot_y2,")" )
    print("3: (",robot_x3,",",robot_y3,")" )
    print("4: (",robot_x4,",",robot_y4,")" )

    #Center of mass
    cv2.circle(img2, (  robot_pos[0],  robot_pos[1]), 3, (0,0,255), -1 )

    #Robot Perimeter
    cv2.drawContours(img2,  [np.array([ (robot_x1,robot_y1),
                                        (robot_x2,robot_y2),
                                        (robot_x3,robot_y3),
                                        (robot_x4,robot_y4) ]).astype(np.int32)], 
                            0, (150,150,150) , 2 )

    #Drive Power Lines
    drive = 10

    #Right Drive
    cv2.line(img2,  (robot_x1,robot_y1),
                    ( int ( robot_x1 + drive*math.sin(robot_heading_rad) ), 
                      int ( robot_y1 - drive*math.cos(robot_heading_rad) )  ),
                    (0,0,255),2)

    #Left Drive
    cv2.line(img2,  (robot_x4,robot_y4),
                    ( int ( robot_x4 + drive*math.sin(robot_heading_rad) ), 
                      int ( robot_y4 - drive*math.cos(robot_heading_rad) )  ),
                    (0,0,255),2)



    cv2.imshow('image', img2)
    #cv2.waitKey(150)              #=wait for key,  >0 = ms wait    
    cv2.waitKey(0)              #=wait for key,  >0 = ms wait


