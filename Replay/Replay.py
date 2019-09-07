# Replay 
# ** All units are in inches **
import math
import cv2
import numpy as np 


##************************************************
## Def: Draw Profile Path
## Draws Robot Intended Path from profile generator
def DrawProfilePath(img):
    for i in range(1,len(path_profile)):

        cv2.circle(img, ( int((start_pos[0] + path_profile[i][0])*PPI),
                        int((start_pos[1] - path_profile[i][1])*PPI) ),
                        2, (255,0,0), -1 )

        cv2.line(img,   ( int((start_pos[0] + path_profile[i-1][0])*PPI),
                        int((start_pos[1] - path_profile[i-1][1])*PPI) ),
                        ( int((start_pos[0] + path_profile[i][0])*PPI),
                        int((start_pos[1] - path_profile[i][1])*PPI) ),
                        (255,0,0), 1 )


##************************************************
## Def: Draw Robot Path
## Draws actual Robot path from generated pp_ debug file
def DrawRobotPath(img):
    for i in range(1,len(robot_data)):

        # cv2.circle(img, ( int((start_pos[0] + robot_data[i][1])*PPI),
        #                 int((start_pos[1] - robot_data[i][2])*PPI) ),
        #                 2, (255,0,0), -1 )

        cv2.line(img,   ( int((start_pos[0] + robot_data[i-1][1])*PPI),
                        int((start_pos[1] - robot_data[i-1][2])*PPI) ),
                        ( int((start_pos[0] + robot_data[i][1])*PPI),
                        int((start_pos[1] - robot_data[i][2])*PPI) ),
                        (125,0,150), 1 )



def OnTrackbar(val):

    img2 = img.copy()
    DrawProfilePath(img2)
    DrawRobotPath(img2)                             

    ROBOT_WIDTH   = 26
    ROBOT_LENGTH  = 24.5
    LOOKAHEAD_DISTANCE = 20

    robot_pos     = [ robot_data[val][1],robot_data[val][2] ]
    robot_heading =  robot_data[val][3]

    ##-----------------------------------------------
    robot_heading_rad = math.radians(-robot_heading)

    #Slice length and width into x/y components and calculate corner coordinates in inches
    #   4---1        (0,0) *---->  +x            0
    #   | 0 |              |                -90 -+- 90
    #   3---2           +y |                    180
    width_dx  = (ROBOT_WIDTH/2)  * math.cos(robot_heading_rad)
    width_dy  = (ROBOT_WIDTH/2)  * math.sin(robot_heading_rad)
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
    cv2.circle(img2, (robot_x0,  robot_y0), 3, (0,0,255), -1 )

    #Lookahead circle
    cv2.circle(img2, (robot_x0,  robot_y0), int(LOOKAHEAD_DISTANCE*PPI), (255,191,0), 1 )

    #Robot Perimeter
    cv2.drawContours(img2,  [np.array([ (robot_x1, robot_y1),
                                        (robot_x2, robot_y2),
                                        (robot_x3, robot_y3),
                                        (robot_x4, robot_y4) ]).astype(np.int32)], 
                            0, (150,150,150) , 2 )

    cv2.line(img2,   (robot_x0,robot_y0), (robot_x1,robot_y1),   (155,0,0),1)


    #Place Lookahead point
    lookahead_pt = [ robot_data[val][7],robot_data[val][8] ]

    cv2.circle(img2, ( int((start_pos[0] + lookahead_pt[0])*PPI),  int((start_pos[1] - lookahead_pt[1])*PPI)), 3, (0,255,0), -1 )


    #Add Turn Radius
    curve =  robot_data[val][9]

    if( abs(curve) < 0.001):
        #straight
        cv2.line(img2,  ( int ( robot_x0 + 1000*math.sin(-robot_heading_rad)),
                          int ( robot_y0 - 1000*math.cos(-robot_heading_rad)) ), 
                        ( int ( robot_x0 - 1000*math.sin(-robot_heading_rad)),
                          int ( robot_y0 + 1000*math.cos(-robot_heading_rad)) ), 
                     (0,255,255),1)
    else:
        if( np.sign(curve)> 0.0):
            #Turn to the right
            #print("Right",abs(1/curve))
            xc = int( robot_x0 + math.cos(robot_heading_rad)*abs(1/curve)*PPI ) 
            yc = int( robot_y0 - math.sin(robot_heading_rad)*abs(1/curve)*PPI ) 
        else:
            #Turn to the left
            #print("Left",abs(1/curve))
            xc = int( robot_x0 - math.cos(robot_heading_rad)*abs(1/curve)*PPI ) 
            yc = int( robot_y0 + math.sin(robot_heading_rad)*abs(1/curve)*PPI )      
        
        cv2.circle(img2, (xc,yc), int(abs((1/curve)*PPI)), (0,255,255), 1)



    #Drive Power Lines
    PPMD = 30       #Pixels per max drive (100%)
    ldrive = int( robot_data[val][12]*PPMD )
    rdrive = int( robot_data[val][13]*PPMD )

    #Right Drive
    cv2.line(img2,  (robot_x1,robot_y1),
                    ( int ( robot_x1 + rdrive*math.sin(-robot_heading_rad) ), 
                      int ( robot_y1 - rdrive*math.cos(-robot_heading_rad) )  ),
                    (0,0,255),2)

    cv2.circle(img2,
                    ( int ( robot_x1 + PPMD*math.sin(-robot_heading_rad) ), 
                      int ( robot_y1 - PPMD*math.cos(-robot_heading_rad) )  ),
                    1, (0,255,255), 1)                    

    #Left Drive
    cv2.line(img2,  (robot_x4,robot_y4),
                    ( int ( robot_x4 + ldrive*math.sin(-robot_heading_rad) ), 
                      int ( robot_y4 - ldrive*math.cos(-robot_heading_rad) )  ),
                    (0,0,255),2)

    cv2.circle(img2,
                    ( int ( robot_x4 + PPMD*math.sin(-robot_heading_rad) ), 
                      int ( robot_y4 - PPMD*math.cos(-robot_heading_rad) )  ),
                    1, (0,255,255), 1)    

    cv2.imshow('image', img2)




##############
##   MAIN   ##
##############


#read in path profile
path_profile = []
with open('output.csv',"r") as file:
    for i,line in enumerate( file.readlines() ):
        if( i>0 ):
            #Skip first line (Profile name & date)
            path_profile.append( [float(x) for x in line.split(',') ] )


#read in robot data
robot_data = []
with open('pp.csv',"r") as file:
    for i,line in enumerate(file.readlines()):
        robot_data.append( [float(x) for x in line.split(',') ] )



#Create Field img
INCHES_PER_FOOT = 12
FIELD_LENGTH = (27 * INCHES_PER_FOOT)   
FIELD_WIDTH  = (27 * INCHES_PER_FOOT)  
PPI = 2.0       #Pixels per Inch scale factor

start_pos = (int(FIELD_WIDTH/2), int(FIELD_LENGTH-12)  )

img = np.zeros((int(FIELD_LENGTH*PPI),int(FIELD_WIDTH*PPI),3), np.uint8)




##openCV:  Draw 
img2 = img.copy()
DrawProfilePath(img2)
DrawRobotPath(img2)                             



cv2.imshow('image', img2)


cv2.createTrackbar("Position", 'image' , 1, len(robot_data)-1, OnTrackbar)

cv2.waitKey(0)
