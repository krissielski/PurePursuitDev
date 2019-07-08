#Pure Pursuit Robpt simulator
# ** All units are in inches **
import math
import cv2
import numpy as np 


LOOKAHEAD_DISTANCE = 20

ROBOT_TRACK_WIDTH  = 30     

end_of_path = False

##************************************************
## Def: Find CLosest Point
## Find closest point on path to current location
## Returns index of point
def FindClosestPoint():

    min_index = 0
    min_dist  = 100000
    for i,pp in enumerate(path_profile):
        dist = math.sqrt(  (robot_pos[0]-pp[0])**2 +  (robot_pos[1]-pp[1])**2 )
        if( dist <= min_dist):
            min_dist  = dist
            min_index = i

    return min_index


##************************************************
## Def: Find Lookahead Point
## Find points where lookout_circle intercepts robot path
## Crazy math algorithm straight from Pure Pursuit paper
## Work backwards from end point so we always know first point found is forwards
## Returns (x,y) of point
def FindLookaheadPoint():
    global end_of_path

    for i in range(len(path_profile)-2,0,-1):

        d = ( path_profile[i+1][0]-path_profile[i][0], path_profile[i+1][1]-path_profile[i][1] )    #L-E:  line segment angle vector
        f = ( path_profile[i][0]-robot_pos[0], path_profile[i][1]-robot_pos[1] )                    #E-C:  Start to Center 


        a = d[0]**2 + d[1]**2                                   # d dot d
        b = 2*( d[0]*f[0] + d[1]*f[1] )                         # 2* d dot f
        c = f[0]**2 + f[1]**2 - float(LOOKAHEAD_DISTANCE)**2    # f dot f -r^2

        dis = b**2 - 4*a*c                                      # Discriminant

        if( dis >= 0 ):

            dis = math.sqrt(dis)
            t1 = (-b - dis)/(2*a)
            t2 = (-b + dis)/(2*a)

            if( (t1>=0) and (t1<=1) ):
                if( i == (len(path_profile)-2)):
                    end_of_path = True
                #print(i,"T1 Hit!", t1)
                return( path_profile[i][0] + t1*d[0], path_profile[i][1] + t1*d[1] ) 
                
            if( (t2>=0) and (t2<=1) ):
                if( i == (len(path_profile)-2)):
                    end_of_path = True
                #print(i,"T2 Hit!",t2)
                return( path_profile[i][0] + t2*d[0], path_profile[i][1] + t2*d[1] ) 



    #If here. we failed!  Robot must have exited the path!
    return None

##************************************************
## Def: Find Curvature
## Find the arc path required to navagate the robot toward the lookahead point
## Crazy math algorithm straight from Pure Pursuit paper
## Returns signed curvature (<0 = Left Turn;  >0 = Right Turn)
def FindCurvature( aim_pt ):

    #convert degrees to radians
    #Note:  Offset angle by 90 to account for axis rotaion of the field
    robot_heading_rad = math.radians(90-robot_heading)

    a = -math.tan( robot_heading_rad )
    c = math.tan( robot_heading_rad ) * robot_pos[0] - robot_pos[1] 
    x = abs( a * aim_pt[0] + aim_pt[1] + c)/math.sqrt( a**2 + 1)

    #Side Calculation
    x_side = math.sin(robot_heading_rad)*( aim_pt[0]-robot_pos[0])
    y_side = math.cos(robot_heading_rad)*( aim_pt[1]-robot_pos[1])

    side = np.sign( x_side - y_side )

#    print("-----------")
#    print("a:",robot_heading)
#    print("x:",x_side)
#    print("y:",y_side)
#    print("s:",side)
#    print("-----------")

#   print("******")
#   print("a:",a)
#   print("c:",c)
#   print("x:",x)
#   print("s:",side)
#   print("!:",2*x/LOOKAHEAD_DISTANCE**2)
#   print("r:",1/(2*x/LOOKAHEAD_DISTANCE**2))#
#   print("-----------")


    return side * ( 2*x/LOOKAHEAD_DISTANCE**2)


##************************************************
## Def: Find Velocity
## Calculate the velocity of each wheel based on curve and nearest point
def FindVelocity(  ):
    #+curve=Right turn = higher Left drive
    #-curve=Left turn = highrt right drive
    Lv = path_profile[profile_index][5] * (2 + curve*ROBOT_TRACK_WIDTH)/2
    Rv = path_profile[profile_index][5] * (2 - curve*ROBOT_TRACK_WIDTH)/2
   
    return(Lv,Rv)




##************************************************
## Def: Draw Path
## Draws Robot Path
def DrawPath(img):
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
    rdrive = int( robot_speed[1]/5 )
    ldrive = int( robot_speed[0]/5 )

    #Right Drive
    cv2.line(img,   (robot_x1,robot_y1),
                    ( int ( robot_x1 + rdrive*math.sin(-robot_heading_rad) ), 
                      int ( robot_y1 - rdrive*math.cos(-robot_heading_rad) )  ),
                    (0,0,255),2)

    #Left Drive
    cv2.line(img ,  (robot_x4,robot_y4),
                    ( int ( robot_x4 + ldrive*math.sin(-robot_heading_rad) ), 
                      int ( robot_y4 - ldrive*math.cos(-robot_heading_rad) )  ),
                    (0,0,255),2)

    #Place Lookahead point
    lookahead_pt = FindLookaheadPoint()
    if( lookahead_pt != None):
        cv2.circle(img, ( int((start_pos[0] + lookahead_pt[0])*PPI),  int((start_pos[1] - lookahead_pt[1])*PPI)), 3, (0,255,0), -1 )

    #Add Turn Radius
    if( abs(curve) < 0.001):
        #assume straight
        #print("Straight")
        cv2.line(img,   ( int ( robot_x0 + 1000*math.sin(-robot_heading_rad)),
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
        
        cv2.circle(img, (xc,yc), int(abs((1/curve)*PPI)), (0,255,255), 1)



##############
##   MAIN   ##
##############

#Setup
robot_pos     = [ -10 , 0 ]
robot_heading = 0
robot_speed   = [0.0,0.0]
velocity      = [0.0,0.0]

#delta = 0.02
delta = 0.02
#delta = 0.005

#read in config file
path_profile = []
with open('output.csv',"r+") as file:
    for line in file.readlines():
        path_profile.append( [float(x) for x in line.split(',') ] )

#Create Field img
INCHES_PER_FOOT = 12
FIELD_LENGTH = (27 * INCHES_PER_FOOT)   
FIELD_WIDTH  = (27 * INCHES_PER_FOOT)  
PPI = 2.0       #Pixels per Inch scale factor

start_pos = (int(3*FIELD_WIDTH/4), int(FIELD_LENGTH-12)  )

img = np.zeros((int(FIELD_LENGTH*PPI),int(FIELD_WIDTH*PPI),3), np.uint8)


#Prematch setup
profile_index = FindClosestPoint()
lookahead_pt = FindLookaheadPoint()
curve = FindCurvature(lookahead_pt)

##openCV:  Draw 
img2 = img.copy()
DrawPath(img2)
DrawRobot(img2)                                              
cv2.imshow('image', img2)
#cv2.waitKey(0)


while(True):

    profile_index = FindClosestPoint()

    lookahead_pt = FindLookaheadPoint()

    curve = FindCurvature(lookahead_pt)

    velocity = FindVelocity()


    #******************

    #for i,rspeed in enumerate(robot_speed):

    ROBOT_MAX_DELTA_VEL = 500 #in/sec^2
    
    robot_speed[0] = robot_speed[0] + min(ROBOT_MAX_DELTA_VEL*delta,  max( -ROBOT_MAX_DELTA_VEL*delta, (velocity[0]-robot_speed[0]) ) )
    robot_speed[1] = robot_speed[1] + min(ROBOT_MAX_DELTA_VEL*delta,  max( -ROBOT_MAX_DELTA_VEL*delta, (velocity[1]-robot_speed[1]) ) )

    robot_pos[0] = robot_pos[0] + (robot_speed[0] + robot_speed[1])/2 * delta*math.sin( math.radians(robot_heading) )
    robot_pos[1] = robot_pos[1] + (robot_speed[0] + robot_speed[1])/2 * delta*math.cos( math.radians(robot_heading) )

    robot_heading += math.degrees( math.atan( ((robot_speed[0]-robot_speed[1])/ROBOT_TRACK_WIDTH) * delta ) ) 

    print("******************")
    print("pi:",profile_index)
    print("lk:",lookahead_pt)
    print("cur:",curve)
    print("v: ",velocity)

    print("sp:",robot_speed)

    print("pos:",robot_pos)

    print("hed:",robot_heading)

    ##openCV:  Draw 
    img2 = img.copy()
    DrawPath(img2)
    DrawRobot(img2)                                              
    cv2.imshow('image', img2)
    #cv2.waitKey(0)
    cv2.waitKey(50)

    if( end_of_path ):
        break



####  Draw Results  ####
##  color = (B,G,R)



##openCV:  Draw 
#DrawPath(img)
#DrawRobot(img)             
#
#                                   
#cv2.imshow('image', img)
#cv2.waitKey(0)

""" 

##***** ROTATION TEST *************
#Setup: Left of path
robot_pos     = [ -10 , 0 ]
robot_heading = -0

lookahead_pt = FindLookaheadPoint()

for angle in range(-90,90,+5):

   robot_heading = angle

   img2 = img.copy()

   curve = FindCurvature(lookahead_pt)

   DrawPath(img2)
   DrawRobot(img2)                      
   cv2.imshow('image', img2)
   cv2.waitKey(0)

#Setup: Right of path
robot_pos     = [ 10 , 0 ]
robot_heading = -0

lookahead_pt = FindLookaheadPoint()

for angle in range(-90,90,+5):

   robot_heading = angle

   img2 = img.copy()

   curve = FindCurvature(lookahead_pt)

   DrawPath(img2)
   DrawRobot(img2)                      
   cv2.imshow('image', img2)
   cv2.waitKey(0)

 """
