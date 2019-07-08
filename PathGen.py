#Pure Pursuit Path Generator
# ** All units are in inches **
import math
import copy
import cv2
import numpy as np 

#Constants
POINTS_SPACING = 6          #inches


#User defined Waypoint
waypoints = [ (0,0),  (0,225), (-160,225),(-160,150) ]     # Raw user input list



##############
##   MAIN   ##
##############


####  Points Injection  ####
#Take pairs of user waypoints and inject points based on points spacing parameter.

path_points = [] 

for i in range( len(waypoints)-1 ):

    #calculate distance between waypoint pairs 
    wp_dist = math.sqrt(  (waypoints[i+1][0]-waypoints[i][0])**2 + (waypoints[i+1][1]-waypoints[i][1])**2 )

    #Keep a running sum of length travled on segment
    seg_dist = 0.0

    #We know we need to get to X1Y1 from X0Y0.  The intermediate points can be represented
    #  by the % of the total distance traveled in the segment (seg_dist/wp_dist)
    while(seg_dist < wp_dist ):
        xp = waypoints[i][0] + (seg_dist/wp_dist) * (waypoints[i+1][0]-waypoints[i][0])
        yp = waypoints[i][1] + (seg_dist/wp_dist) * (waypoints[i+1][1]-waypoints[i][1])
        path_points.append( [xp,yp] )
        seg_dist += float(POINTS_SPACING)

#Add final point
path_points.append( [ waypoints[-1][0],waypoints[-1][1] ] )


####  Path Smoothing  ####
# Using algorithm straight out of Pure Pursuit paper

#Constant setup
#  larger 'b' = smoother
#  a = 1-b
SMOOTHER_TOLERANCE     = 0.001 
SMOOTHER_WEIGHT_DATA   = 0.2        #'a'
SMOOTHER_WEIGHT_SMOOTH = 0.8        #'b'   where 0.75 < b < 0.98

path_profile = copy.deepcopy( path_points )       #Must do a deep copy here  [0]=X, [1]=Y

change  = SMOOTHER_TOLERANCE
while( change >= SMOOTHER_TOLERANCE):
    change = 0.0
    for i in range ( 1,len(path_points)-1 ):
        for j in range ( len(path_points[i]) ):
            aux = path_profile[i][j]
            path_profile[i][j] += SMOOTHER_WEIGHT_DATA * ( path_points[i][j] - path_profile[i][j] ) + \
                SMOOTHER_WEIGHT_SMOOTH * (path_profile[i-1][j] + path_profile[i+1][j] - (2.0 * path_profile[i][j]) )
            change += abs( aux - path_profile[i][j] )


####  Distance between points  ####     
# Using standard distance formula
# [2] = Distance
path_profile[0].append(0.0)   #Start distance at 0
for i in range( 1, len( path_profile) ):
    dist = math.sqrt( (path_profile[i][0]-path_profile[i-1][0])**2 + (path_profile[i][1]-path_profile[i-1][1])**2 )
    path_profile[i].append( path_profile[i-1][2] + dist ) 


####  Curvature  ####     
# Using algorithm straight out of Pure Pursuit paper
# [3] = Curvature
for i in range( 1, len( path_profile)-1 ):

    #Setup
    x1 = path_profile[i-1][0]
    x2 = path_profile[i+0][0]
    x3 = path_profile[i+1][0]
    y1 = path_profile[i-1][1]
    y2 = path_profile[i+0][1]
    y3 = path_profile[i+1][1]
    if( x1 == x2 ):
        x1 += 0.0001

    #Math
    k1 = 0.5 * (x1**2 + y1**2 - x2**2 - y2**2)/(x1- x2)
    k2 = (y1-y2)/(x1-x2)
    b  = 0.5 * (x2**2 - 2*x2*k1 + y2**2 - x3**2 + 2*x3*k1 - y3**2)/(x3*k2 - y3 + y2 - x2*k2)
    a  = k1 - k2*b
    r  = math.sqrt( (x1-a)**2 + (y1-b)**2 )
    
    path_profile[i].append( 1/r )

#Set the first and last
path_profile[0].append( 0.0 )
path_profile[-1].append( 0.0 )


####  Velocity Target  ####
# Use curve to generate a target velocity, greater the curve the slower we want to go     
# min( max_velocity, k/curvature)
# [4] = Velocity

MAX_VELOCITY  = 168     # inches/sec
TURN_CONSTANT = 7       # 1<x<6 = How fast to make turn?  1=slow,  6=fast

for pp in path_profile:
    if( abs(pp[3]) < 0.001):             #assume straight ahead, no curve
        pp.append(MAX_VELOCITY)
    else:
        pp.append( min( MAX_VELOCITY, TURN_CONSTANT/pp[3]) )


####  Deceleration Target ####   
#Working backwards, calculate deceleration to arrive at a turn with the proper speed   
# min( current_velocity, calculated next velocity)
# [5] = Acceleration

MAX_DECELERATION  = 200     # inches/sec^2

path_profile[-1].append(0.0)    #start with last point as zero


#Work Backwards
for i in range( len(path_profile)-2,0,-1 ):
    dist = math.sqrt( (path_profile[i+1][0]-path_profile[i][0])**2 + (path_profile[i+1][1]-path_profile[i][1])**2 )
    vf   = math.sqrt(  path_profile[i+1][5]**2 + 2*MAX_DECELERATION* dist )
    path_profile[i].append( min(path_profile[i][4], vf ) )

path_profile[0].append(MAX_VELOCITY)    #Set first point to max_veocity


####  Write to File ####
with open('output.csv',"w+") as file:
    for i,pp in enumerate(path_profile):
        #file.write( str(i) + "," )
        file.write( str(round(pp[0],1)) + "," )     # [0] = x
        file.write( str(round(pp[1],1)) + "," )     # [1] = y
        file.write( str(round(pp[2],1)) + "," )     # [2] = running distance
        file.write( str(round(pp[3],1)) + "," )     # [3] = curvature
        file.write( str(round(pp[4],1)) + "," )     # [4] = max velocity
        file.write( str(round(pp[5],1)) + "\n" )    # [5] = velocity with acceleration applied


#print( path_profile )





####  Draw Results  ####
##  color = (B,G,R)

INCHES_PER_FOOT = 12
FIELD_LENGTH = (27 * INCHES_PER_FOOT)   
FIELD_WIDTH  = (27 * INCHES_PER_FOOT)   

PPI = 2.0       #Pixels per Inch scale factor

start_pos = (int(3*FIELD_WIDTH/4), int(FIELD_LENGTH-12)  )

img = np.zeros((int(FIELD_LENGTH*PPI),int(FIELD_WIDTH*PPI),3), np.uint8)

#User Waypoints
for pt in waypoints:
    pt_adj = ( int((start_pos[0] + pt[0])*PPI ), int((start_pos[1] - pt[1])*PPI) )
    cv2.circle(img, pt_adj, 3, (0,255,255), -1 )

#Smoothed Injected points
for i in range(1,len(path_profile)):

    v_go   = int(255 * path_profile[i][5]/MAX_VELOCITY)
    v_nogo = min( 4*(255-v_go),255) 

    cv2.circle(img, ( int((start_pos[0] + path_profile[i][0])*PPI),
                      int((start_pos[1] - path_profile[i][1])*PPI) ),
                    2, (0, v_go, v_nogo), -1 )

    cv2.line(img,   ( int((start_pos[0] + path_profile[i-1][0])*PPI),
                      int((start_pos[1] - path_profile[i-1][1])*PPI) ),
                    ( int((start_pos[0] + path_profile[i][0])*PPI),
                      int((start_pos[1] - path_profile[i][1])*PPI) ),
                    (0, v_go, v_nogo), 1 )
                      
                      
cv2.imshow('image', img)
cv2.waitKey(0)

