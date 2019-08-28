import rospy
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from mavros_msgs.srv import *
from geometry_msgs.msg import TwistStamped,PoseStamped,Twist
from mavros_msgs.msg import *
import numpy as np
import math
from geodesy.utm import *

latitude =0.0
longitude=0.0

def uploadmission(data):
    waypoint_clear_client()
    wl=WaypointList()
    for point in data:
        wp=Waypoint()
        wp.frame=0#MAV_FRAME_GLOBAL
        wp.command = 16  # simple point
        wp.is_current = False
        wp.autocontinue = True
        wp.param1 = 0  # takeoff altitude
        wp.param2 = 0
        wp.param3 = 0
        wp.param4 = 0
        wp.x_lat = point[0]
        wp.y_long = point[1]
        wp.z_alt = 0
        wl.waypoints.append(wp)
    wl.waypoints[0].is_current=True
    try:
        wpPushservice = rospy.ServiceProxy('mavros/mission/push', mavros_msgs.srv.WaypointPush)
        wpPushservice(waypoints=wl.waypoints)
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e	

def waypoint_clear_client():
    try:
        wpClearservice = rospy.ServiceProxy('mavros/mission/clear', mavros_msgs.srv.WaypointClear)
        wpClearservice()
    except rospy.ServiceException,e:
        print "Service call failed: %s" % e

def setArm():
    rospy.wait_for_service('/mavros/cmd/arming')
    try:
        armService = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
        armService(True)
    except rospy.ServiceException, e:
        print "Service arm call failed: %s"%e

def setDisarm():
    rospy.wait_for_service('/mavros/cmd/arming')
    try:
        armService = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
        armService(False)
    except rospy.ServiceException, e:
        print "Service arm call failed: %s"%e

def globalPositionCallback(globalPositionCallback):
    global latitude
    global longitude
    latitude = globalPositionCallback.latitude
    longitude = globalPositionCallback.longitude
def rotate(point):
    x=cos*point[0]-sin*point[1]
    y=cos*point[1]+sin*point[0]
    rpoint=np.array([[x,y]]).reshape(1,2)
    return rpoint
def createwaypoint():
    global sin
    global cos
    num=int(input("Enter the number of boundry points:"))
    gcoord=np.zeros((0,2))
    for i in range(num):
        longitude_in=float(input("Enter boundry longitude:"))
        latitude_in=float(input("Enter boundry latitude:"))
        pt=fromLatLong(latitude_in,longitude_in)
	print pt
	point=pt.toPoint()
	print point
        cin=np.array([[point.x,point.y]]).reshape(1,2)
        gcoord=np.append(gcoord,cin,axis=0)
    gcoord = gcoord[gcoord[:,0].argsort()]
    cgcoord=gcoord-gcoord[0]
    angle=float(input("Enter the angle of the route():"))
    sin=math.sin(angle/180*math.pi)
    cos=math.cos(angle/180*math.pi)
    rcoord=np.zeros((0,2))
    for i in range(num):
        rpoint=rotate(cgcoord[i]).reshape(1,2)
        rcoord=np.append(rcoord,rpoint,axis=0)
    coord=rcoord[rcoord[:,0].argsort()]
    vertex=np.zeros((0,2))
    vertex=np.append(vertex,coord[0].reshape(1,2),axis=0)
    vertex=np.append(vertex,coord[num-1].reshape(1,2),axis=0)
    for i in range(1,num-1):
        split=(coord[i][0]-coord[0][0])/(coord[num-1][0]-coord[0][0])*(coord[num-1][1]-coord[0][1])+coord[0][1]
        if(coord[i][1]>split):
            up=True
        else:
            up=False
        if(up):
            j=0
            while(vertex[j][0]<=coord[i][0]):
                j=j+1
            vertex=np.insert(vertex, j, values=coord[i], axis=0)
        else:
            j=np.argwhere(vertex[:,0]==coord[num-1][0])[0][0]
            while(j<len(vertex)):
                if(vertex[j][0]>=coord[i][0]):
                    j=j+1
                else:
                    vertex=np.insert(vertex, j, values=coord[i], axis=0)
                    break
            if(j==len(vertex)):
                vertex=np.append(vertex, coord[i].reshape(1,2), axis=0)
    pivot=np.argwhere(vertex[:,0]==coord[num-1][0])[0][0]
    width=float(input("Enter width of the route:"))
    cutpos=np.arange(coord[0][0]+width,coord[num-1][0]+width,width)
    pointpos=np.arange(coord[0][0]+width/2,coord[num-1][0]+width/2,width)
    allvertex=vertex
    upaddnum=0
    for i in range(len(cutpos)-1):
        j=0
        k=pivot
        while(cutpos[i]>vertex[j][0]):
            j=j+1
        lefttop=vertex[j-1]
        righttop=vertex[j]
        while(k<len(vertex)):
            if(cutpos[i]<vertex[k][0]):
                k=k+1
            else:
                rightbuttom=vertex[k-1]
                leftbuttom=vertex[k]
                break
        if(k==len(vertex)):
            rightbuttom=vertex[k-1]
            leftbuttom=vertex[0]
        upperpos=[(cutpos[i],(cutpos[i]-lefttop[0])/(righttop[0]-lefttop[0])*(righttop[1]-lefttop[1])+lefttop[1])]
        lowerpos=[(cutpos[i],(cutpos[i]-leftbuttom[0])/(rightbuttom[0]-leftbuttom[0])*(rightbuttom[1]-leftbuttom[1])+leftbuttom[1])]
        allvertex=np.insert(allvertex, upaddnum+j, values=upperpos, axis=0)
        allvertex=np.insert(allvertex, upaddnum+k+1, values=lowerpos, axis=0)
        upaddnum=upaddnum+1
    pointrange=np.zeros(shape=[0,2])
    allvertex=np.append(allvertex,allvertex[0].reshape(1,2),axis=0)
    allpivot=np.argwhere(allvertex[:,0]==coord[num-1][0])[0][0]
    for i in range(len(cutpos)):
        if(i==0):
            leftupindex=0
            leftdownindex=len(allvertex)-1
        else:
            leftupindex=np.argwhere(allvertex[:,0]>=cutpos[i-1])[0][0]
            leftdownindex=np.argwhere(allvertex[allpivot:len(allvertex),0]<cutpos[i-1])[0][0]+allpivot-1
        if(i==len(cutpos)-1):
            rightupindex=allpivot
            rightdownindex=allpivot
        else:
            rightupindex=np.argwhere(allvertex[:,0]>cutpos[i])[0][0]-1
            rightdownindex=np.argwhere(allvertex[allpivot:len(allvertex),0]<cutpos[i])[0][0]+allpivot-1       
        upsearchrange=allvertex[leftupindex:rightupindex+1,:]
        downsearchrange=allvertex[rightdownindex:leftdownindex+1,:]   
        top=upsearchrange.max(0)[1]    
        buttom=downsearchrange.min(0)[1]
        pointrange=np.append(pointrange,[[top,buttom]],axis=0)
#generate waypoints
    waypoint=np.zeros(shape=[0,2])
    for i in range(len(pointpos)):
        newpoint=np.arange(pointrange[i][1]+width/2,pointrange[i][0]+width/2,width)
        newpoint=newpoint.reshape(len(newpoint),1)
	if(i%2==1):
	    newpoint=np.flipud(newpoint)
        newpoint=np.insert(newpoint,0,[pointpos[i]],axis=1)
        waypoint=np.append(waypoint,newpoint,axis=0)
#rotate back everything
    rwaypoint=np.zeros(shape=[len(waypoint),2])
    sin=math.sin(-angle/180*math.pi)
    cos=math.cos(-angle/180*math.pi)
    for i in range(len(waypoint)):
        rwaypoint[i]=rotate(waypoint[i])
    rwaypoint=rwaypoint+gcoord[0]
    gwaypoint=np.zeros(shape=[0,2])
    for i in range(len(waypoint)):
	band=pt.band
	zone=pt.zone
	upoint=UTMPoint(easting=rwaypoint[i][0],northing=rwaypoint[i][1],zone=zone,band=band)
        print upoint
    	msg=upoint.toMsg()
	print msg
	gwaypoint=np.append(gwaypoint,[[msg.latitude,msg.longitude]],axis=0)
    return gwaypoint
def menu():
    print "Press"
#    print "1: to set mode to GUIDED"
    print "1: to set mode to ARM the drone"
    print "2: to set mode to DISARM the drone"
    print "3: to go ahead"
    print "4: to print GPS coordinates"
    print "5: to set target local position"
    print "6: to set target GPS coordination"
    print "7: to create mission"
    print "8: to view mission"
def myLoop():
    x='2'
    while ((not rospy.is_shutdown())):
        menu()
        x = raw_input("Enter your input: ");
        if(x=='1'):
            setArm()
        elif(x=='2'):
            setDisarm()
        elif(x=='3'):
            go=Twist()
            go.linear.x=0.5
            velocity_pub.publish(go)
        elif(x=='4'):
            global latitude
            global longitude
            print ("latitude: %.7f" %latitude)
            print ("longitude: %.7f" %longitude)
        elif(x=="5"):
            get=PoseStamped()
            get.pose.position.x=float(raw_input("X: "))
            get.pose.position.y=float(raw_input("Y: "))
            target_pub.publish(target)
        elif(x=="6"):
            gtarget=GlobalPositionTarget()
            gtarget.latitude=float(raw_input("latitude: "))
            gtarget.longitude=float(raw_input("longitude: "))
            gtarget_pub.publish(gtarget)
        elif(x=="7"):
            waypoint=createwaypoint()
            uploadmission(waypoint)
	elif(x=="8"):
	    num=len(mission)
	    print "total waypoints: %d" %num
	    for point in mission:
		print "(%f,%f)" %(point.x_lat,point.y_long)
        else: 
            stop()
            setDisarm()
            print ("Exit")

def stop(): 
    stop = Twist() 
    stop.linear.x=0.0
    stop.linear.y=0.0
    velocity_pub.publish(stop)
def waypointCallback(waypointlist):
    global mission
    mission=waypointlist.waypoints
if __name__ == '__main__':
    rospy.init_node('mission2_node', anonymous=True)
    rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, globalPositionCallback)
    rospy.Subscriber("/mavros/mission/waypoints", mavros_msgs.msg.WaypointList, waypointCallback)
    velocity_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=10)
    target_pub=rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
    gtarget_pub=rospy.Publisher('/mavros/setpoint_raw/global',mavros_msgs.msg.GlobalPositionTarget, queue_size=10)
    myLoop()
    rospy.spin()
    
