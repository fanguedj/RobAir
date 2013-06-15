#!/usr/bin/env python
import roslib; roslib.load_manifest('robair_demo')
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalID
import ImageDraw
from PIL import Image

res=0.05
sizeM = 1000*res
mapPGM="/home/jeremy/ros_workspace/robair_demo/maps/ensimag3_new.pgm"
mapJPG=mapPGM+".jpg"
mapJPGori=mapPGM+"_ori.jpg"
xyaml = -25.4
yyaml = -7.8
rayonPx = 12

xPos=100
yPos=100
xGoal=100
yGoal=100

colorPos = (0,255,0)
colorGoal = (255,0,0)

cancel=True

def drawCircle(img, x, y, r, color):
    draw = ImageDraw.Draw(img)
    draw.ellipse((x-r, y-r, x+r, y+r), fill=color)

def calcul(x,y):
    global sizeM
    global xyaml
    global yyaml
    global res
    global rayonPx
    xM = x - xyaml
    yM = sizeM - (y -yyaml)
    xP = xM*(1/res)
    yP = yM*(1/res)
    return (xP, yP)    

def updateIMG():
    global mapJPGori
    global mapJPG
    global xPos
    global yPos
    global xGoal
    global yGoal
    global colorPos
    global colorGoal
    global cancel
    i = Image.open(mapJPGori)
    drawCircle(i,xPos,yPos,rayonPx,colorPos)
    if not cancel:
        drawCircle(i,xGoal,yGoal,rayonPx,colorGoal)
    i.save(mapJPG)

def newPose(data):
    rospy.loginfo(rospy.get_name() + ": I heard a amcl pose")
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    global xPos
    global yPos
    (xPos, yPos) = calcul(x,y)
    updateIMG()

def newOrder(data):
    rospy.loginfo(rospy.get_name() + ": I heard a goal")
    x = data.pose.position.x
    y = data.pose.position.y
    print x
    print y
    global xGoal
    global yGoal
    (xGoal, yGoal) = calcul(x,y)
    global cancel
    cancel = False
    updateIMG()

def newCancel(data):
    rospy.loginfo(rospy.get_name() + ": I heard a cancel  action")
    global cancel
    cancel = True
    updateIMG()

def listener():
    rospy.init_node('poseDisplayTablet')
    rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, newPose)
    rospy.Subscriber("move_base_simple/goal", PoseStamped, newOrder)
    rospy.Subscriber("move_base/cancel", GoalID, newCancel)

    rospy.spin()


if __name__ == '__main__':
    
    im = Image.open(mapPGM)
    im = im.convert('RGB')
    im.save(mapJPGori)

    listener()
