#!/usr/bin/env python

import datetime
import threading
import os

import roslib
roslib.load_manifest('robair_demo')
import rospy

from geventwebsocket.handler import WebSocketHandler
from gevent.pywsgi import WSGIServer
from flask import Flask, request, render_template,session, redirect

from robair_demo.msg import Command
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point


class TabletNode(threading.Thread):
    '''Robair Tablet node'''
    def __init__(self, topic='/cmd', node_name="tablet"):
        self.node_name = node_name
        self.pub = rospy.Publisher(topic, Command)
        self.pubPos = rospy.Publisher("/goal_from_tablet", Pose)
        rospy.init_node('tablet')


    def key_pressed(self, key):
        if (not rospy.is_shutdown()):
            directions = {"top": 0,
                          "bottom": 1,
                          "left": 2,
                          "right": 3,
                          "s": 4}
        if key in directions.keys():
            print("%s" % key)
            self.pub.publish(Command(directions[key],0.0,0.0))    
        if "st" in key:
            val = key.split("_")
            print("commande : %s" % key)
            speed = float(val[1])
            turn = float(val[2])
            self.pub.publish(Command(5,speed,turn))
        if "pos" in key:
            val = key.split("_")
            print("position d'arrivee : %s" % key)
            p = Pose()
            new_position = Point()
            new_position.x = float(val[1])
            new_position.y = float(val[2])
            p.position = new_position
            self.pubPos.publish(p)


tablet_node = TabletNode()
app = Flask(__name__)
app.secret_key = 'blablabla'

@app.route('/')
def reserver():
    return render_template('reservation.html')

@app.route('/commande')
def index():
    return render_template('navigation.html')

@app.route('/commande/api')
def goApi():
    api();
    
@app.route('/api')
def api():
    if request.environ.get('wsgi.websocket'):
        ws = request.environ['wsgi.websocket']
        while True:
            message = ws.receive()
            tablet_node.key_pressed(message)
            ws.send(message)
    return

if __name__ == '__main__':
    ip = os.popen("bash ~/ros_workspace/robair_demo/utils/getLocalIP.sh", "r").read()
    ip=ip.replace('\n','')
    address = ip, 4243
    #address = "127.0.0.1", 4243
    http_server = WSGIServer(address, app, handler_class=WebSocketHandler)
    rospy.loginfo("%s running..." % tablet_node.node_name)
    print("Server running on http://%s:%d. Ctrl+C to quit" % address)
    http_server.serve_forever()
    
