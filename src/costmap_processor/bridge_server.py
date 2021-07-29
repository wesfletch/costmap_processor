#!/usr/bin/env python

# system imports
import signal
import sys
import socket
import json

# ros imports
import rospy
from occupancy_grid_python import OccupancyGridManager
from rospy_message_converter import json_message_converter

# ros msg imports
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point32
from costmap_processor.msg import Contour as ContourMsg

import std_msgs.msg
import geometry_msgs.msg
import nav_msgs.msg
import costmap_processor.msg

HOST, PORT = "localhost", 15155
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

class BridgeServer(object):

    def __init__(self, sock=None, host=None, port=None):

        signal.signal(signal.SIGINT, self.ctrl_c_handler)

        # create socket if not provided
        if sock is None:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        else:
            self.sock = sock

        # default to localhost if <host> not provided
        if host is None:
            self.host = 'localhost'
        else:
            self.host = host

        # default to 10000 if <port> not provided
        if port is None:
            self.port = 10000
        else:
            self.port = port

        self.subscriber = None

        # dict that stores all publishers that've been configured
        # key:      topic_name (String)
        # value:    rospy.Publisher
        self._publishers = {}
        
        self.sock.bind((self.host, self.port))
        self.sock.listen(5)

        self.conn, self.addr = self.sock.accept()

    def configure_subscriber(self, topic, message_type, msg_id, callback_method=None):

        ''' configure the subscriber + callback for this BridgeServer
            a custom callback can be (optionally) configured by passing the function name '''

        # use custom callback method (if given), else use the generic self.msg_callback
        if callback_method is None:
            callback = self.echo_msg_callback
        else:
            callback = callback_method

        # if msg_id = None:

        try:
            self.subscriber = rospy.Subscriber(topic, message_type, callback, callback_args=(topic, msg_id), queue_size=10)
        except rospy.ROSException as e:
            rospy.logerr("Error configuring subscriber: {}".format(e))

    def configure_publisher(self, topic, message_type):

        ''' configure a publisher that will be used for a topic published by ROSBridge '''

        try:
            publisher = rospy.Publisher(topic, message_type, queue_size=10)
            self._publishers[topic] = publisher
        except rospy.ROSException as e:
            rospy.logerr("Error configuring publisher: {}".format(e))
        
    def send(self, msg, conn):

        if not isinstance(msg, str):
            try:
                msg = str(msg)
            except Exception as e:
                rospy.loginfo("Error: {}".format(e)) 
                return None

        try:
            conn.sendall(msg + "\r\n")
            # print(msg)
        except socket.error:
            rospy.logerr("sending error...")
            conn.shutdown(socket.SHUT_RDWR)
            conn.close()

        # return the response
        # return conn.recv(1024)

    def spin(self):

        while not rospy.is_shutdown():

            received = self.conn.recv(4096)
            received_split = received.splitlines()

            for line in received_split:
                # TODO: NEEDS ERROR CHECKING
                self.handle_message(line)

    def handle_message(self, msg):

        rospy.logdebug("RX: " + msg)

        # unpack the received JSON msg
        unpacked = json.loads(msg)
        op = unpacked['op']
        topic = unpacked['topic']
        msg_type = unpacked['type'] 
        msg_id = unpacked['id']
        msg_data = unpacked['data'] 

        
        if op == 'SUBSCRIBE':
            ''' configure subscriber that will send subscribed messages across the ROSBridge connection '''
            msg_class = self.get_class_from_type(msg_type)
            self.configure_subscriber(topic, msg_class, msg_id, self.echo_msg_callback)
        elif op == 'ADVERTISE':
            ''' advertise a topic (configure the publisher) that the ROSBridge connection can publish messages on '''
            msg_class = self.get_class_from_type(msg_type)
            self.configure_publisher(topic, msg_class)
        elif op == 'PUBLISH':
            ''' publish a message using a previously configured publisher '''
            # msg_class = self.get_class_from_type(msg_type)
            message = json_message_converter.convert_json_to_ros_message(msg_type, msg_data)
            
            if self._publishers.has_key(topic):
                self._publishers[topic].publish(message)

    def get_class_from_type(self, type_string): 

        ''' Return class of a ROS message given the _type string using import nonsense
            Assumes format "<module>/<message>"
            This is probably an ill-advised way of handling this. '''

        #                 type_string --> module/message

        parts = type_string.split('/')  # [module, message]
        module = parts[0]               # module
        m = __import__(module)          # import module
        m = getattr(m, "msg")           # module.msg  

        return getattr(m, parts[1])     # return module.msg.message

    def echo_msg_callback(self, msg, args):

        ''' when a ROS message is received (from ROS), create and send a JSON message across the ROSBridge to Java-side '''

        topic_name = args[0]
        msg_id = args[1]

        tx_msg = json_message_converter.convert_ros_message_to_json(msg)

        # create the dict that will become our JSON msg
        tx = {}
        tx["op"] = "MESSAGE"
        tx["id"] = msg_id
        tx["topic"] = topic_name
        tx["type"] = msg._type.replace(" ","_")
        tx["data"] = tx_msg

        # generate json
        tx = json.dumps(tx)

        rospy.logdebug("TX: " + tx)

        self.send(tx, self.conn)

    def ctrl_c_handler(self, sig, frame):
        self.conn.shutdown(socket.SHUT_RDWR)
        self.conn.close()
        sys.exit(0)


if __name__ == '__main__':

    rospy.init_node('msg_to_json', anonymous=True, log_level=rospy.DEBUG)

    bridge = BridgeServer(sock, HOST, PORT)

    # TODO: probably better to just spin off an IO thread for this
    bridge.spin()

    rospy.spin()