#!/usr/bin/env python

''' acts as the Python side of a Python <--> Java socket connection 
    that allows a Java (or any language with sockets, tbh) process to publish/subscribe to ROS messages '''

# system imports
import signal
import sys
import socket, errno
import json

# ros imports
import rospy
from occupancy_grid_python import OccupancyGridManager
from rospy_message_converter import json_message_converter

# ros msg imports
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point32
from costmap_processor.msg import Contour as ContourMsg

# ros msg imports
import std_msgs.msg
import geometry_msgs.msg
import nav_msgs.msg
import costmap_processor.msg

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

        # used to tell our BridgeServer to stop spinning if we run into transport errors
        self._kill = None

        # dict that stores all publishers that've been configured
        # key:      topic_name (String)
        # value:    rospy.Publisher
        self._publishers = {}
        
        self.sock.bind((self.host, self.port))
        self.sock.listen(5)

        # TODO: if BridgeServer is meant to handle multiple connections, this needs to change
        self.conn, self.addr = self.sock.accept()

    def configure_subscriber(self, topic, message_type, msg_id, callback_method=None):

        ''' configure the subscriber + callback for this BridgeServer
            a custom callback can be (optionally) configured by passing the function name '''

        # use custom callback method (if given), else use the generic self.echo_msg_callback
        if callback_method is None:
            callback = self.echo_msg_callback
        else:
            callback = callback_method

        # create the callback, providing additional callback args (the topic name and msg_id)
        try:
            self.subscriber = rospy.Subscriber(topic, message_type, callback, callback_args=(topic, msg_id), queue_size=10)
        except rospy.ROSException as e:
            rospy.logerr("Error configuring subscriber: {}".format(e))

    def configure_publisher(self, topic, message_type):

        ''' configure a publisher that will be used for a topic published by ROSBridge '''

        try:
            # creating the publisher
            publisher = rospy.Publisher(topic, message_type, queue_size=10)
            # adding to dict so we can access it later by topic name
            self._publishers[topic] = publisher
        except rospy.ROSException as e:
            rospy.logerr("Error configuring publisher: {}".format(e))
        
    def send(self, msg, conn):

        if self._kill:
            return

        if not isinstance(msg, str):
            try:
                msg = str(msg)
            except Exception as e:
                rospy.loginfo("Error casting input msg to string: {}".format(e)) 
                return None

        try:
            conn.sendall(msg + "\r\n")
        # an absurd amount of error checking
        except socket.error as serr:
            rospy.logerr("Socket error... {}".format(serr))
            if serr.errno == 32:
                self._kill = True
        except error as err:
            # transport endpoint not connected --> 
            rospy.logerr("sending error... {}".format(err))
            if err.errno == 107:
                self._kill = True

    def spin(self, rate):

        ''' main I/O loop of BridgeServer; sends/receives messages on the socket connection at specified rospy.Rate
            acts as analogue to rospy.spin()
            input: rospy.Rate rate as a frequency (Hz) '''

        while not rospy.is_shutdown():

            if self._kill:
                break
            
            received = ""
            try:
                # JSON messages are delineated by { and }
                # do our best to receive a full message, but if we don't, keep going
                while not "}\n" in received:
                    received += self.conn.recv(32768)   # 32768 is the max amount of bytes we try to receive

                # split data by newline (\n), each item in received_split should be a complete message
                # mostly here to handle the possibility of multiple messages in one transmission
                received_split = received.splitlines()

            except socket.error as err:
                # connection reset --> just end the bridge_server loop
                if err.errno == errno.ECONNRESET:
                    break

            # handle each message in received_split (most of the time it's just 1 message)
            for line in received_split:
                self.handle_message(line)

            rate.sleep()

    def handle_message(self, msg):

        ''' handles incoming messages, routing them to appropriate functions to publish, subscribe, advertise... '''

        # unpack the received JSON msg's required fields
        try:
            unpacked = json.loads(msg)
        except ValueError as e:
            rospy.logwarn("Issue unpacking json message.")
            rospy.logwarn("the offending message: {}".format(msg))
            print(e)
            return False

        # message fields
        op = unpacked['op']
        topic = unpacked['topic']
        msg_type = unpacked['type'] 
        msg_id = unpacked['id']
        msg_data = unpacked['data'] 

        if op == 'SUBSCRIBE':
            ''' configure a subscriber that upon callback will send messages back across the ROSBridge connection '''
            msg_class = self.get_class_from_type(msg_type)
            self.configure_subscriber(topic, msg_class, msg_id, self.echo_msg_callback)
        elif op == 'ADVERTISE':
            ''' advertise a topic (configure the publisher) that the ROSBridge connection can publish messages from the Java client '''
            msg_class = self.get_class_from_type(msg_type)
            self.configure_publisher(topic, msg_class)
        elif op == 'PUBLISH':
            ''' publish a message from the Java client using a previously configured publisher '''
            try:
                message = json_message_converter.convert_json_to_ros_message(msg_type, msg_data)
            except AttributeError as err:
                print(err)
                return False

            if self._publishers.has_key(topic):
                self._publishers[topic].publish(message)
            else:
                rospy.logerr("No publisher configured for {}. \
                    Make sure that the Java client is sending an ADVERTISE message first.".format(topic))
        elif op == 'HEARTBEAT':
            ''' HEARTBEAT messages just tell us the connection is still active to prevent timeouts. Nothing needs to be done.'''
            pass
        

    def get_class_from_type(self, type_string): 

        ''' Return class of a ROS message given the _type string using __import__ tomfoolery
            Assumes format "<module>/<message>"
            This is probably an ill-advised way of handling this. '''

        #                 type_string --> module/message

        parts = type_string.split('/')  # [module, message]
        module = parts[0]               # module
        m = __import__(module)          # import module
        m = getattr(m, "msg")           # module.msg  

        return getattr(m, parts[1])     # return module.msg.message

    def echo_msg_callback(self, msg, args):

        ''' when a ROS message is received (from ROS) on a topic that ROSBridge is subscribed to,
            create and send a JSON message across the ROSBridge to Java-side '''

        if self._kill:
            return

        # unpacking the callback args (defined in self.configure_subscriber())
        topic_name = args[0]
        msg_id = args[1]

        # convert message to JSON
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

        self.send(tx, self.conn)

    def ctrl_c_handler(self, sig, frame):
        self.conn.shutdown(socket.SHUT_RDWR)
        self.conn.close()
        sys.exit(0)


if __name__ == '__main__':

    rospy.init_node('bridge_server', anonymous=True, log_level=rospy.DEBUG)

    HOST, PORT = "localhost", 15158
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    bridge = BridgeServer(sock, HOST, PORT)

    rate = rospy.Rate(20)

    # BridgeServer.spin acts as rospy.spin() in this case
    bridge.spin(rate)

