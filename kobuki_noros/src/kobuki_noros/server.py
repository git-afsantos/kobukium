
import threading
import socket

import rospy
from geometry_msgs.msg import Twist

MAX_LINEAR = 0.375  # m/s
MAX_ANGULAR = 1.9   # rad/s

###############################################################################
#   Client Thread
###############################################################################

class ClientThread(threading.Thread):
    def __init__(self, client_socket):
        threading.Thread.__init__(self)
        self._stop_event = threading.Event()
        self.client = client_socket
        self.cmd_vel_pub = None
        self.linear_vel = 0
        self.angular_vel = 0

    def stop(self):
        self._stop_event.set()

    def stopped(self):
        return self._stop_event.is_set()

    def run(self):
        try:
            pub_name = self.client.recv(4096)
            if not self.is_valid_name(pub_name):
                print "Received invalid name:", pub_name
                self.client.sendall("NOK")
                return False
            self.client.sendall("OK")
            rate = rospy.Rate(1) # 1hz
            self.client.settimeout(1)
            self.cmd_vel_pub = rospy.Publisher(pub_name, Twist, queue_size = 10)
            while not rospy.is_shutdown() and not self.stopped():
                try:
                    command = self.client.recv(4096)
                    if not command:
                        return False
                    self.parse_command(command)
                except socket.timeout as e:
                    pass
                self.publish_command()
                rate.sleep()
            return True
        finally:
            self.client.close()

    def publish_command(self):
        vel_msg = Twist()
        vel_msg.linear.x = self.linear_vel
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = self.angular_vel
        self.cmd_vel_pub.publish(vel_msg)

    def is_valid_name(self, name):
        if not name:
            return False
        for c in name:
            if not c.isalnum() and not c == "/" and not c == "_":
                return False
        return True

    def parse_command(self, command):
        tokens = command.split()
        if len(tokens) == 2:
            try:
                vx = float(tokens[0])
                wz = float(tokens[1])
                if (vx < -MAX_LINEAR or vx > MAX_LINEAR
                        or wz < -MAX_ANGULAR or wz > MAX_ANGULAR):
                    self.client.sendall("NOK")
                else:
                    self.linear_vel = vx
                    self.angular_vel = wz
                    self.client.sendall("OK")
            except ValueError as e:
                self.client.sendall("NOK")
        else:
            self.client.sendall("NOK")


###############################################################################
#   Server
###############################################################################

class Server(object):
    def __init__(self):
        self.clients = []
        self.server = None

    def run(self):
        print "[NoROS] Starting the server."
        rospy.init_node("noros")
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.bind((socket.gethostname(), 8421))
        self.server.listen(5)
        print "[NoROS] Listening for connections."
        try:
            while not rospy.is_shutdown():
                (client, addr) = self.server.accept()
                print "[NoROS] A client connected."
                c = ClientThread(client)
                self.clients.append(c)
                c.start()
        finally:
            self.stop()

    def stop(self):
        print "[NoROS] Shutting down the server."
        self.server.close()
        for client in self.clients:
            client.stop()
            client.join()
        self.clients = []
        self.server = None
