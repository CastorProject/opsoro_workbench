#!/usr/bin/env python

#import onohat hardware library
from hardware import Hardware
#import services required to interface the onohat
from opsoro_workbench.srv import SetStatusLed
from opsoro_workbench.srv import Ping
from opsoro_workbench.srv import Reset
from opsoro_workbench.srv import EnablePcaPower
from opsoro_workbench.srv import ServoCommand
from opsoro_workbench.srv import ServoBroadcastCommand
#general purpose libraries
import rospy
import time
from std_msgs.msg import Int32MultiArray

class OnohatRos(object):
	def __init__(self):
		#initialize node
		rospy.init_node("onohat_base_controller")
		#set rates
		self.rate = rospy.Rate(10) # 10hz
		#log node initialization
		rospy.loginfo("Onohat base controller started")
		#reset board
		Hardware.reset()
		#wait until reset
		time.sleep(1)
		#check board connection
		self.check_board_connection()
		#init servo interface
		self.init_servo_interface()
		#init capacitive interface
		self.initCapacitiveInterface()
		#init variables
		self.initVariables()
		#launch services
		self.set_services()
		#launch publishers and subscribers
		self.set_publishers()

	def initVariables(self):
		self.touchSensor = Int32MultiArray()

	def set_services(self):
		"""base services"""
		rospy.loginfo("setting services")
		#1. statusled service
		rospy.Service("set_status_led", SetStatusLed, self.status_led_server)
		#2  ping service
		rospy.Service("reset_onohat", Reset, self.reset_server)
		#3  reset service
		rospy.Service("ping_onohat", Ping, self.ping_server)

		"""servo services"""
		#1. pca power service
		rospy.Service("torque_enable", EnablePcaPower, self.set_pca_server)
		#2. set command service
		rospy.Service("goal_position", ServoCommand, self.set_servo_server)
		#3. broadcast command service
		rospy.Service("broadcast_servo_command", ServoBroadcastCommand, self.broadcast_servo_command)

	def set_publishers(self):
		self.pubTouchSensors = rospy.Publisher("/touchSensors", Int32MultiArray, queue_size = 10)

	def check_board_connection(self):
		with Hardware.lock:
			res = Hardware.ping()
		if res:
			rospy.loginfo("Connection succesfully established")
		else:
			rospy.logerr("cannot connect to the onohat board, check connection")
			#finish node
			rospy.signal_shutdown("failed connecting to the onohat")
		return res

	"""service request handlers"""

	def init_servo_interface(self):
		with Hardware.lock:
            		Hardware.Servo.init()

	def status_led_server(self, req):
		#turn led on
		if req.value:
			with Hardware.lock:
				Hardware.led_on()
		else:
			with Hardware.lock:
				Hardware.led_off()
		return True

	def ping_server(self, req):
		with Hardware.lock:
			res = Hardware.ping()
		return res

	def reset_server(self, req):
		with Hardware.lock:
			Hardware.reset()
		return True

	"""service request handlers for servo control"""

	def set_pca_server(self, req):
		if req.value:
			with Hardware.lock:
				Hardware.Servo.enable()
		else:
			with Hardware.lock:
				Hardware.Servo.disable()
		return True

	def set_servo_server(self, req):
		with Hardware.lock:
			Hardware.Servo.set(channel = req.id, pos=req.value)
		return True

	def broadcast_servo_command(self, req):
		with Hardware.lock:
			Hardware.Servo.set_all(pos_list = req.command_list)
		return True

	def initCapacitiveInterface(self):
		with Hardware.lock:
			Hardware.Capacitive.init(electrodes=12, gpios=0, autoconfig=True)

	"""main loop node"""
	def main_loop(self):
			while not (rospy.is_shutdown()):
				self.touchSensor.data = Hardware.Capacitive.get_filtered_data()
				self.pubTouchSensors.publish(self.touchSensor)
				self.rate.sleep()
			return

if __name__=="__main__":
	onohat = OnohatRos()
	onohat.main_loop()
