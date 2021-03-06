#!/usr/bin/env python

# The Keyboard Controller Node for the tutorial "Up and flying with the AR.Drone and ROS | Getting Started"
# https://github.com/mikehamer/ardrone_tutorials

# This controller extends the base DroneVideoDisplay class, adding a keypress handler to enable keyboard control of the drone

# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
import roslib; roslib.load_manifest('ardrone_tutorials')
import rospy
import time

# Load the DroneController class, which handles interactions with the drone, and the DroneVideoDisplay class, which handles video display
from drone_controller import BasicDroneController
from drone_video_display import DroneVideoDisplay
from vision import show_image
from ardrone_autonomy.msg import Navdata # for receiving navdata feedback
from nav_msgs.msg import Odometry

# We need to use resource locking to handle synchronization between GUI thread and ROS topic callbacks
from threading import Lock

# Finally the GUI libraries
from PySide import QtCore, QtGui


global roll_angle
global pitch_angle


# Here we define the keyboard map for our controller (note that python has no enums, so we use a class)
class KeyMapping(object):
	PitchForward     = QtCore.Qt.Key.Key_E
	PitchBackward    = QtCore.Qt.Key.Key_D
	RollLeft         = QtCore.Qt.Key.Key_S
	RollRight        = QtCore.Qt.Key.Key_F
	YawLeft          = QtCore.Qt.Key.Key_W
	YawRight         = QtCore.Qt.Key.Key_R
	IncreaseAltitude = QtCore.Qt.Key.Key_Q
	DecreaseAltitude = QtCore.Qt.Key.Key_A
	Takeoff          = QtCore.Qt.Key.Key_Y
	Land             = QtCore.Qt.Key.Key_H
	Emergency        = QtCore.Qt.Key.Key_Space

	HeightControl	 = QtCore.Qt.Key.Key_I
	Rotation	 = QtCore.Qt.Key.Key_O
	Square		 = QtCore.Qt.Key.Key_L
	Circle		 = QtCore.Qt.Key.Key_C


# Our controller definition, note that we extend the DroneVideoDisplay class
class KeyboardController(DroneVideoDisplay):
	def __init__(self):
		super(KeyboardController,self).__init__()

		# Subscribe to the /ardrone/navdata topic, of message type navdata, and call self.ReceiveNavdata when a message is received
		self.subNavdata2 = rospy.Subscriber('/ardrone/navdata',Navdata,self.ReceiveNavdata2) 
		
		self.pitch = 0
		self.roll = 0
		self.yaw_velocity = 0 
		self.z_velocity = 0

	def ReceiveNavdata2(self,navdata):
		# Indicate that new data has been received (thus we are connected)
		self.communicationSinceTimer = True

		# Name the navdata variable to be used as a global variable
		if (navdata.rotZ < 0):
			yaw_angle = 360 + navdata.rotZ
		else:
			yaw_angle = navdata.rotZ

		global yaw_angle

		height = int(navdata.altd)
		global height

# We add a keyboard handler to the DroneVideoDisplay to react to keypresses
	def keyPressEvent(self, event):
		key = event.key()
		def Rotate90():
			initial_angle = yaw_angle
			if (yaw_angle< 270 and yaw_angle>= 0):
				giro = 0
				self.yaw_velocity += -1
				while(giro<90):
					giro = yaw_angle - initial_angle
					controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)
				final_angle = yaw_angle				
				self.yaw_velocity = 0
				controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)
			elif (yaw_angle >= 270):
				self.yaw_velocity -= 1
				while(yaw_angle < (360)):
					controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)

				while(yaw_angle < (initial_angle-270)):
					controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)
				self.yaw_velocity = 0
				controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)
					
				final_angle = yaw_angle	
			else:
				self.yaw_velocity = 0
				controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)
				final_angle = yaw_angle	
			global final_angle
			
		
		def Forward(duration):
			self.pitch += 1
			controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)
			a= time.time()
			tiempo = 0
			while (tiempo < duration):
				b = time.time(duration)
				tiempo = b-a
				# if (yaw_angle < final_angle):
				#	self.yaw_velocity += 1
				#elif (yaw_angle > final_angle):
				#	self.yaw_velocity += -1
				#else: 
				#	self.yaw_velocity = 0
				controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)

		# If we have constructed the drone controller and the key is not generated from an auto-repeating key
		if controller is not None and not event.isAutoRepeat():
			# Handle the important cases first!
			if key == KeyMapping.Emergency:
				controller.SendEmergency()
			elif key == KeyMapping.Takeoff:
				controller.SendTakeoff()
			elif key == KeyMapping.Land:
				controller.SendLand()
			else:
				# Now we handle moving, notice that this section is the opposite (+=) of the keyrelease section
				if key == KeyMapping.YawLeft:
					self.yaw_velocity += 1
				elif key == KeyMapping.YawRight:
					self.yaw_velocity += -1

				elif key == KeyMapping.PitchForward:
					self.pitch += 1
				elif key == KeyMapping.PitchBackward:
					self.pitch += -1

				elif key == KeyMapping.RollLeft:
					self.roll += 1
				elif key == KeyMapping.RollRight:
					self.roll += -1

				elif key == KeyMapping.IncreaseAltitude:
					self.z_velocity += 1
				elif key == KeyMapping.DecreaseAltitude:
					self.z_velocity += -1

				elif key == KeyMapping.HeightControl:
					controller.SendTakeoff()
					time.sleep(5)
					while(height<750):
						self.z_velocity += 1
						controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)
					
					self.z_velocity += -1
					controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)		
					
					controller.SendLand()

				elif key == KeyMapping.Rotation:
					self.yaw_velocity += 1
					while(yaw_angle<100):
						controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)
					self.yaw_velocity -=1
					controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)

				elif key == KeyMapping.Square:
					self.pitch += 1
					controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)
					time.sleep(1.5)
					self.pitch = 0
					controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)
					time.sleep(1)
					
					self.roll += 1
					controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)
					time.sleep(1.5)
					self.roll = 0
					controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)
					time.sleep(1)

					self.pitch -= 1
					controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)
					time.sleep(1.5)
					self.pitch = 0
					controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)
					time.sleep(1)
					
					self.roll -= 1
					controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)
					time.sleep(1.5)
					self.roll = 0
					controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)
					time.sleep(1)

				elif key == KeyMapping.Circle:
					a= time.time()
					self.pitch = 1
					self.yaw_velocity = -1
					controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)
					tiempo = 0
					while (tiempo < 8.5):
						b = time.time()
						tiempo = b-a
					self.pitch = 0
					self.yaw_velocity = 0
					controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)
					
									

			# finally we set the command to be sent. The controller handles sending this at regular intervals
			controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)


	def keyReleaseEvent(self,event):
		key = event.key()

		# If we have constructed the drone controller and the key is not generated from an auto-repeating key
		if controller is not None and not event.isAutoRepeat():
			# Note that we don't handle the release of emergency/takeoff/landing keys here, there is no need.
			# Now we handle moving, notice that this section is the opposite (-=) of the keypress section
			if key == KeyMapping.YawLeft:
				self.yaw_velocity -= 1
			elif key == KeyMapping.YawRight:
				self.yaw_velocity -= -1

			elif key == KeyMapping.PitchForward:
				self.pitch -= 1
			elif key == KeyMapping.PitchBackward:
				self.pitch -= -1

			elif key == KeyMapping.RollLeft:
				self.roll -= 1
			elif key == KeyMapping.RollRight:
				self.roll -= -1

			elif key == KeyMapping.IncreaseAltitude:
				self.z_velocity -= 1
			elif key == KeyMapping.DecreaseAltitude:
				self.z_velocity -= -1

			# finally we set the command to be sent. The controller handles sending this at regular intervals
			controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)



# Setup the application
if __name__=='__main__':
	
	import sys
	# Firstly we setup a ros node, so that we can communicate with the other packages
	rospy.init_node('ardrone_keyboard_controller')
	
	# Now we construct our Qt Application and associated controllers and windows
	app = QtGui.QApplication(sys.argv)
	controller = BasicDroneController()
	display = KeyboardController()
	
	display.show()
	show_image()
	# executes the QT application
	status = app.exec_()
	
	# and only progresses to here once the application has been shutdown
	rospy.signal_shutdown('Great Flying!')
	sys.exit(status)

