#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
import sys, select, os

if os.name == 'nt':
	import msvcrt
else:
	import tty, termios

msg = """
Control Your AGV!
---------------------------
Moving around:
        w
   a         d
        x

CTRL-C to quit
"""

e = """
Communications Failed
"""

def getKey():
	if os.name == 'nt':
		return msvcrt.getch()

	tty.setraw(sys.stdin.fileno())
	rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
	if rlist:
		key = sys.stdin.read(1)
	else:
		key = ''

	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

if __name__=="__main__":
	if os.name != 'nt':
		settings = termios.tcgetattr(sys.stdin)

	boolF = Bool()
	boolB = Bool()
	boolL = Bool()
	boolR = Bool()

	rospy.init_node('relay_teleop')
	pubF = rospy.Publisher('/relayF', Bool, queue_size=10)
	pubB = rospy.Publisher('/relayB', Bool, queue_size=10)
	pubL = rospy.Publisher('/relayL', Bool, queue_size=10)
	pubR = rospy.Publisher('/relayR', Bool, queue_size=10)

	status = 0

	try:
		print(msg)
		while(1):
			key = getKey()
			if key == 'w' :
				boolF.data = True
				pubF.publish(boolF)
				print("Keypressed: %s" % key)
			elif key == 'x' :
				boolB.data = True
				pubB.publish(boolB)
				print("Keypressed: %s" % key)
			elif key == 'a' :
				boolL.data = True
				pubL.publish(boolL)
				print("Keypressed: %s" % key)
			elif key == 'd' :
				boolR.data = True
				pubR.publish(boolR)
				print("Keypressed: %s" % key)
			else:
				if (key == '\x03'):
					break
				else:
					boolF.data = False
					boolB.data = False
					boolL.data = False
					boolR.data = False

					pubF.publish(boolF)
					pubB.publish(boolB)
					pubL.publish(boolL)
					pubR.publish(boolR)

	except:
		print(e)

	finally:
		boolF.data = False
		boolB.data = False
		boolL.data = False
		boolR.data = False

		pubF.publish(boolF)
		pubB.publish(boolB)
		pubL.publish(boolL)
		pubR.publish(boolR)

	if os.name != 'nt':
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
