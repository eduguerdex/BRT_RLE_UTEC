#!/usr/bin/env python
#

from __future__ import print_function
import rospy
from sensor_msgs.msg import JointState

from markers import *
from BraccioDEV import *

# Initialize the node
rospy.init_node("testKineControlPosition")
print('starting motion ... ')
# Publisher: publish to the joint_states topic
pub = rospy.Publisher('joint_states', JointState, queue_size=10)
# Files for the logs
fxcurrent = open("/tmp/xcurrent.txt", "w")                
fxdesired = open("/tmp/xdesired.txt", "w")
fq = open("/tmp/q.txt", "w")

# Markers for the current and desired positions
bmarker_current  = BallMarker(color['RED'])
bmarker_desired = BallMarker(color['GREEN'])

# Joint names
jnames = ['base_joint','shoulder_joint', 'elbow_joint', 'wrist_pitch_joint','wrist_roll_joint']

# Desired position     [-0.068, 0.086, 0.11]

xd = np.array([0.17,0.124, 0.11])
# Initial configuration
q0 = np.array([1.57,3.14,0,1.57,1.57])

# Resulting initial position (end effector with respect to the base link)
T = fkine_BRT(q0)
x0 = T[0:3,3]

# Red marker shows the achieved position
bmarker_current.xyz(x0)
# Green marker shows the desired position
bmarker_desired.xyz(xd)

# Instance of the JointState message
jstate = JointState()
# Values of the message
jstate.header.stamp = rospy.Time.now()
jstate.name = jnames
# Add the head joint value (with value 0) to the joints
jstate.position = q0

# Frequency (in Hz) and control period 
freq = 200
dt = 1.0/freq
rate = rospy.Rate(freq)

# Initial joint configuration
q = copy(q0)
# Main loop
while not rospy.is_shutdown():
    # Current time (needed for ROS)
    jstate.header.stamp = rospy.Time.now()
    # Kinematic control law for position (complete here)
    # -----------------------------
    epsilon = np.array([0.01, 0.01, 0.01])
    k = 0.5
    Td = fkine_BRT(q)
    # Posicion
    x = Td[0:3, 3]
    # error
    e = (x - xd)
    # Ley de control
    de = -k*e
    # Jacobiano
    J = jacobian_BRT(q,delta = 0.01)
    dq = np.linalg.pinv(J).dot(de)
    # Integracion
    q = q + dt*dq


    # -----------------------------

    # Log values                                                      
    fxcurrent.write(str(x[0])+' '+str(x[1]) +' '+str(x[2])+'\n')
    fxdesired.write(str(xd[0])+' '+str(xd[1])+' '+str(xd[2])+'\n')
    fq.write(str(q[0])+" "+str(q[1])+" "+str(q[2])+" "+str(q[3])+" "+
             str(q[4])+"\n")
    
    # Publish the message
    jstate.position = q
    pub.publish(jstate)
    bmarker_desired.xyz(xd)
    bmarker_current.xyz(x)
    # Wait for the next iteration
    rate.sleep()

print('ending motion ...')
print("Posicion deseada")
print(xd)
print("Posicion y orientacion del efector final")
print(np.round(Td,3))
print("Configuracion articular")
print(np.round(q,2))
fxcurrent.close()
fxdesired.close()
fq.close()
