#!/usr/bin/env python

import rospy

import pid
reload(pid)

class Node():
    def __init__(self,Kp):
        self.controller = pid.Pid(Kp,1,1,1)
        self.deriv = None
    def state_callback(self,data):
        state = data.???
        out,p,i,d = self.controller.execute(dt,state,self.deriv)
        
    

if __name__ == '__main__':
    rospy.init_node('pid_node', anonymous=True)
    # Parameters
    Kp = rospy.get_param('~Kp',1.0)
    rospy.loginfo("Kp: %d"%Kp)

    # Create node
    node = Node(Kp)
    
    # Subscriptions
    state_topic = 'pid_state'
    setpoint_topic = 'pid_setpoint'
    enabled_topic = 'pid_enabled'
    rospy.Subscriber(state_topic, ,node.state_callback)




    try:
        node=Node()
    except rospy.ROSInterruptException:
        pass
