#!/usr/bin/python3
import rospy
from std_msgs.msg import Bool
import rosnode

def check_nodes(nodes_to_check):
    # Get the list of currently active nodes
    active_nodes = rosnode.get_node_names()
    active_nodes_status = all(node in active_nodes for node in nodes_to_check)
    return active_nodes_status

def status_publisher():
    # Initialize the node
    rospy.init_node('robody_ros1_status_checker', anonymous=True)
    
    # Create a publisher object
    init_pub = rospy.Publisher('/roboy/pinky/status/initialized', Bool, queue_size=1)
    nodes_pub = rospy.Publisher('/roboy/pinky/status/nodes/up', Bool, queue_size=1)

    init_msg = Bool()
    node_msg = Bool()
    rate = rospy.Rate(1)  # 1 Hz
    rospy.loginfo("Started publishing robody ROS1 status") 
    while not rospy.is_shutdown():
        # Check the "initialized" parameter
        initialized = rospy.get_param('/initialized', {})
        if initialized:
            init_msg.data = initialized["head"] and initialized["shoulder_left"] and initialized ["shoulder_right"]
        else:
            init_msg.data = False
        
        # Specify the nodes to check for
        nodes_to_check = ['/rosserial_node_usb0', '/assisted_drive_controller', 
                               '/bullet_joints', '/pinky_upper_body', '/roboy_fpga_00_00_f3_be_ef_02']
        
        # Check if specified nodes are active
        node_msg.data = check_nodes(nodes_to_check)
        
        init_pub.publish(init_msg)
        nodes_pub.publish(node_msg)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        status_publisher()
    except rospy.ROSInterruptException:
        pass
