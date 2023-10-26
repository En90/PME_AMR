from firebase_bridge_class import Order
from firebase_admin import firestore
from firebase_admin import messaging
from firebase_admin import exceptions
import rospy
from std_msgs.msg import Int16MultiArray


class Order_Manage_Service:
    def __init__(self):
        pass

    def __del__(self):
        # self.OrderListener.unsubscribe()
        pass

    def InitService(self):
        try:
            self.FS = firestore.client()
            self.order_state_sub = rospy.Subscriber(
                "/order_state", Int16MultiArray, self.order_state_callback
            )
            rospy.loginfo("Init order manage service success")
        except Exception as e:
            rospy.logerr("Error when init order manage service: %s", e)

    def order_state_callback(msg):
        order_id = msg.layout.dim[0].label
        state = msg.data[0]
        robot_id = msg.data[1]
