from firebase_bridge_class import Order
from firebase_admin import firestore
from firebase_admin import messaging
from firebase_admin import exceptions
import rospy
from std_msgs.msg import Int16MultiArray


class Order_Manage_Service:
    def __init__(self, app, confirmed: dict):
        self.confirmed = confirmed
        self.app = app
        self.InitService()
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

    def order_state_callback(self, msg):
        order_id: str = msg.layout.dim[0].label
        state: int = msg.data[0]
        robot_id: int = msg.data[1]
        if state == 1:
            order: Order = self.confirmed[order_id]
            order.state = 1
            self.send_order_state(
                self.init_pl_msg(
                    order_id,
                    robot_id,
                    order.recipient,
                    order.sender,
                    order.recipient_location,
                    order.sender_location,
                )
            )
        elif state == 2:
            order: Order = self.confirmed["order_id"]
            order.state = 2
            self.send_order_state(
                self.init_pick_package_msg(
                    order_id,
                    robot_id,
                    order.recipient,
                    order.sender,
                    order.recipient_location,
                    order.sender_location,
                )
            )
        elif state == 3:
            order: Order = self.confirmed["order_id"]
            order.state = 3
            self.send_order_state(
                self.init_dl_msg(
                    order_id,
                    robot_id,
                    order.recipient,
                    order.sender,
                    order.recipient_location,
                    order.sender_location,
                )
            )
        elif state == 4:
            order: Order = self.confirmed["order_id"]
            order.state = 4
            self.send_order_state(
                self.init_end_msg(
                    order_id,
                    robot_id,
                    order.recipient,
                    order.sender,
                    order.recipient_location,
                    order.sender_location,
                )
            )
            order_ref = self.FS.collection("Orders").document(order_id)
            order_ref.update({"state": 2})  # represent complete
            self.confirmed.pop(order_id)
        else:
            # imposible state
            pass

    def init_pl_msg(
        self,
        order_id: str,
        robot_id: int,
        recipient: str,
        sender: str,
        recipient_location: str,
        sender_location: str,
    ):
        try:
            narrate1 = "'" + recipient + "'" + " in topics"
            narrate2 = "'" + sender + "'" + " in topics"
            condition = narrate1 + " || " + narrate2
            log: str = (
                "robot " + str(robot_id) + " arrive pick up location " + sender_location
            )
            message = messaging.Message(
                condition=condition,
                notification=messaging.Notification(
                    title="PME_AMR",
                    body=log,
                ),
                data={"order_id": order_id, "state": "4"},
            )
            rospy.loginfo("init arrive pick up location message succeed")
            return message
        except Exception as e:
            rospy.logerr("init arrive pick up location message error %s", e)

    def init_dl_msg(
        self,
        order_id: str,
        robot_id: int,
        recipient: str,
        sender: str,
        recipient_location: str,
        sender_location: str,
    ):
        try:
            narrate1 = "'" + recipient + "'" + " in topics"
            narrate2 = "'" + sender + "'" + " in topics"
            condition = narrate1 + " || " + narrate2
            log = (
                "robot "
                + str(robot_id)
                + " arrive drop off location "
                + recipient_location
            )
            message = messaging.Message(
                condition=condition,
                notification=messaging.Notification(
                    title="PME_AMR",
                    body=log,
                ),
                data={"order_id": order_id, "state": "5"},
            )
            rospy.loginfo("init arrive drop off location message succeed")
            return message
        except Exception as e:
            rospy.logerr("init arrive drop off location message error %s", e)

    def init_end_msg(
        self,
        order_id: str,
        robot_id: int,
        recipient: str,
        sender: str,
        recipient_location: str,
        sender_location: str,
    ):
        try:
            narrate1 = "'" + recipient + "'" + " in topics"
            narrate2 = "'" + sender + "'" + " in topics"
            condition = narrate1 + " || " + narrate2
            log = "order: " + order_id + " successfully complete!"
            message = messaging.Message(
                condition=condition,
                notification=messaging.Notification(
                    title="PME_AMR",
                    body=log,
                ),
                data={"order_id": order_id, "state": "2"},
            )
            rospy.loginfo("init complete message succeed")
            return message
        except Exception as e:
            rospy.logerr("init complete message error %s", e)

    def init_pick_package_msg(
        self,
        order_id: str,
        robot_id: int,
        recipient: str,
        sender: str,
        recipient_location: str,
        sender_location: str,
    ):
        try:
            narrate1 = "'" + recipient + "'" + " in topics"
            narrate2 = "'" + sender + "'" + " in topics"
            condition = narrate1 + " || " + narrate2
            log = order_id + ": robot has pick up package successfully!"
            message = messaging.Message(
                condition=condition,
                notification=messaging.Notification(
                    title="PME_AMR",
                    body=log,
                ),
                data={"order_id": order_id, "state": "-1"},
            )
            rospy.loginfo("init pick package message succeed")
            return message
        except Exception as e:
            rospy.logerr("init pick package message error %s", e)

    def send_order_state(self, message):
        try:
            response = messaging.send(message, False, self.app)
            rospy.loginfo("Successfully sent message Established: %s", response)
        except exceptions.FirebaseError:
            rospy.logerr("Send Msg: Firebase Error")
        except ValueError as e:
            rospy.logerr("Send Msg: Value Error: %s", e)
        except:
            rospy.logerr("Send Msg: Else Error")
