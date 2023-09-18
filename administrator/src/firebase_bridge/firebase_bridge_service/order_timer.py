import rospy
import sys
from copy import deepcopy
from datetime import datetime
from firebase_bridge_class import Order
from administrator.msg import order_msgs
from firebase_admin import firestore
from firebase_admin import messaging
from firebase_admin import exceptions


class Order_Timeout_Service:
    def __init__(self, app, unconfirmed: dict):
        self.unconfirmed = unconfirmed
        self.app = app
        self.InitService()

    def __del__(self):
        pass

    def InitService(self):
        try:
            self.confirmed_pub = rospy.Publisher("confirmed_order", order_msgs)
            self.FS = firestore.client()
            self.RegistOrderTimer()

            rospy.loginfo("Init order timeout service success")
        except Exception as e:
            rospy.logerr("Error when init Firestore: %s", e)

    def RegistOrderTimer(self):
        def callback(event):
            unconfirmed = deepcopy(
                self.unconfirmed
            )  # prevent RuntimeError: dictionary changed size during iteration
            for order_id, order in unconfirmed.items():
                if self.CheckTimeout(order_id):
                    rospy.logwarn("%s timeout", order_id)
                    # timeout
                    order_ref = self.FS.collection("Orders").document(order_id)
                    order_ref.update({"state": 2})  # represent Fail
                    order_to_del = self.unconfirmed.pop(order_id)
                    self.SendOrderFailed(
                        order_to_del.recipient,
                        order_to_del.sender,
                        order_id,
                    )
                else:
                    if order.state == 1:
                        self.SendToVehicleRouter(order_id, order)
                    elif order.state == 0:
                        rospy.loginfo("not yet")
                        pass

        rospy.Timer(rospy.Duration(1), callback)

    def CheckTimeout(self, id: str):
        POSIX_now = datetime.now().timestamp()
        POSIX_time = self.IdDecode(id)
        limit_second = 300
        if (POSIX_now - POSIX_time) > limit_second:
            return True
        else:
            return False

    def IdDecode(self, id: str):
        try:
            idsplit = id.split("-")
            timestamp_str = idsplit[1]
            order_datetime = datetime.strptime(timestamp_str, "%Y%m%d%H%M%S")
            POSIX_time = order_datetime.timestamp()
            return POSIX_time
        except ValueError as e:
            rospy.logerr("IdDecode error, id may be wrong: %s", e)
        except Exception as e:
            rospy.logerr("IdDecode error: %s", e)

    def SendToVehicleRouter(self, order_id: str, order: Order):
        try:
            msg = self.OrderToMsg(order_id=order_id, order=order)
            self.confirmed_pub.publish(msg)
            rospy.loginfo("send to vehicle router")
        except Exception as e:
            rospy.logerr("send to vehicle router error %s", e)

    def SendOrderFailed(self, recipient: str, sender: str, order_id: str):
        try:
            narrate1 = "'" + recipient + "'" + " in topics"
            narrate2 = "'" + sender + "'" + " in topics"
            condition = narrate1 + " || " + narrate2
            message = messaging.Message(
                condition=condition,
                notification=messaging.Notification(
                    title="PME_AMR",
                    body="Order timeout and failed!",
                ),
                data={"order_id": order_id},
            )
            rospy.loginfo("init message succeed")
        except Exception as e:
            rospy.logerr("Init compound message error %s", e)
            sys.exit()

        try:
            response = messaging.send(message, False, self.app)
            rospy.loginfo("Successfully sent message: %s", response)
        except exceptions.FirebaseError:
            rospy.logerr("Send Msg: Firebase Error")
            sys.exit()
        except ValueError:
            rospy.logerr("Send Msg: Value Error")
            sys.exit()
        except:
            rospy.logerr("Send Msg: Else Error")
            sys.exit()

    def OrderToMsg(self, order_id: str, order: Order, prior: int = -1):
        msg = order_msgs()
        msg.order_id = order_id
        msg.missionType = order.missionType
        msg.random_password = order.random_password
        msg.recipient = order.recipient
        msg.recipient_location = order.recipient_location
        msg.sender = order.sender
        msg.sender_location = order.sender_location
        msg.priority = prior
        return msg
