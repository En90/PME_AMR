import rospy
import sys
from datetime import datetime
from firebase_admin import firestore
from firebase_admin import messaging
from firebase_admin import exceptions


class Order_Timeout_Service:
    def __init__(self, app, confirmed: dict):
        self.confirmed = confirmed
        self.app = app
        self.allids = confirmed.keys()

    def __del__(self):
        pass

    def InitService(self):
        try:
            self.FS = firestore.client()
            self.RegistOrderTimer()
            rospy.loginfo("Init order timeout service success")
        except Exception as e:
            rospy.logerr("Error when init Firestore: %s", e)

    def RegistOrderTimer(self):
        def callback(event):
            for order_id in self.allids:
                if self.CheckTimeout(order_id):
                    # timeout
                    order_ref = self.FS.collection("Orders").document(order_id)
                    order_ref.update({"state": 2})  # represent Fail
                    order_to_del = self.confirmed.pop(order_id)
                    self.SendOrderFailed(
                        order_to_del.recipient,
                        order_to_del.sender,
                        order_id,
                    )
                else:
                    self.SendToVehicleRouter(order_id)

        rospy.Timer(rospy.Duration(1), callback)

    def CheckTimeout(self, id: str):
        POSIX_now = datetime.now().timestamp()
        POSIX_time = self.IdDecode(id)
        limit_second = 300
        if (POSIX_now - POSIX_time) > limit_second:
            return True
        else:
            return False

    def IdDecode(id: str):
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

    def SendToVehicleRouter(self):
        pass

    def SendOrderFailed(self, recipient: str, sender: str, order_id: str):
        try:
            message = messaging.Message(
                topic=recipient + "," + sender,
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
