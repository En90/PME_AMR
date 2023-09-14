from firebase_bridge_class import Order
from firebase_admin import firestore
from firebase_admin import messaging
from firebase_admin import exceptions

import sys
import rospy


class Receive_Order_Service:
    def __init__(self, unconfirmed: dict, confirmed: dict):
        self.InitService()
        self.unconfirmed = unconfirmed
        self.confirmed = confirmed

    def __del__(self):
        self.Confirmed_listener.unsubscribe()
        self.Unconfirmed_listener.unsubscribe()

    def InitService(self):
        try:
            self.FS = firestore.client()
            self.RegistUnconfirmedOrdersListener()
            self.RegistConfirmedOrdersListener()
            rospy.loginfo("Init receive order service success")
        except Exception as e:
            rospy.logerr("Error when init Firestore: %s", e)

    def RegistUnconfirmedOrdersListener(self):
        def on_snapshot(col_snapshot, changes, read_time):
            # rospy.loginfo("in callback")
            # for doc in col_snapshot:
            #     rospy.loginfo("Received document snapshot: %s", doc.id)
            for change in changes:
                if change.type.name == "ADDED":
                    rospy.loginfo("New unconfirmed:\n%s", change.document.id)
                    order = Order.from_dict(change.document.to_dict())
                    # rospy.loginfo("order info: %s", order)
                    # add order into unconfirmed dict byorder_id
                    self.unconfirmed[change.document.id] = order
                elif change.type.name == "MODIFIED":
                    rospy.loginfo("Modified: %s", change.document.id)
                elif change.type.name == "REMOVED":
                    rospy.loginfo("Removed: %s", change.document.id)

        try:
            Orders_ref = self.FS.collection("Orders").where(
                filter=firestore.FieldFilter("state", "==", 0)
            )
            self.Unconfirmed_listener = Orders_ref.on_snapshot(on_snapshot)
        except Exception as e:
            rospy.logerr("Error when init FireStore listener: %s", e)

    def RegistConfirmedOrdersListener(self):
        def on_snapshot(col_snapshot, changes, read_time):
            # rospy.loginfo("in callback")
            # for doc in col_snapshot:
            #     rospy.loginfo("Received document snapshot: %s", doc.id)
            for change in changes:
                if change.type.name == "ADDED":
                    rospy.loginfo("New confirmed:\n%s", change.document.id)
                    order = Order.from_dict(change.document.to_dict())
                    # rospy.loginfo("order info: %s", order)
                    # remove order from unconfirm dict, and add to confirmed dict
                    self.unconfirmed.pop(change.document.id)
                    self.confirmed[change.document.id] = order
                elif change.type.name == "MODIFIED":
                    rospy.loginfo("Modified: %s", change.document.id)
                elif change.type.name == "REMOVED":
                    rospy.loginfo("Removed: %s", change.document.id)

        try:
            Orders_ref = self.FS.collection("Orders").where(
                filter=firestore.FieldFilter("state", "==", 1)
            )
            self.Confirmed_listener = Orders_ref.on_snapshot(on_snapshot)
        except Exception as e:
            rospy.logerr("Error when init FireStore listener: %s", e)

    def InitCompoundMsg(self):
        try:
            message = messaging.Message(
                topic="/topics/gg",
                notification=messaging.Notification(
                    title="PME_AMR",
                    body="Packages awaiting your signature~",
                ),
                data={
                    "sender": "userA",
                    "recipient": "userB",
                    "sender_location": "locA",
                    "recipient_location": "locB",
                    "order_id": "order_id",
                },
            )
            rospy.loginfo("init message succeed")
            return message
        except Exception as e:
            rospy.logerr("Init compound message error %s", e)
            sys.exit()

    def SendMsg(self, msg):
        try:
            response = messaging.send(msg, False, self.app)
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