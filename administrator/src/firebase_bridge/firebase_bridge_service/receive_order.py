from firebase_bridge_class import Order
from firebase_admin import firestore
from firebase_admin import messaging
from firebase_admin import exceptions
from administrator.msg import order_msgs
import sys
import rospy


class Receive_Order_Service:
    def __init__(self, app, unconfirmed: dict, confirmed: dict):
        self.InitService()
        self.unconfirmed = unconfirmed
        self.confirmed = confirmed
        self.app = app

    def __del__(self):
        # self.Confirmed_listener.unsubscribe()
        # self.Unconfirmed_listener.unsubscribe()
        self.OrderListener.unsubscribe()
        pass

    def InitService(self):
        try:
            self.FS = firestore.client()
            # self.RegistUnconfirmedOrdersListener()
            # self.RegistConfirmedOrdersListener()
            self.RegistOrderListener()
            self.confirmed_pub = rospy.Publisher(
                "confirmed_order", order_msgs, queue_size=10
            )
            rospy.loginfo("Init receive order service success")
        except Exception as e:
            rospy.logerr("Error when init Firestore: %s", e)

    # def RegistUnconfirmedOrdersListener(self):
    #     def on_snapshot(col_snapshot, changes, read_time):
    #         # rospy.loginfo("in callback")
    #         # for doc in col_snapshot:
    #         #     rospy.loginfo("Received document snapshot: %s", doc.id)
    #         for change in changes:
    #             if change.type.name == "ADDED":
    #                 rospy.loginfo("New unconfirmed:\n%s", change.document.id)
    #                 order = Order.from_dict(change.document.to_dict())
    #                 # rospy.loginfo("order info: %s", order)
    #                 # add order into unconfirmed dict byorder_id
    #                 self.unconfirmed[change.document.id] = order
    #             elif change.type.name == "MODIFIED":
    #                 rospy.loginfo("Modified: %s", change.document.id)
    #             elif change.type.name == "REMOVED":
    #                 rospy.loginfo("Removed: %s", change.document.id)

    #     try:
    #         Orders_ref = self.FS.collection("Orders").where(
    #             filter=firestore.FieldFilter("state", "==", 0)
    #         )
    #         self.Unconfirmed_listener = Orders_ref.on_snapshot(on_snapshot)
    #     except Exception as e:
    #         rospy.logerr("Error when init FireStore listener: %s", e)

    # def RegistConfirmedOrdersListener(self):
    #     def on_snapshot(col_snapshot, changes, read_time):
    #         # rospy.loginfo("in callback")
    #         # for doc in col_snapshot:
    #         #     rospy.loginfo("Received document snapshot: %s", doc.id)
    #         for change in changes:
    #             if change.type.name == "ADDED":
    #                 rospy.loginfo("New confirmed:\n%s", change.document.id)
    #                 order = Order.from_dict(change.document.to_dict())
    #                 # rospy.loginfo("order info: %s", order)
    #                 # remove order from unconfirm dict, and add to confirmed dict
    #                 self.unconfirmed.pop(change.document.id)
    #                 self.confirmed[change.document.id] = order
    #             elif change.type.name == "MODIFIED":
    #                 rospy.loginfo("Modified: %s", change.document.id)
    #             elif change.type.name == "REMOVED":
    #                 rospy.loginfo("Removed: %s", change.document.id)

    #     try:
    #         Orders_ref = self.FS.collection("Orders").where(
    #             filter=firestore.FieldFilter("state", "==", 1)
    #         )
    #         self.Confirmed_listener = Orders_ref.on_snapshot(on_snapshot)
    #     except Exception as e:
    #         rospy.logerr("Error when init FireStore listener: %s", e)

    def RegistOrderListener(self):
        def on_snapshot(col_snapshot, changes, read_time):
            for doc in col_snapshot:
                rospy.loginfo("Received document snapshot: %s", doc.id)
            for change in changes:
                if change.type.name == "ADDED":
                    rospy.loginfo("New order:\n%s", change.document.id)
                    order = Order().from_dict(change.document.to_dict())
                    if order.state == 0:
                        self.unconfirmed[change.document.id] = order
                        self.RemindRecipient(order.recipient, change.document.id)
                    elif order.state == 1:
                        pass
                        # self.unconfirmed.pop(change.document.id, None)
                        # self.confirmed[change.document.id] = order
                    else:
                        pass
                    # rospy.loginfo(len(self.unconfirmed))
                    # rospy.loginfo(len(self.confirmed))
                elif change.type.name == "MODIFIED":
                    rospy.loginfo("Modified: %s", change.document.id)
                    order = Order().from_dict(change.document.to_dict())
                    if order.state == 0:
                        # self.unconfirmed[change.document.id] = order
                        pass
                    elif order.state == 1:
                        self.unconfirmed.pop(change.document.id)
                        self.confirmed[change.document.id] = order
                        self.SendOrderEstablished(
                            order.recipient, order.sender, change.document.id
                        )
                        self.SendToVehicleRouter(change.document.id, order)
                    else:
                        pass
                    # rospy.loginfo(len(self.unconfirmed))
                    # rospy.loginfo(len(self.confirmed))
                elif change.type.name == "REMOVED":
                    rospy.loginfo("Removed: %s", change.document.id)
                    pass

        try:
            Orders_ref = self.FS.collection("Orders")
            self.OrderListener = Orders_ref.on_snapshot(on_snapshot)
        except Exception as e:
            rospy.logerr("Error when init FireStore listener: %s", e)

    # def InitCompoundMsg(self):
    #     try:
    #         message = messaging.Message(
    #             topic="/topics/gg",
    #             notification=messaging.Notification(
    #                 title="PME_AMR",
    #                 body="Packages awaiting your signature~",
    #             ),
    #             data={
    #                 "sender": "userA",
    #                 "recipient": "userB",
    #                 "sender_location": "locA",
    #                 "recipient_location": "locB",
    #                 "order_id": "order_id",
    #                 "state": "0",
    #             },
    #         )
    #         rospy.loginfo("init message succeed")
    #         return message
    #     except Exception as e:
    #         rospy.logerr("Init compound message error %s", e)
    #         sys.exit()

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

    def RemindRecipient(self, recipient: str, order_id: str):
        rospy.loginfo("remind recipient")
        try:
            message = messaging.Message(
                topic=recipient,
                notification=messaging.Notification(
                    title="PME_AMR",
                    body="Order waiting to be confirmed!",
                ),
                data={"order_id": order_id, "state": "1"},
            )
            rospy.loginfo("init message succeed")
        except Exception as e:
            rospy.logerr("Init compound message error %s", e)
            sys.exit()

        try:
            response = messaging.send(message, False, self.app)
            rospy.loginfo("Successfully sent message RemindRecipient: %s", response)
        except exceptions.FirebaseError:
            rospy.logerr("Send Msg: Firebase Error")
            sys.exit()
        except ValueError:
            rospy.logerr("Send Msg: Value Error")
            sys.exit()
        except:
            rospy.logerr("Send Msg: Else Error")
            sys.exit()

    def SendOrderEstablished(self, recipient: str, sender: str, order_id: str):
        try:
            narrate1 = "'" + recipient + "'" + " in topics"
            narrate2 = "'" + sender + "'" + " in topics"
            condition = narrate1 + " || " + narrate2
            message = messaging.Message(
                condition=condition,
                notification=messaging.Notification(
                    title="PME_AMR",
                    body="Order established!",
                ),
                data={"order_id": order_id},
            )
            rospy.loginfo("init message succeed")
        except Exception as e:
            rospy.logerr("Init compound message error %s", e)
            sys.exit()

        try:
            response = messaging.send(message, False, self.app)
            rospy.loginfo("Successfully sent message Established: %s", response)
        except exceptions.FirebaseError:
            rospy.logerr("Send Msg: Firebase Error")
            sys.exit()
        except ValueError as e:
            rospy.logerr("Send Msg: Value Error: %s", e)
            sys.exit()
        except:
            rospy.logerr("Send Msg: Else Error")
            sys.exit()

    def SendToVehicleRouter(self, order_id: str, order: Order):
        try:
            msg = self.OrderToMsg(order_id=order_id, order=order)
            self.confirmed_pub.publish(msg)
            rospy.loginfo("send to vehicle router")
        except Exception as e:
            rospy.logerr("send to vehicle router error %s", e)

    def OrderToMsg(self, order_id: str, order: Order, prior: int = -1):
        msg = order_msgs()
        msg.order_id.data = order_id
        msg.missionType.data = order.missionType
        msg.random_password.data = order.random_password
        msg.recipient.data = order.recipient
        msg.recipient_location.data = order.recipient_location
        msg.sender.data = order.sender
        msg.sender_location.data = order.sender_location
        msg.priority.data = prior
        return msg
