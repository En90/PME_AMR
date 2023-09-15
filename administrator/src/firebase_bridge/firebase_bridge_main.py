import rospy
import atexit
from .firebase_bridge_class import Robot, Order
from .firebase_bridge_service import Receive_Order_Service
from firebase_admin import credentials
from firebase_admin import initialize_app
from firebase_admin import delete_app


class firebase_bridge:
    def __init__(self):
        rospy.init_node("firebase_bridge")
        self.app = self.app_init()
        self.Unconfirmed = dict()
        self.Confirmed = dict()
        self.receive_order_service = Receive_Order_Service(
            self.app, self.Unconfirmed, self.Confirmed
        )
        atexit.register(self.exit_handler)

    def exit_handler(self):
        rospy.loginfo("shutdown time!")
        self.receive_order_service = None
        delete_app(self.app)

    def app_init(self):
        try:
            cred = credentials.Certificate(
                "/home/en/catkin_ws/src/graduation_project/administrator/cfg/pme-amr-ba13a-firebase-adminsdk-s75re-46e2eb31ac.json"
            )
            conf = {
                "apiKey": "AIzaSyB6_p9elCG7TcKEN6DMukoXzUWZEASb_0U",
                "authDomain": "pme-amr-ba13a.firebaseapp.com",
                "databaseURL": "https://pme-amr-ba13a-default-rtdb.asia-southeast1.firebasedatabase.app",
                "projectId": "pme-amr-ba13a",
                "storageBucket": "pme-amr-ba13a.appspot.com",
                "messagingSenderId": "323862032769",
                "appId": "1:323862032769:web:2fc273dc01fcc238f07880",
                "measurementId": "G-M21YYTBNY6",
            }
            app = initialize_app(
                credential=cred, options=conf
            )  # options = conf, name = "fcm_server"
            return app
        except Exception as e:
            rospy.logerr(e)


def main():
    node = firebase_bridge()
    rospy.spin()


if __name__ == "__main__":
    main()
