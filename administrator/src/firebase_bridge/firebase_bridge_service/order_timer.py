import rospy


class Order_Timeout_Service:
    def __init__(self, confirmed: dict):
        self.confirmed = confirmed

    def RegistOrderTimer(self):
        def callback(event):
            pass

        rospy.Timer(rospy.Duration(1), callback)

    def CheckTimeout(self):
        pass
