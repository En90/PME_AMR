import rospy
from firebase_admin import db
from firebase_admin import exceptions
from firebase_bridge_class import Site
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped


class Site_Manage_Service:
    def __init__(self, app, sites):
        self.app = app
        self.sites = sites
        self.InitService()

    def __del__(self):
        del self.site_location_pub
        del self.loc_ref
        pass

    def InitService(self):
        try:
            self.site_location_pub = rospy.Publisher(
                "/site_location", PoseStamped, queue_size=500
            )
            self.loc_ref = db.reference("/Location")
            self.administrator_mode = db.reference("/administrator_mode")
            self.RegistSiteListener()
            # self.RegistModeListener()
            self.GetOnce()
            while not rospy.is_shutdown():
                if self.site_location_pub.get_num_connections():
                    self.SendSitesLocation()
                    break
            rospy.loginfo("Init site manage service success!")

        except Exception as e:
            rospy.logerr("error when regist site listener: %s", e)
            reason: str = "Init compound message error"
            rospy.signal_shutdown(reason)

    def RegistSiteListener(self):
        # Error: listen() takes 2 positional arguments but 3 were given
        def on_child_added(event):
            print("added:", event.data)

        # self.loc_ref.listen("child_added", on_child_added)
        self.loc_ref.listen(on_child_added)

    def GetOnce(self):
        for site_id, site_value in self.loc_ref.get().items():
            site = Site().from_dict(source=site_value)
            self.sites[site_id] = site

    def SendSitesLocation(self):
        for site_id, site_value in self.sites.items():
            msg = PoseStamped()
            msg.header.frame_id = site_id
            msg.pose.position.x = site_value.position[0]
            msg.pose.position.y = site_value.position[1]
            msg.pose.position.z = site_value.position[2]
            msg.pose.orientation.x = site_value.orientation[0]
            msg.pose.orientation.y = site_value.orientation[1]
            msg.pose.orientation.z = site_value.orientation[2]
            msg.pose.orientation.w = site_value.orientation[3]
            msg.pose.position.z = site_value.floor
            self.site_location_pub.publish(msg)

    # def RegistModeListener(self):
    #     def callback(msg):
    #         pass

    #     def on_mode_update(event):
    #         if event.data == 0:
    #             rospy.loginfo("unrigister")
    #             self.order_state_sub.unregister()
    #         elif event.data == 1:
    #             rospy.loginfo("In administrator mode")
    #             self.order_state_sub.add_callback(callback)

    #     self.order_state_sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped)
    #     self.administrator_mode.listen(on_mode_update)

    def SendSitesState(self):
        pass
