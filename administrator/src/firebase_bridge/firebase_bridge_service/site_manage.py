import rospy
from firebase_admin import db
from firebase_admin import exceptions
from firebase_bridge_class import Site
from geometry_msgs.msg import PoseStamped


class Site_Manage_Service:
    def __init__(self, app, sites):
        self.app = app
        self.sites = sites
        self.InitService()

    def __del__(self):
        pass

    def InitService(self):
        try:
            self.site_location_pub = rospy.Publisher(
                "/site_location", PoseStamped, queue_size=500
            )
            self.loc_ref = db.reference("/Location")
            # self.RegistSiteListener()
            self.GetOnce()
            while True:
                if self.site_location_pub.get_num_connections():
                    self.SendSitesLocation()
                    break
            rospy.loginfo("start site manage service success!")

        except Exception as e:
            rospy.logerr("error when regist site listener: %s", e)

    def RegistSiteListener(self):
        # Error: listen() takes 2 positional arguments but 3 were given
        def on_child_removed(event):
            print("removed: ", event.data)

        def on_child_changed(event):
            print("changed: ", event.data)

        def on_child_added(event):
            print("added:", event.data)

        self.loc_ref.listen("child_added", on_child_added)
        self.loc_ref.listen("child_removed", on_child_removed)
        self.loc_ref.listen("child_changed", on_child_changed)

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
            rospy.loginfo("send site?")

    def SendSitesState(self):
        pass
