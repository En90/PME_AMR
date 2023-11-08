import rospy
from firebase_admin import db
from firebase_admin import exceptions
from firebase_bridge_class import Site
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Pose
from rospy import ROSException


class Site_Manage_Service:
    def __init__(self, app, sites):
        self.app = app
        self.sites = sites
        self.InitService()

    def __del__(self):
        self.location_listener.close()
        self.administrator_mode_listener.close()
        # will not go here, close() take too long and cause escalating to SIGTERM
        del self.loc_ref
        rospy.logwarn("shut down Site_Manage_Service")

    def InitService(self):
        try:
            self.in_administrator_mode = False
            self.site_location_pub = rospy.Publisher(
                "/site_location", PoseStamped, queue_size=500
            )
            self.loc_ref = db.reference("/Location")
            self.RegistLocationListener()
            # self.RegistModeListener()
            self.GetOnce()
            while not rospy.is_shutdown():
                if self.site_location_pub.get_num_connections():
                    for site_id, site_value in self.sites.items():
                        self.SendSitesLocation(site_id, site_value)
                    break
            rospy.logwarn("Init site manage service success")

        except Exception as e:
            rospy.logerr("error when regist site listener: %s", e)
            reason: str = "Site_Manage_Service error"
            rospy.signal_shutdown(reason)

    def RegistLocationListener(self):
        def on_child_added(event):
            if event.data == None:
                rospy.logwarn("delete site information: %s", event.path)
            else:
                if event.path == "/":
                    if self.in_administrator_mode == True:
                        self.UpdateSiteLocation(event.data)
                    else:
                        pass
                        # rospy.loginfo("first time initial listener")
                else:
                    if event.event_type == "patch":
                        rospy.loginfo("add new data: %s at %s", event.data, event.path)
                    elif event.event_type == "put":
                        rospy.loginfo(
                            "update value to: %s at %s", event.data, event.path
                        )

        try:
            self.location_listener = self.loc_ref.listen(on_child_added)
        # except exceptions.FirebaseError as e:
        #     rospy.logerr("error when regist location listener: %s", e.code)
        # except exceptions as e:
        #     rospy.logerr("error when regist location listener: %s", e)
        except Exception as e:
            rospy.logwarn("error when regist location listener: %s", e)

    def GetOnce(self):
        for site_id, site_value in self.loc_ref.get().items():
            site = Site().from_dict(source=site_value)
            self.sites[site_id] = site

    def SendSitesLocation(self, site_id, site_value):
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

    def RegistModeListener(self):
        def on_mode_update(event):
            if event.data == 0:
                rospy.loginfo("Shutdown administrator mode")
                self.in_administrator_mode = False
            elif event.data == 1:
                rospy.loginfo("In administrator mode")
                self.in_administrator_mode = True

        try:
            self.administrator_mode_listener = db.reference(
                "/administrator_mode"
            ).listen(on_mode_update)
        except Exception as e:
            rospy.logwarn("error when regist mode listener: %s", e)
        # except exceptions as e:
        #     rospy.logwarn("error when regist mode listener: %s", e)

    # def SendSitesState(self):
    #     pass

    def UpdateSiteLocation(self, data: dict):
        timeout_ = 5
        site_name = (list(data.keys()))[0]
        try:
            rospy.loginfo(
                "Get the current position of the robot as the site %s location",
                site_name,
            )
            msg_get: PoseWithCovarianceStamped = rospy.wait_for_message(
                "/amcl_pose", PoseWithCovarianceStamped, timeout_
            )
        except ROSException as e:
            rospy.logwarn("timeout over %d seconds", timeout_)
        else:
            p: Pose = msg_get.pose.pose
            posit: tuple = (p.position.x, p.position.y, p.position.x)
            orien: tuple = (
                p.orientation.x,
                p.orientation.y,
                p.orientation.z,
                p.orientation.w,
            )
            # add floor
            new_site = Site(floor=3, position=posit, orientation=orien, state=0)
            self.sites[site_name] = new_site
            self.SendSitesLocation(site_name, new_site)
            site_update: dict = new_site.to_dict()
            self.loc_ref.child(site_name).update(site_update)
