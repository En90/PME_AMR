# missions:["start_mapping", "finish_control_mapping", "check_map", "choose_map", "move_goal", "move_goal_key", "record_position"
#           "move_to_goal_floor"]
string mission

# if mission is "move_goal", set goal
geometry_msgs/PoseStamped goal

# if mission is "choose_map", choose a map
string choose_map_name

# if mission is "record_position", set name of position
string position_key

# if mission is "check_map", set map name
bool map_ok
string set_map_name

# send to which robot
std_msgs/Int8 robot_id

# if mission is "move_to_goal_floor", set which floor
std_msgs/Int8 floor