uint32 object_id
string frame_id
geometry_msgs/Pose pose
geometry_msgs/Twist velocity
geometry_msgs/Vector3 dimensions # distance from face to face along each axis

uint8 CLASS_CAR = 0 
uint8 CLASS_BICYCLE = 1 
uint8 CLASS_PERSON = 2
uint8 CLASS_TRAFFIC_LIGHT = 3
uint8 CLASS_STOP_SIGN = 4
uint8 CLASS_PEDESTRIAN_CROSSING = 5
uint8 CLASS_SPEED_BUMP = 6
uint8 CLASS_UNKNOWN = 7
uint8 object_class

uint8 SIGNAL_NA = 0
uint8 SIGNAL_GREEN = 1
uint8 SIGNAL_RED = 2
uint8 SIGNAL_YELLOW = 3
uint8 SIGNAL_UNKNOWN = 4
uint8 signal_state
