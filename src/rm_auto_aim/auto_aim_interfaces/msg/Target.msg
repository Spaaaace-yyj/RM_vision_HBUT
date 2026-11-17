std_msgs/Header header
bool tracking
string id
int32 armors_num
string type
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
float64 yaw
float64 v_yaw
float64 radius_1
float64 radius_2
float64 dz
# 每块装甲板相对 position.z 的高度偏移，长度等于 armors_num。
# 四板是两组高度交替，前哨站是三块各不一样，只靠单个 dz 表达不了。
float64[] dz_list
