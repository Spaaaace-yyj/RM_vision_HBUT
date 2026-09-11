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

# 弹丸到达时刻的预测打击点（仅能量机关填写，其他目标为 0）
geometry_msgs/Point predictive_point
