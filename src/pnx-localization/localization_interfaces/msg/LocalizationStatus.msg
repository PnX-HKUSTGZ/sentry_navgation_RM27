# 后端已提交定位状态；heartbeat 时间与 pose_stamp 分别表示发布时刻和有效源帧。
std_msgs/Header header
builtin_interfaces/Time pose_stamp
string state
string reason
string instance_id
uint64 epoch
uint64 segment
uint64 generation
bool global_trusted
bool map_ready
bool tf_published
bool frozen
