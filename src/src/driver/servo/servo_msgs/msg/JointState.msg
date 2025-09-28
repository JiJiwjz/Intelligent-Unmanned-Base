Header header
string name         # joint name
int32[] servo_ids   # motor ids controlling this joint
int32[] servo_temps # motor temperatures, same order as motor_ids

float64 goal_pos    # commanded position (in radians)
float64 current_pos # current joint position (in radians)
float64 error       # error between commanded and current positions (in radians)
float64 velocity    # current joint speed (in radians per second)