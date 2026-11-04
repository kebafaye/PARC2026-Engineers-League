builtin_interfaces/Time stamp
uint8 IDLE       = 0
uint8 ARMED      = 1
uint8 RUNNING    = 2
uint8 FINISHED   = 3
uint8 TIMED_OUT  = 4
uint8 TERMINATED = 5
uint8 state
float32 elapsed_s
float32 time_limit_s
uint8[] tag_order
uint8[] tags_reported
uint8 start_method
bool false_start
