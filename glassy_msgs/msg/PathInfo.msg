std_msgs/Header header


int32 STRAIGHT=1
int32 ARC=2

int32 ARC_INFO_SIZE=5
int32 STRAIGHT_INFO_SIZE=4

int32[] path_segments
float64[] path_segment_info
float64[] desired_path_velocity_per_segment
bool loop false
bool path_recalculated
int32 path_segs_to_ignore_in_loop 0
float64 x_correction 0
float64 y_correction 0