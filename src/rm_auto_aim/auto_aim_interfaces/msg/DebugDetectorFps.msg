# Published by a 1-second wall timer, independent of debug image publishing.
# Rates use measured steady-clock window duration, not an assumed exact second.
std_msgs/Header header                     # Publication ROS time
float64 window_seconds
float64 input_fps                          # Images entering detector (not hardware camera FPS)
float64 accepted_fps                       # Frames admitted for detection/inference
float64 completed_fps                      # Finished cycles, including zero detections and successful fallback
float64 published_fps                      # /detector/armors publications, including empty Armors
float64 rejected_fps                       # OpenVINO pool full
float64 failed_fps                         # Cycles with an error; may overlap completed via fallback
float64 discarded_fps                      # Accepted async frames invalidated by a mode switch
uint64 input_total
uint64 accepted_total
uint64 completed_total
uint64 published_total
uint64 rejected_total
uint64 failed_total
uint64 discarded_total
uint32 pending                            # OpenVINO tasks/results not yet received
uint32 capacity
