# The representation of a solution to a planning problem

# The corresponding robot state
RobotState[] trajectory_starts

# The group used for planning (usually the same as in the request)
string group_name

# A solution trajectory, if found
RobotTrajectory[] trajectories

# Planning time (seconds)
float64 planning_time

int64 start_index

# Error code - encodes the overall reason for failure
MoveItErrorCodes error_code
