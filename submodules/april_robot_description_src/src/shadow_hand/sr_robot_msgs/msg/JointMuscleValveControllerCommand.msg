# This command will allow the user to specify a separate command for each of the two muscles that control the joint.
# The user will also specify a duration in ms for that command. During this duration the command will be sent to the hand
# every ms. 
# Once this duration period has elapsed, a command of 0 will be sent to the muscle (meaning both the filling and emptying valves for that 
# muscle remain closed), until a new command is received
# A duration of 0 means that there is no timeout, so the valve command will be sent to the muscle until a different valve command is received.
# BE CAREFUL WHEN USING A DURATION OF 0 AS THIS COULD EVENTUALLY DAMAGE THE MUSCLE

# Allowed values for the cmd_valve_muscle are integers from -4 to +4.
# +4 the filling valve is open during a full ms cycle. Emptying valve is closed
# +3 the filling valve is open during 0.75 ms and closed during 0.25 ms in a 1 ms cycle. Emptying valve is closed
# +2 the filling valve is open during 0.5 ms and closed during 0.5 ms in a 1 ms cycle. Emptying valve is closed
# +1 the filling valve is open during 0.25 ms and closed during 0.75 ms in a 1 ms cycle. Emptying valve is closed
# 0 both valves (filling and emptying) remain closed in a 1 ms cycle
# ...
# -3 the emptying valve is open during 0.75 ms and closed during 0.25 ms in a 1 ms cycle. Filling valve is closed
# -4 the emptying valve is open during a full ms cycle. Filling valve is closed

int8[2] cmd_valve_muscle
uint64[2] cmd_duration_ms



