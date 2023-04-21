# the contrlr name (e.g. smart_motor_ff2)
string   contrlr_name

# specify here a list of parameter_name:value
# e.g. p:10 sets the p value of the controller to 10
# the possible parameters are:
#int16  p
#int16  i
#int16  d
#int16  imax
#int16  target
#int16  sensor
#int16  valve 
#int16  deadband
#int16  offset
#int16  shift
#int16  max
#
## parameters for the motors
#int16  motor_maxforce
#int16  motor_safeforce
#int16  force_p
#int16  force_i
#int16  force_d
#int16  force_imax
#int16  force_out_shift
#int16  force_deadband
#int16  force_offset
#int16  sensor_imax
#int16  sensor_deadband
#int16  sensor_offset
#int16  max_temperature
#int16  max_current

string[] list_of_parameters

# the size of the list_of_parameters you are sending
uint8 	 length_of_list


