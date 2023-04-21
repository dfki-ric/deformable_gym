int8 motor_id                     # the id of the motor you want to control

bool enable_backlash_compensation # switch on/off backlash compensation at the motor level
bool increase_sgl_tracking        # increment the tracking value for the left gauge
bool decrease_sgl_tracking        # decrement the tracking value for the left gauge
bool increase_sgr_tracking        # increment the tracking value for the right gauge
bool decrease_sgr_tracking        # decrement the tracking value for the right gauge
bool initiate_jiggling            # starts jiggling the given motor
bool write_config_to_eeprom       # write the current configuration to the eeprom