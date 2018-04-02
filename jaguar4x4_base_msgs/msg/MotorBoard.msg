# Copyright 2018 Toyota Research Institute.  All rights reserved.
uint8  MOTOR_CONTROL_MODE_OPEN_LOOP = 0
uint8  MOTOR_CONTROL_MODE_CLOSED_SPEED = 1
uint8  MOTOR_CONTROL_MODE_POSITION = 2
uint8  MOTOR_CONTROL_MODE_TORQUE = 4

float32 amp_1         # motor 1 amperage
float32 amp_2         # motor 2 amperage
float32 motor_temp_1  # motor 1 temperature
float32 motor_temp_2  # motor 2 temperature
int16   motor_power_1 # motor 1 power
int16   motor_power_2 # motor 2 power
int64   encoder_pos_1 # encoder position for motor 1
int64   encoder_pos_2 # encoder position for motor 2
int64   encoder_vel_1 # encoder velocity for motor 1
int64   encoder_vel_2 # encoder velocity for motor 2
int64   encoder_diff_1 # Encoder difference from last time
int64   encoder_diff_2 # Encoder difference from last time
float32 board_temp_1  # motor board 1 temperature
float32 board_temp_2  # motor board 2 temperature
float32 volt_main     # motor board main power voltage (battery voltage)
float32 volt_12v      # motor board 12V power voltage
float32 volt_5v       # motor board 5V power voltage
uint8   motor_control_mode_1 # see possible MOTOR_CONTROL_MODE_ values above
uint8   motor_control_mode_2 # see possible MOTOR_CONTROL_MODE_ values above
bool    overheat      # TRUE if board reports overheat
bool    overvoltage   # TRUE if board reports overvoltage
bool    undervoltage  # TRUE if board reports undervoltage
bool    e_short       # TRUE if board reports short
bool    estop         # TRUE if board reports estop
bool    last_cmd_ack  # FALSE for invalid command, TRUE for valid command
int32   digital_input # Status of motor digital input (if it exists)
int32   digital_output # Status of motor digital output (if it exists)
