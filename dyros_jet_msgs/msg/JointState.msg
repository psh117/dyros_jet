
# DXL Error
uint32 INPUT_VOLTAGE_ERROR	=1
uint32 MOTOR_HALL_SENSOR_ERROR	=2
uint32 OVERHEAT_ERROR		=4
uint32 MOTOR_ENCODER_ERROR	=8
uint32 ELECTRONICAL_SHOCK_ERROR	=16
uint32 OVERLOAD_ERROR		=32
uint32 RESERVED_ERROR		=64
uint32 NOT_UPDATED		=128


string[] name
float64[] angle
float64[] velocity
float64[] current

# bit 0000 0000 0000 0000 0000 0001 0000 0000 = Not updated
# bit 0000 0000 0000 0000 0000 0000 0000 0100 = Not updated, Overheat error
uint32[] error

