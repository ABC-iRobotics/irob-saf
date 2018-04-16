# InstrumentInfo.msg
# Instrument type
int8 GRIPPER = 1
int8 SCISSORS = 2

# Instrument information
string name
int8 basic_type
float64 jaw_length		# in mm
InstrumentJawPart[] jaw_parts


