# Safe, display-oriented state for one MAVLink-M cue/handover endpoint.
# This topic never commands navigation, arming, flight mode, or payload state.

uint64 timestamp
uint64 assignment_time_usec

uint8 STATE_IDLE = 0
uint8 STATE_PENDING = 1
uint8 STATE_ACTIVE = 2
uint8 STATE_QUEUED = 3
uint8 STATE_REJECTED = 4
uint8 STATE_ABORTED = 5
uint8 STATE_EXPIRED = 6
uint8 state

uint32 message_id
uint32 instance_id
uint8 source_system
uint8 source_component
uint8 target_class
uint8 target_force
uint8 queue_depth

bool prompt
bool restored
bool source_fresh
bool position_valid

int32 lat
int32 lon
float32 alt_msl_m
float32 bearing_deg
float32 relative_bearing_deg
float32 range_m

uint8[16] track_uid
char[51] name
