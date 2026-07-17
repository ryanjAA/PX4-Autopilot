# Display and local state for one MAVLink-M cue/handover endpoint.
# Vehicle command authority is controlled separately by MAV_M_ACTION.

uint64 timestamp
uint64 assignment_time_usec
uint64 track_identity_time_usec

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
uint32 target_set_id
uint8 source_system
uint8 source_component
uint8 origin_system
uint8 target_class
uint8 target_force
uint8 queue_depth

bool prompt
bool restored
bool source_fresh
bool position_valid
bool track_identity_valid

int32 lat
int32 lon
float32 alt_msl_m
float32 bearing_deg
float32 relative_bearing_deg
float32 range_m

uint8[16] track_uid
char[51] name
