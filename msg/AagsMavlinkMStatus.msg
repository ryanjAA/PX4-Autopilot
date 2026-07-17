# Inert MAVLink-M cue state for local pilot/OSD presentation.
# This topic is descriptive only and must never be consumed as a navigation,
# arming, flight-mode, or payload command.

uint64 timestamp
uint32 cue_id
uint32 status_sequence
uint64 valid_until_usec

int32 lat
int32 lon
float32 alt
float32 bearing_rad
float32 distance_m
float32 confidence_score

uint8 STATE_EMPTY = 0
uint8 STATE_PENDING = 1
uint8 STATE_ACTIVE = 2
uint8 STATE_EN_ROUTE = 3
uint8 STATE_READY = 4
uint8 STATE_COMPLETE = 5
uint8 STATE_REJECTED = 6
uint8 STATE_ABORTED = 7
uint8 STATE_FAILED = 8
uint8 STATE_EXPIRED = 9

uint8 state
uint8 source_system
uint8 source_component
uint8 cue_type
uint8 target_class
uint8 target_force
bool position_valid
bool local_decision_available
char[20] name
