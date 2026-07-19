# Local operator decision for one finalized MAVLink-M cue or handover.
# This is an internal uORB event. Optional INVESTIGATE-cue navigation and its
# altitude policy after acceptance are controlled separately by MAV_M_ACTION.

# Keep short-lived shell/UI publishers advertised until the MAVLink instance
# consumes the decision, and bound bursts of local operator input.
uint8 ORB_QUEUE_LENGTH = 4

uint64 timestamp
uint32 task_msgid
uint32 task_instance
uint8 target_system

uint8 ACTION_ACCEPT = 1
uint8 ACTION_REJECT = 2
uint8 action
