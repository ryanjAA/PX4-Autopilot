# Local operator decision for one finalized MAVLink-M cue or handover.
# This is an internal uORB event. MAV_M_ACTION is the local movement permission
# ceiling. requested_effect selects the effect for this exact accepted cue.

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

uint8 EFFECT_DEFAULT = 0
uint8 EFFECT_LEVEL_TRAVEL = 1
uint8 EFFECT_INTERCEPT = 2
uint8 requested_effect
