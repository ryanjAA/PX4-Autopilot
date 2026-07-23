#!/usr/bin/env bash

# Deterministic two-aircraft Gazebo Harmonic lab for AAGS/MAVLink-M testing.
# The runner uses only normal PX4 1.16 Gazebo support, isolated rootfs
# directories, one private Gazebo partition, and no global process kills.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
BUILD="${PX4_BUILD_DIR:-$ROOT/build/px4_sitl_default}"
RUN_ROOT="${AAGS_DUAL_RUN_ROOT:-$BUILD/aags-dual-gz}"
SESSION="${AAGS_DUAL_TMUX_SESSION:-aags-dual-gz}"
PARTITION="${AAGS_DUAL_GZ_PARTITION:-$SESSION}"
WORLD="${AAGS_DUAL_GZ_WORLD:-default}"
WORLD_FILE="$ROOT/Tools/simulation/gz/worlds/$WORLD.sdf"
PRESERVE_CUE_STATE="${AAGS_DUAL_PRESERVE_CUE_STATE:-0}"
OPEN_OWNER="${AAGS_DUAL_OPEN_OWNER:-0}"
INSTANCE_OFFSET="${AAGS_DUAL_INSTANCE_OFFSET:-${AAGS_DUAL_PORT_OFFSET:-0}}"
PORT_OFFSET="${AAGS_DUAL_PORT_OFFSET:-$INSTANCE_OFFSET}"
PX4="$BUILD/bin/px4"
GZ_ENV="$BUILD/rootfs/gz_env.sh"
RUN_ROOT_MARKER_NAME=".aags-dual-gz-owned"

[[ "$INSTANCE_OFFSET" =~ ^-?[0-9]+$ ]] \
	|| { echo "error: AAGS_DUAL_INSTANCE_OFFSET must be an integer" >&2; exit 1; }
[[ "$PORT_OFFSET" =~ ^-?[0-9]+$ ]] \
	|| { echo "error: AAGS_DUAL_PORT_OFFSET must be an integer" >&2; exit 1; }
[[ "$INSTANCE_OFFSET" == "$PORT_OFFSET" ]] \
	|| {
		echo "error: PX4 1.16 derives its Fleet UDP port from the SITL instance;" >&2
		echo "       AAGS_DUAL_INSTANCE_OFFSET and AAGS_DUAL_PORT_OFFSET must match" >&2
		exit 1
	}

FIRST_INSTANCE=$((43 + INSTANCE_OFFSET))
SECOND_INSTANCE=$((44 + INSTANCE_OFFSET))
FIRST_SYSID=$((FIRST_INSTANCE + 1))
SECOND_SYSID=$((SECOND_INSTANCE + 1))
FIRST_PORT=$((18613 + PORT_OFFSET))
SECOND_PORT=$((18614 + PORT_OFFSET))
FIRST_WINDOW="sys$FIRST_SYSID"
SECOND_WINDOW="sys$SECOND_SYSID"

(( FIRST_INSTANCE >= 0 && SECOND_INSTANCE <= 254 )) \
	|| { echo "error: instance offset produces a MAVLink system ID outside 1..255" >&2; exit 1; }
(( FIRST_PORT >= 1024 && SECOND_PORT <= 65535 )) \
	|| { echo "error: port offset produces a Fleet UDP port outside 1024..65535" >&2; exit 1; }
(( FIRST_PORT == 18570 + FIRST_INSTANCE && SECOND_PORT == 18570 + SECOND_INSTANCE )) \
	|| { echo "error: Fleet UDP ports do not match the PX4 1.16 SITL port formula" >&2; exit 1; }

usage() {
	cat <<'EOF'
Usage: run_dual_gz.sh [start|stop|status|configure|attach|reset]

start      Start headless Gazebo Harmonic and PX4 SYS44/SYS45 (default)
stop       Cleanly stop only this runner's tmux session
status     Show the runner panes, Gazebo world, and Fleet UDP ports
configure  Reassert and save the owner and MAVLink-M test parameters
attach     Attach an interactive terminal to the runner
reset      Stop the runner and delete only its generated rootfs state

Port allocation:
  SYS44: PX4 instance 43, Fleet UDP 18613
  SYS45: PX4 instance 44, Fleet UDP 18614

Both AAGS stations register on each vehicle's one bounded Fleet peer table.

Optional environment:
  PX4_BUILD_DIR                    Select another px4_sitl_default build
  AAGS_DUAL_RUN_ROOT               Select another generated run directory
  AAGS_DUAL_TMUX_SESSION           Select another tmux session name
  AAGS_DUAL_GZ_PARTITION           Select another isolated Gazebo partition
  AAGS_DUAL_GZ_WORLD               Select another installed Gazebo world
  AAGS_DUAL_PRESERVE_CUE_STATE=1   Reuse parameters and durable cue state
  AAGS_DUAL_OPEN_OWNER=1           Permit any registered AAGS system to cue
                                   and decide on both vehicles
  AAGS_DUAL_INSTANCE_OFFSET=N       Add N to both PX4 instances and sysids
  AAGS_DUAL_PORT_OFFSET=N           Add the same N to both Fleet UDP ports

PX4 1.16 derives the GCS/Fleet port as 18570 plus the SITL instance, so the
two offsets must match. Their default is zero, which preserves SYS44/SYS45 and
ports 18613/18614.
EOF
}

die() {
	echo "error: $*" >&2
	exit 1
}

normalize_run_root() {
	local create="${1:-0}"
	[[ "$RUN_ROOT" == /* ]] \
		|| die "AAGS_DUAL_RUN_ROOT must be an absolute path"
	[[ "$RUN_ROOT" != "/" && "$RUN_ROOT" != "$HOME" && "$RUN_ROOT" != "$HOME/" ]] \
		|| die "refusing unsafe run directory: $RUN_ROOT"
	[[ ! -L "$RUN_ROOT" ]] \
		|| die "refusing symlink run directory: $RUN_ROOT"

	if [[ ! -d "$RUN_ROOT" ]]; then
		[[ "$create" == "1" ]] \
			|| die "run directory does not exist: $RUN_ROOT"
		mkdir -p "$RUN_ROOT"
	fi

	RUN_ROOT="$(cd "$RUN_ROOT" && pwd -P)"
	[[ "$RUN_ROOT" != "/" && "$RUN_ROOT" != "$HOME" && "$RUN_ROOT" != "$HOME/" ]] \
		|| die "refusing unsafe canonical run directory: $RUN_ROOT"
}

run_root_marker_contents() {
	printf 'AAGS_DUAL_GZ_RUN_ROOT_V1\nrun_root=%s\nsession=%s\npartition=%s\n' \
		"$RUN_ROOT" "$SESSION" "$PARTITION"
}

validate_run_root_marker() {
	normalize_run_root 0
	local marker="$RUN_ROOT/$RUN_ROOT_MARKER_NAME"
	[[ -f "$marker" && ! -L "$marker" ]] \
		|| die "refusing unmarked run directory: $RUN_ROOT"
	local expected actual
	expected="$(run_root_marker_contents)"
	actual="$(<"$marker")"
	[[ "$actual" == "$expected" ]] \
		|| die "run directory marker does not match this session and partition: $RUN_ROOT"
}

prepare_run_root() {
	normalize_run_root 1
	local marker="$RUN_ROOT/$RUN_ROOT_MARKER_NAME"

	if [[ -e "$marker" ]]; then
		validate_run_root_marker
		return
	fi

	if find "$RUN_ROOT" -mindepth 1 -maxdepth 1 -print -quit | grep -q .; then
		die "refusing nonempty unmarked run directory: $RUN_ROOT"
	fi

	run_root_marker_contents >"$marker"
}

require_command() {
	command -v "$1" >/dev/null 2>&1 || die "required command is missing: $1"
}

session_exists() {
	tmux has-session -t "=$SESSION" 2>/dev/null
}

window_exists() {
	local window="$1"
	tmux list-windows -t "=$SESSION" -F '#{window_name}' 2>/dev/null \
		| grep -Fxq "$window"
}

capture_window() {
	local window="$1"
	tmux capture-pane -p -t "=$SESSION:$window" -S -200 2>/dev/null || true
}

wait_for_text() {
	local window="$1"
	local text="$2"
	local attempts="${3:-160}"

	for ((attempt = 0; attempt < attempts; attempt++)); do
		if capture_window "$window" | grep -Fq "$text"; then
			return 0
		fi
		sleep 0.25
	done
	return 1
}

wait_for_world() {
	for _ in {1..160}; do
		if env GZ_PARTITION="$PARTITION" GZ_IP=127.0.0.1 gz topic -l 2>/dev/null \
			| grep -Fqx "/world/$WORLD/clock"; then
			return 0
		fi
		sleep 0.25
	done
	return 1
}

fleet_port_is_busy() {
	local port="$1"
	lsof -nP -iUDP:"$port" 2>/dev/null | tail -n +2 | grep -q .
}

owner_for_vehicle() {
	local system_id="$1"
	if [[ "$OPEN_OWNER" == "1" ]]; then
		echo "-1"
	elif [[ "$system_id" == "$FIRST_SYSID" ]]; then
		echo "254"
	else
		echo "253"
	fi
}

px4_launch_command() {
	local instance="$1"
	local system_id="$2"
	local pose="$3"
	local rootfs="$4"
	local owner
	owner="$(owner_for_vehicle "$system_id")"

	local command
	printf -v command 'source %q && exec env' "$GZ_ENV"
	local assignments=(
		"GZ_PARTITION=$PARTITION"
		"GZ_IP=127.0.0.1"
		"HEADLESS=1"
		"PX4_GZ_STANDALONE=1"
		"PX4_GZ_WORLD=$WORLD"
		"PX4_GZ_MODEL_POSE=$pose"
		"PX4_SIM_MODEL=gz_rc_cessna"
		"PX4_PARAM_MAV_M_MODE=1"
		"PX4_PARAM_MAV_M_SAME_EP=1"
		"PX4_PARAM_MAV_M_INST=0"
		"PX4_PARAM_MAV_M_SRC_SYS=$owner"
		"PX4_PARAM_MAV_M_SRC_CMP=190"
		"PX4_PARAM_MAV_M_CTL_INST=0"
		"PX4_PARAM_MAV_M_CTL_SYS=$owner"
		"PX4_PARAM_MAV_M_CTL_CMP=190"
		"PX4_PARAM_MAV_M_LNK_ID=0"
		"PX4_PARAM_MAV_M_CTL_LNK=0"
		"PX4_PARAM_MAV_M_PEERS=4"
		"PX4_PARAM_MAV_M_P_TMO=30"
		"PX4_PARAM_MAV_M_MAX_AGE=300"
		"PX4_PARAM_MAV_M_RC_CH=0"
		"PX4_PARAM_MAV_M_ACTION=2"
		"PX4_PARAM_MAV_M_INT_CLR=-1"
	)
	local assignment
	for assignment in "${assignments[@]}"; do
		printf -v command '%s %q' "$command" "$assignment"
	done
	printf -v command '%s %q %q -i %q -w %q -s %q' \
		"$command" "$PX4" "$BUILD/etc" "$instance" "$rootfs" \
		"etc/init.d-posix/rcS"
	printf '%s\n' "$command"
}

status() {
	if session_exists; then
		echo "tmux session: $SESSION"
		tmux list-panes -s -t "=$SESSION" \
			-F '#{session_name}:#{window_name} pid=#{pane_pid} command=#{pane_current_command}'
	else
		echo "tmux session: stopped"
	fi

	echo "Gazebo partition: $PARTITION"
	echo "Gazebo world: $WORLD"
	echo "SITL Intercept terrain override: MAV_M_INT_CLR=-1"
	env GZ_PARTITION="$PARTITION" GZ_IP=127.0.0.1 gz topic -l 2>/dev/null \
		| grep -E '^/world/.*/clock$' || true
	echo "SYS$FIRST_SYSID Fleet UDP: $FIRST_PORT"
	echo "SYS$SECOND_SYSID Fleet UDP: $SECOND_PORT"
	lsof -nP -iUDP:"$FIRST_PORT" -iUDP:"$SECOND_PORT" 2>/dev/null || true
}

stop() {
	if ! session_exists; then
		echo "Dual Gazebo session is already stopped"
		return 0
	fi

	local gazebo_pid=""
	if window_exists gazebo; then
		gazebo_pid="$(tmux display-message -p -t "=$SESSION:gazebo" '#{pane_pid}' 2>/dev/null || true)"
	fi

	for window in "$FIRST_WINDOW" "$SECOND_WINDOW"; do
		if window_exists "$window"; then
			tmux send-keys -t "=$SESSION:$window" shutdown Enter
		fi
	done

	for _ in {1..32}; do
		if ! window_exists "$FIRST_WINDOW" && ! window_exists "$SECOND_WINDOW"; then
			break
		fi
		sleep 0.25
	done

	if window_exists gazebo; then
		tmux send-keys -t "=$SESSION:gazebo" C-c
	fi
	for _ in {1..20}; do
		if [[ -z "$gazebo_pid" ]] || ! kill -0 "$gazebo_pid" 2>/dev/null; then
			break
		fi
		sleep 0.25
	done
	if [[ -n "$gazebo_pid" ]] && kill -0 "$gazebo_pid" 2>/dev/null; then
		kill -TERM "$gazebo_pid" 2>/dev/null || true
	fi

	tmux kill-session -t "=$SESSION" 2>/dev/null || true
	echo "Stopped dual Gazebo session: $SESSION"
}

reset() {
	stop
	validate_run_root_marker
	rm -rf "$RUN_ROOT"
	echo "Removed generated lab state: $RUN_ROOT"
}

send_parameter_set() {
	local window="$1"
	local name="$2"
	local value="$3"
	tmux send-keys -t "=$SESSION:$window" "param set $name $value" Enter
}

configure_vehicle() {
	local window="$1"
	local system_id="$2"
	local owner
	owner="$(owner_for_vehicle "$system_id")"

	send_parameter_set "$window" MAV_M_MODE 1
	send_parameter_set "$window" MAV_M_SAME_EP 1
	send_parameter_set "$window" MAV_M_INST 0
	send_parameter_set "$window" MAV_M_SRC_SYS "$owner"
	send_parameter_set "$window" MAV_M_SRC_CMP 190
	send_parameter_set "$window" MAV_M_CTL_INST 0
	send_parameter_set "$window" MAV_M_CTL_SYS "$owner"
	send_parameter_set "$window" MAV_M_CTL_CMP 190
	send_parameter_set "$window" MAV_M_LNK_ID 0
	send_parameter_set "$window" MAV_M_CTL_LNK 0
	send_parameter_set "$window" MAV_M_PEERS 4
	send_parameter_set "$window" MAV_M_P_TMO 30
	send_parameter_set "$window" MAV_M_MAX_AGE 300
	send_parameter_set "$window" MAV_M_RC_CH 0
	send_parameter_set "$window" MAV_M_ACTION 2
	send_parameter_set "$window" MAV_M_INT_CLR -1
	tmux send-keys -t "=$SESSION:$window" "param save" Enter
	tmux send-keys -t "=$SESSION:$window" "param show MAV_M_*" Enter
	tmux send-keys -t "=$SESSION:$window" "aags_cfg_done_$system_id" Enter
}

configure() {
	session_exists || die "tmux session $SESSION is not running"
	window_exists "$FIRST_WINDOW" || die "SYS$FIRST_SYSID PX4 window is not running"
	window_exists "$SECOND_WINDOW" || die "SYS$SECOND_SYSID PX4 window is not running"

	configure_vehicle "$FIRST_WINDOW" "$FIRST_SYSID"
	configure_vehicle "$SECOND_WINDOW" "$SECOND_SYSID"
	wait_for_text "$FIRST_WINDOW" "Invalid command: aags_cfg_done_$FIRST_SYSID" 80 \
		|| die "SYS$FIRST_SYSID did not finish parameter configuration"
	wait_for_text "$SECOND_WINDOW" "Invalid command: aags_cfg_done_$SECOND_SYSID" 80 \
		|| die "SYS$SECOND_SYSID did not finish parameter configuration"

	for window in "$FIRST_WINDOW" "$SECOND_WINDOW"; do
		local output
		output="$(capture_window "$window")"
		local system_id="$SECOND_SYSID"
		[[ "$window" == "$FIRST_WINDOW" ]] && system_id="$FIRST_SYSID"
		local owner
		owner="$(owner_for_vehicle "$system_id")"
		local expectations=(
			"MAV_M_MODE:1"
			"MAV_M_SAME_EP:1"
			"MAV_M_INST:0"
			"MAV_M_SRC_SYS:$owner"
			"MAV_M_CTL_INST:0"
			"MAV_M_CTL_SYS:$owner"
			"MAV_M_PEERS:4"
			"MAV_M_P_TMO:30"
			"MAV_M_MAX_AGE:300"
			"MAV_M_ACTION:2"
			"MAV_M_INT_CLR:-1"
		)
		local expected name value
		for expected in "${expectations[@]}"; do
			name="${expected%%:*}"
			value="${expected#*:}"
			printf '%s\n' "$output" \
				| grep -Eq "^[x+*[:space:]]*${name}[^:]*:[[:space:]]*${value}([.]0+)?[[:space:]]*$" \
				|| die "$window did not apply $name=$value"
		done
	done
	echo "Reasserted and saved MAVLink-M parameters for SYS$FIRST_SYSID and SYS$SECOND_SYSID"
}

start() {
	require_command tmux
	require_command gz
	require_command lsof
	[[ -x "$PX4" ]] || die "PX4 SITL binary is missing: $PX4"
	[[ -d "$BUILD/etc/init.d-posix" ]] || die "PX4 SITL etc tree is missing: $BUILD/etc"
	[[ -f "$GZ_ENV" ]] || die "Gazebo environment is missing: $GZ_ENV"
	[[ -f "$WORLD_FILE" ]] || die "Gazebo world is missing: $WORLD_FILE"
	session_exists && die "tmux session $SESSION is already running"
	fleet_port_is_busy "$FIRST_PORT" && die "Fleet UDP port $FIRST_PORT is already in use"
	fleet_port_is_busy "$SECOND_PORT" && die "Fleet UDP port $SECOND_PORT is already in use"
	prepare_run_root

	if [[ "$PRESERVE_CUE_STATE" != "1" ]]; then
		rm -rf "$RUN_ROOT/rootfs"
	fi
	mkdir -p "$RUN_ROOT/rootfs/$FIRST_INSTANCE" "$RUN_ROOT/rootfs/$SECOND_INSTANCE"

	local gz_command
	printf -v gz_command \
		'source %q && exec env GZ_PARTITION=%q GZ_IP=127.0.0.1 HEADLESS=1 gz sim --verbose=1 -r -s %q' \
		"$GZ_ENV" "$PARTITION" "$WORLD_FILE"
	tmux new-session -d -s "$SESSION" -n gazebo "$gz_command"

	if ! wait_for_world; then
		capture_window gazebo >&2
		stop
		die "Gazebo world $WORLD did not become ready"
	fi

	local first_command
	first_command="$(px4_launch_command "$FIRST_INSTANCE" "$FIRST_SYSID" "0,0,0" "$RUN_ROOT/rootfs/$FIRST_INSTANCE")"
	tmux new-window -t "=$SESSION" -n "$FIRST_WINDOW" "$first_command"
	if ! wait_for_text "$FIRST_WINDOW" "Startup script returned successfully"; then
		capture_window "$FIRST_WINDOW" >&2
		stop
		die "PX4 SYS$FIRST_SYSID did not finish its startup script"
	fi

	local second_command
	second_command="$(px4_launch_command "$SECOND_INSTANCE" "$SECOND_SYSID" "0,80,0" "$RUN_ROOT/rootfs/$SECOND_INSTANCE")"
	tmux new-window -t "=$SESSION" -n "$SECOND_WINDOW" "$second_command"
	if ! wait_for_text "$SECOND_WINDOW" "Startup script returned successfully"; then
		capture_window "$SECOND_WINDOW" >&2
		stop
		die "PX4 SYS$SECOND_SYSID did not finish its startup script"
	fi

	for _ in {1..80}; do
		if fleet_port_is_busy "$FIRST_PORT" && fleet_port_is_busy "$SECOND_PORT"; then
			configure
			echo "Started dual Gazebo Harmonic lab in tmux session: $SESSION"
			echo "SYS$FIRST_SYSID Fleet UDP: 127.0.0.1:$FIRST_PORT"
			echo "SYS$SECOND_SYSID Fleet UDP: 127.0.0.1:$SECOND_PORT"
			echo "Attach with: $0 attach"
			return 0
		fi
		sleep 0.25
	done

	status >&2
	stop
	die "PX4 Fleet UDP ports did not become ready"
}

action="${1:-start}"
case "$action" in
	start) start ;;
	stop) stop ;;
	status) status ;;
	configure) configure ;;
	attach)
		session_exists || die "tmux session $SESSION is not running"
		exec tmux attach -t "=$SESSION"
		;;
	reset) reset ;;
	-h|--help|help) usage ;;
	*) usage >&2; die "unknown action: $action" ;;
esac
