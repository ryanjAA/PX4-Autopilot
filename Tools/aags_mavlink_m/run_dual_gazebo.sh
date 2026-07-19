#!/usr/bin/env bash

# Deterministic two-vehicle Gazebo Classic lab for AAGS/MAVLink-M testing.
# Uses one private Gazebo master, two generated plane_lidar models, isolated
# PX4 rootfs directories, SYSIDs 44/45, and no global pkill operations.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
BUILD="${PX4_BUILD_DIR:-$ROOT/build/px4_sitl_default}"
GAZEBO_ROOT="$ROOT/Tools/simulation/gazebo-classic/sitl_gazebo-classic"
RUN_ROOT="${AAGS_DUAL_RUN_ROOT:-$BUILD/aags-dual-gazebo}"
SESSION="${AAGS_DUAL_TMUX_SESSION:-aags-dual-sitl}"
MASTER_URI="${AAGS_DUAL_GAZEBO_MASTER_URI:-http://127.0.0.1:11346}"
HOME_LAT="${PX4_HOME_LAT:-33.4370404}"
HOME_LON="${PX4_HOME_LON:--99.8158795}"
HOME_ALT="${PX4_HOME_ALT:-457}"
PYTHON="$ROOT/.venv/bin/python"
PX4="$BUILD/bin/px4"

usage() {
	cat <<'EOF'
Usage: run_dual_gazebo.sh [start|stop|status|attach]

start   Start headless Gazebo and PX4 SYSIDs 44/45 (default)
stop    Cleanly stop only this runner's tmux session
status  Show tmux panes and allocated simulator/GCS ports
attach  Attach an interactive terminal to both PX4 shells

Port allocation:
  SYSID 44: PX4 instance 43, simulator TCP 4603, local cue/owner UDP 18613,
            observer UDP 18615
  SYSID 45: PX4 instance 44, simulator TCP 4604, local cue/owner UDP 18614,
            observer UDP 18616
EOF
}

die() {
	echo "error: $*" >&2
	exit 1
}

session_exists() {
	tmux has-session -t "$SESSION" 2>/dev/null
}

status() {
	if session_exists; then
		echo "tmux session: $SESSION"
		tmux list-panes -a -t "$SESSION" -F '#{session_name}:#{window_name} pid=#{pane_pid} command=#{pane_current_command}'
	else
		echo "tmux session: stopped"
	fi

	lsof -nP \
		-iTCP:4603 -iTCP:4604 \
		-iUDP:18613 -iUDP:18614 \
		-iUDP:18615 -iUDP:18616 2>/dev/null || true
}

stop() {
	if ! session_exists; then
		echo "Dual Gazebo session is already stopped"
		return 0
	fi

	for window in sys44 sys45; do
		if tmux list-windows -t "$SESSION" -F '#{window_name}' | grep -Fxq "$window"; then
			tmux send-keys -t "$SESSION:$window" shutdown Enter
		fi
	done

	for _ in {1..20}; do
		if ! pgrep -P "$(tmux display-message -p -t "$SESSION:gazebo" '#{pane_pid}')" gzserver >/dev/null 2>&1; then
			break
		fi
		sleep 0.1
	done

	tmux kill-session -t "$SESSION"
	echo "Stopped dual Gazebo session: $SESSION"
}

write_instance_params() {
	local directory="$1"
	local local_owner_system="$2"
	local gcs_port="$3"

	mkdir -p "$directory"
	rm -rf "$directory/etc"
	cp -R "$BUILD/etc" "$directory/etc"
	# This lab exposes each vehicle to two independent AAGS stations. MAVLink
	# forwarding on either PX4 endpoint would reflect GCS heartbeats between
	# those stations and create a routing loop, so every lab-facing stream is
	# intentionally non-forwarding.
	sed -i '' \
		"s#mavlink start -x -u \$udp_gcs_port_local -r 4000000 -f#mavlink start -x -u \$udp_gcs_port_local -o $gcs_port -r 4000000#" \
		"$directory/etc/init.d-posix/px4-rc.mavlink"
	{
		echo '#!/bin/sh'
		echo "param set MAV_0_REMOTE_PRT $gcs_port"
		echo 'param set MAV_M_MODE 1'
		# The local AAGS receives a peer handover, materializes it as a contact,
		# then sends a fresh cue to its own PX4 and explicitly approves it in
		# the owner TASKS card. Both operations therefore use the stock
		# instance-0 owner link. MAV_M_SAME_EP authorizes that co-located
		# workflow without auto-accepting anything.
		echo 'param set MAV_M_SAME_EP 1'
		echo 'param set MAV_M_INST 0'
		echo "param set MAV_M_SRC_SYS $local_owner_system"
		echo 'param set MAV_M_SRC_CMP 190'
		echo 'param set MAV_M_CTL_INST 0'
		echo "param set MAV_M_CTL_SYS $local_owner_system"
		echo 'param set MAV_M_CTL_CMP 190'
		echo 'param set MAV_M_LNK_ID 0'
		echo 'param set MAV_M_CTL_LNK 0'
		echo 'param set MAV_M_MAX_AGE 300'
		echo 'param set MAV_M_RC_CH 0'
		echo 'param set MAV_M_ACTION 0'
	} >"$directory/px4-rc.params"
	chmod +x "$directory/px4-rc.params"
}

generate_model() {
	local sysid="$1"
	local tcp_port="$2"
	local udp_port="$3"
	local model_dir="$RUN_ROOT/models/aags_plane_$sysid"
	local wrapper="$RUN_ROOT/plane_lidar_$sysid.sdf"

	mkdir -p "$model_dir"
	arch -x86_64 "$PYTHON" \
		"$GAZEBO_ROOT/scripts/jinja_gen.py" \
		"$GAZEBO_ROOT/models/plane/plane.sdf.jinja" \
		"$GAZEBO_ROOT" \
		--mavlink_tcp_port "$tcp_port" \
		--mavlink_udp_port "$udp_port" \
		--mavlink_id "$sysid" \
		--gst_udp_port "$((5600 + sysid))" \
		--video_uri "$((5600 + sysid))" \
		--mavlink_cam_udp_port "$((14530 + sysid))" \
		--output-file "$model_dir/plane.sdf"
	cp "$GAZEBO_ROOT/models/plane/model.config" "$model_dir/model.config"
	sed "s#model://plane#model://aags_plane_$sysid#" \
		"$GAZEBO_ROOT/models/plane_lidar/plane_lidar.sdf" >"$wrapper"
}

start() {
	[[ "$(uname -s)" == "Darwin" ]] || die "this runner currently targets macOS"
	command -v tmux >/dev/null 2>&1 || die "tmux is required"
	[[ -x "$PX4" ]] || die "PX4 SITL binary is missing: $PX4"
	[[ -x "$PYTHON" ]] || die "Intel PX4 Python environment is missing: $PYTHON"
	[[ -x /usr/local/bin/gzserver && -x /usr/local/bin/gz ]] ||
		die "Intel Gazebo Classic is required in /usr/local/bin"

	if session_exists; then
		die "tmux session $SESSION is already running; use '$0 status' or '$0 stop'"
	fi

	mkdir -p "$RUN_ROOT/models" "$RUN_ROOT/rootfs/43" "$RUN_ROOT/rootfs/44"
	# GCS254 owns and cues SYS44; GCS253 owns and cues SYS45. A handover
	# between the two AAGS stations is accepted into the receiving contact
	# store before that local station creates the vehicle cue.
	write_instance_params "$RUN_ROOT/rootfs/43" 254 14551
	write_instance_params "$RUN_ROOT/rootfs/44" 253 14550
	generate_model 44 4603 14603
	generate_model 45 4604 14604

	local setup="$ROOT/Tools/simulation/gazebo-classic/setup_gazebo.bash"
	local model_path="$RUN_ROOT/models:$GAZEBO_ROOT/models"
	local common_env="PATH=/usr/local/bin:/usr/local/sbin:/usr/bin:/bin GAZEBO_MASTER_URI=$MASTER_URI GAZEBO_MODEL_PATH=$model_path PX4_HOME_LAT=$HOME_LAT PX4_HOME_LON=$HOME_LON PX4_HOME_ALT=$HOME_ALT"

	tmux new-session -d -s "$SESSION" -n gazebo \
		"arch -x86_64 env $common_env bash -lc 'source \"$setup\" \"$ROOT\" \"$BUILD\"; export GAZEBO_MODEL_PATH=\"$model_path:\$GAZEBO_MODEL_PATH\"; exec gzserver \"$GAZEBO_ROOT/worlds/empty.world\" --verbose'"

	local ready=0
	for _ in {1..50}; do
		if arch -x86_64 env PATH=/usr/local/bin:/usr/bin:/bin GAZEBO_MASTER_URI="$MASTER_URI" \
			/usr/local/bin/gz world -l >/dev/null 2>&1; then
			ready=1
			break
		fi
		sleep 0.2
	done
	((ready)) || {
		tmux capture-pane -p -t "$SESSION:gazebo" -S -80 >&2 || true
		stop
		die "Gazebo did not become ready"
	}

	for sysid in 44 45; do
		local x=0
		[[ "$sysid" == 45 ]] && x=40
		arch -x86_64 env \
			PATH=/usr/local/bin:/usr/local/sbin:/usr/bin:/bin \
			GAZEBO_MASTER_URI="$MASTER_URI" \
			GAZEBO_MODEL_PATH="$model_path" \
			/usr/local/bin/gz model \
			--spawn-file="$RUN_ROOT/plane_lidar_$sysid.sdf" \
			--model-name="plane_lidar_$sysid" \
			-x "$x" -y 0 -z 0.83
	done

	tmux new-window -t "$SESSION" -n sys44 \
		"cd \"$RUN_ROOT/rootfs/43\" && exec arch -x86_64 env $common_env PX4_SIM_MODEL=gazebo-classic_plane_lidar PATH=\"$RUN_ROOT/rootfs/43:/usr/local/bin:/usr/local/sbin:/usr/bin:/bin\" \"$PX4\" -i 43 \"$RUN_ROOT/rootfs/43/etc\""
	tmux new-window -t "$SESSION" -n sys45 \
		"cd \"$RUN_ROOT/rootfs/44\" && exec arch -x86_64 env $common_env PX4_SIM_MODEL=gazebo-classic_plane_lidar PATH=\"$RUN_ROOT/rootfs/44:/usr/local/bin:/usr/local/sbin:/usr/bin:/bin\" \"$PX4\" -i 44 \"$RUN_ROOT/rootfs/44/etc\""

	local px4_ready=0
	for _ in {1..80}; do
		if lsof -nP -iUDP:18613 2>/dev/null | grep -q 'px4' &&
			lsof -nP -iUDP:18614 2>/dev/null | grep -q 'px4'; then
			px4_ready=1
			break
		fi
		sleep 0.25
	done
	((px4_ready)) || {
		stop
		die "PX4 owner MAVLink ports did not become ready"
	}

	# Independent, non-task telemetry streams let both AAGS stations display
	# both vehicles while each station keeps one local cue/owner link.
	tmux send-keys -t "$SESSION:sys44" \
		'mavlink start -x -u 18615 -o 14552 -t 127.0.0.1 -r 4000000' Enter
	tmux send-keys -t "$SESSION:sys45" \
		'mavlink start -x -u 18616 -o 14553 -t 127.0.0.1 -r 4000000' Enter

	echo "Started dual Gazebo lab in tmux session: $SESSION"
	echo "Attach: tmux attach -t $SESSION"
	echo "SYSID 44 -> local cue/owner UDP 18613, observer UDP 18615"
	echo "SYSID 45 -> local cue/owner UDP 18614, observer UDP 18616"
}

action="${1:-start}"
case "$action" in
	start) start ;;
	stop) stop ;;
	status) status ;;
	attach) exec tmux attach -t "$SESSION" ;;
	-h|--help|help) usage ;;
	*) usage >&2; die "unknown action: $action" ;;
esac
