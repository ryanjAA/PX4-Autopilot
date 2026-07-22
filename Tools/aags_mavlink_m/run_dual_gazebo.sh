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
WORLD="${AAGS_DUAL_GAZEBO_WORLD:-$GAZEBO_ROOT/worlds/empty.world}"
PRESERVE_CUE_STATE="${AAGS_DUAL_PRESERVE_CUE_STATE:-0}"
OPEN_OWNER="${AAGS_DUAL_OPEN_OWNER:-0}"
HOME_LAT="${PX4_HOME_LAT:-33.4370404}"
HOME_LON="${PX4_HOME_LON:--99.8158795}"
HOME_ALT="${PX4_HOME_ALT:-457}"
PYTHON="$ROOT/.venv/bin/python"
PX4="$BUILD/bin/px4"

usage() {
	cat <<'EOF'
Usage: run_dual_gazebo.sh [start|stop|status|configure|attach]

start   Start headless Gazebo and PX4 SYSIDs 44/45 (default)
stop    Cleanly stop only this runner's tmux session
status  Show tmux panes and allocated simulator/GCS ports
configure  Reassert and verify the owner IDs and test action permission
attach  Attach an interactive terminal to both PX4 shells

Port allocation:
  SYSID 44: PX4 instance 43, simulator TCP 4603, local cue/owner UDP 18613,
            observer UDP 18615
  SYSID 45: PX4 instance 44, simulator TCP 4604, local cue/owner UDP 18614,
            observer UDP 18616

Optional environment:
  AAGS_DUAL_GAZEBO_WORLD selects another Gazebo Classic world, such as
  Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/windy.world.
  AAGS_DUAL_PRESERVE_CUE_STATE=1 preserves pending and active cue state.
  The default clears it so each lab start begins without a stale cue.
  AAGS_DUAL_OPEN_OWNER=1 sets MAV_M_SRC_SYS=-1 and MAV_M_CTL_SYS=-1 on
  both vehicles. The default keeps strict GCS254/SYS44 and GCS253/SYS45
  ownership.
EOF
}

die() {
	echo "error: $*" >&2
	exit 1
}

session_exists() {
	tmux has-session -t "$SESSION" 2>/dev/null
}

window_exists() {
	local window="$1"
	tmux list-windows -t "$SESSION" -F '#{window_name}' 2>/dev/null \
		| grep -Fxq "$window"
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
		# This test-only runner is movement-ready after explicit owner
		# acceptance. PX4 still requires armed, airborne AUTO_LOITER and all
		# safety, source, freshness, setpoint, and geofence gates.
		echo 'param set MAV_M_ACTION 2'
	} >"$directory/px4-rc.params"
	chmod +x "$directory/px4-rc.params"
}

configure_live_vehicle() {
	local pane="$1"
	local vehicle_system="$2"
	local owner_system="$3"
	local assignment
	local marker_base="aags_config_${pane}_$$_${RANDOM}"
	local begin_marker="${marker_base}_begin"
	local done_marker="${marker_base}_done"

	# Unique harmless unknown commands delimit only this configuration run.
	# PXH has no echo command, and its invalid-command response is synchronous.
	tmux send-keys -t "$SESSION:$pane" "$begin_marker" Enter
	for assignment in \
		"MAV_M_SRC_SYS $owner_system" \
		"MAV_M_SRC_CMP 190" \
		"MAV_M_CTL_SYS $owner_system" \
		"MAV_M_CTL_CMP 190" \
		"MAV_M_INST 0" \
		"MAV_M_CTL_INST 0" \
		"MAV_M_ACTION 2"; do
		tmux send-keys -t "$SESSION:$pane" "param set $assignment" Enter
	done
	tmux send-keys -t "$SESSION:$pane" 'param save' Enter
	tmux send-keys -t "$SESSION:$pane" 'param show MAV_SYS_ID' Enter
	tmux send-keys -t "$SESSION:$pane" 'param show MAV_M_SRC_SYS' Enter
	tmux send-keys -t "$SESSION:$pane" 'param show MAV_M_CTL_SYS' Enter
	tmux send-keys -t "$SESSION:$pane" 'param show MAV_M_ACTION' Enter
	tmux send-keys -t "$SESSION:$pane" "$done_marker" Enter

	local output ready=0
	for _ in {1..50}; do
		output="$(tmux capture-pane -p -t "$SESSION:$pane" -S -260)"
		if printf '%s\n' "$output" \
			| grep -Fx "Invalid command: $done_marker" >/dev/null; then
			ready=1
			break
		fi
		sleep 0.1
	done
	((ready)) || die "$pane did not finish applying the lab parameters"
	local segment
	segment="$(printf '%s\n' "$output" | awk \
		-v begin="Invalid command: $begin_marker" \
		-v done="Invalid command: $done_marker" \
		'$0 == begin { capture = 1; next }
		 $0 == done { if (capture) exit }
		 capture { print }')"
	[[ -n "$segment" ]] || die "$pane configuration output could not be isolated"
	printf '%s\n' "$segment" \
		| grep -E "MAV_SYS_ID .*: ${vehicle_system}$" >/dev/null \
		|| die "$pane is not PX4 system $vehicle_system"
	printf '%s\n' "$segment" \
		| grep -E "MAV_M_SRC_SYS .*: ${owner_system}$" >/dev/null \
		|| die "$pane did not apply source owner GCS $owner_system"
	printf '%s\n' "$segment" \
		| grep -E "MAV_M_CTL_SYS .*: ${owner_system}$" >/dev/null \
		|| die "$pane did not apply control owner GCS $owner_system"
	printf '%s\n' "$segment" \
		| grep -E 'MAV_M_ACTION .*: 2$' >/dev/null \
		|| die "$pane did not apply MAV_M_ACTION 2"
}

configure() {
	session_exists || die "tmux session $SESSION is not running"
	window_exists sys44 || die "PX4 sys44 pane is missing"
	window_exists sys45 || die "PX4 sys45 pane is missing"
	local sys44_owner=254
	local sys45_owner=253
	if [[ "$OPEN_OWNER" == 1 ]]; then
		sys44_owner=-1
		sys45_owner=-1
	fi
	configure_live_vehicle sys44 44 "$sys44_owner"
	configure_live_vehicle sys45 45 "$sys45_owner"
	if [[ "$OPEN_OWNER" == 1 ]]; then
		echo "Verified: SYS44 and SYS45 accept any nonzero AAGS system on instance 0; MAV_M_ACTION=2"
	else
		echo "Verified: GCS254 owns SYS44; GCS253 owns SYS45; MAV_M_ACTION=2"
	fi
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
	[[ -f "$WORLD" ]] || die "Gazebo world is missing: $WORLD"
	[[ -x /usr/local/bin/gzserver && -x /usr/local/bin/gz ]] ||
		die "Intel Gazebo Classic is required in /usr/local/bin"

	if session_exists; then
		die "tmux session $SESSION is already running; use '$0 status' or '$0 stop'"
	fi
	[[ "$PRESERVE_CUE_STATE" == 0 || "$PRESERVE_CUE_STATE" == 1 ]] \
		|| die "AAGS_DUAL_PRESERVE_CUE_STATE must be 0 or 1"
	[[ "$OPEN_OWNER" == 0 || "$OPEN_OWNER" == 1 ]] \
		|| die "AAGS_DUAL_OPEN_OWNER must be 0 or 1"

	mkdir -p "$RUN_ROOT/models" "$RUN_ROOT/rootfs/43" "$RUN_ROOT/rootfs/44"
	if [[ "$PRESERVE_CUE_STATE" == 0 ]]; then
		# Pending and active cues are intentionally durable in normal firmware.
		# A deterministic test lab starts clean unless preservation is explicit.
		rm -f \
			"$RUN_ROOT/rootfs/43/mavlink_m_state.bin" \
			"$RUN_ROOT/rootfs/43/.mavlink_m_state.tmp" \
			"$RUN_ROOT/rootfs/44/mavlink_m_state.bin" \
			"$RUN_ROOT/rootfs/44/.mavlink_m_state.tmp"
	fi
	# GCS254 owns and cues SYS44; GCS253 owns and cues SYS45. A handover
	# between the two AAGS stations is accepted into the receiving contact
	# store before that local station creates the vehicle cue.
	local sys44_owner=254
	local sys45_owner=253
	if [[ "$OPEN_OWNER" == 1 ]]; then
		sys44_owner=-1
		sys45_owner=-1
	fi
	write_instance_params "$RUN_ROOT/rootfs/43" "$sys44_owner" 14551
	write_instance_params "$RUN_ROOT/rootfs/44" "$sys45_owner" 14550
	generate_model 44 4603 14603
	generate_model 45 4604 14604

	local setup="$ROOT/Tools/simulation/gazebo-classic/setup_gazebo.bash"
	local model_path="$RUN_ROOT/models:$GAZEBO_ROOT/models"
	local common_env="PATH=/usr/local/bin:/usr/local/sbin:/usr/bin:/bin GAZEBO_MASTER_URI=$MASTER_URI GAZEBO_MODEL_PATH=$model_path PX4_HOME_LAT=$HOME_LAT PX4_HOME_LON=$HOME_LON PX4_HOME_ALT=$HOME_ALT"

	tmux new-session -d -s "$SESSION" -n gazebo \
		"arch -x86_64 env $common_env bash -lc 'source \"$setup\" \"$ROOT\" \"$BUILD\"; export GAZEBO_MODEL_PATH=\"$model_path:\$GAZEBO_MODEL_PATH\"; exec gzserver \"$WORLD\" --verbose'"

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
		tmux capture-pane -p -t "$SESSION:sys44" -S -120 >&2 || true
		tmux capture-pane -p -t "$SESSION:sys45" -S -120 >&2 || true
		tmux capture-pane -p -t "$SESSION:gazebo" -S -120 >&2 || true
		stop
		die "PX4 owner MAVLink ports did not become ready"
	}

	# Independent, non-task telemetry streams let both AAGS stations display
	# both vehicles while each station keeps one local cue/owner link.
	tmux send-keys -t "$SESSION:sys44" \
		'mavlink start -x -u 18615 -o 14552 -t 127.0.0.1 -r 4000000' Enter
	tmux send-keys -t "$SESSION:sys45" \
		'mavlink start -x -u 18616 -o 14553 -t 127.0.0.1 -r 4000000' Enter

	# Reassert after full startup and persist the intended owner mapping. This
	# repairs a reused rootfs whose parameters were changed by an earlier
	# cross-owner test instead of trusting stale parameters.bson contents.
	configure

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
	configure) configure ;;
	attach) exec tmux attach -t "$SESSION" ;;
	-h|--help|help) usage ;;
	*) usage >&2; die "unknown action: $action" ;;
esac
