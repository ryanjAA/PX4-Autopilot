#!/usr/bin/env bash

# Prepare and run PX4 1.14 Gazebo Classic SITL on an Apple Silicon Mac using
# the Intel Homebrew Gazebo installation in /usr/local. This script does not
# use Git and is safe to run repeatedly. It preserves custom MAVLink XML files
# and only installs missing generator support for the branch's pymavlink
# version.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
KIT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
COMMON_PATCH="$SCRIPT_DIR/px4_1_14_macos_sitl.patch"
ARUCO_PATCH="$SCRIPT_DIR/px4_1_14_gazebo_aruco.patch"
TCP_RECONNECT_PATCH="$SCRIPT_DIR/px4_1_14_gazebo_tcp_reconnect.patch"
GIMBAL_PATCH="$SCRIPT_DIR/px4_1_14_mavlink_gimbal_extensions.patch"

ROOT="$PWD"
ACTION="run"
MODEL="plane_lidar"
HEADLESS_VALUE=1
HOME_LAT="33.4370404"
HOME_LON="-99.8158795"
HOME_ALT="457"
GENERATOR_SOURCE=""
GAZEBO_SOURCE=""
SKIP_PYTHON=0
SKIP_PATCHES=0

usage() {
	cat <<'EOF'
Usage: px4_1_14_macos_sitl.sh [options]

Locally patches, prepares, builds, and optionally runs a PX4 1.14 checkout.
No Git commands are used and custom MAVLink definitions are preserved.

Options:
  --root PATH              PX4 checkout to prepare (default: current directory)
  --apply-only             Apply compatibility changes, but do not build
  --build                  Apply changes and build without starting SITL
  --run                    Apply changes, build, and run SITL (default)
  --model NAME             Gazebo Classic model (default: plane_lidar)
  --home-lat VALUE         Home latitude (default: 33.4370404)
  --home-lon VALUE         Home longitude (default: -99.8158795)
  --home-alt VALUE         Home altitude in metres (default: 457)
  --gui                    Start the Gazebo GUI
  --headless               Do not start the Gazebo GUI (default)
  --generator-source PATH  Existing pymavlink generator/C directory to copy
                           only when the target branch is missing it
  --gazebo-source PATH     Populated sitl_gazebo-classic source directory to
                           copy only when the target checkout is missing it
  --skip-python            Do not create/update the local Python environment
  --skip-patches           Only prepare/build; do not edit source files
  -h, --help               Show this help

Examples:
  px4_1_14_macos_sitl.sh --root /path/to/custom-1.14 --run
  px4_1_14_macos_sitl.sh --root /path/to/custom-1.14 --build
EOF
}

die() {
	echo "error: $*" >&2
	exit 1
}

while (($#)); do
	case "$1" in
		--root)
			(($# >= 2)) || die "--root requires a path"
			ROOT="$2"
			shift 2
			;;
		--apply-only)
			ACTION="apply"
			shift
			;;
		--build)
			ACTION="build"
			shift
			;;
		--run)
			ACTION="run"
			shift
			;;
		--model)
			(($# >= 2)) || die "--model requires a value"
			MODEL="$2"
			shift 2
			;;
		--home-lat)
			(($# >= 2)) || die "--home-lat requires a value"
			HOME_LAT="$2"
			shift 2
			;;
		--home-lon)
			(($# >= 2)) || die "--home-lon requires a value"
			HOME_LON="$2"
			shift 2
			;;
		--home-alt)
			(($# >= 2)) || die "--home-alt requires a value"
			HOME_ALT="$2"
			shift 2
			;;
		--gui)
			HEADLESS_VALUE=0
			shift
			;;
		--headless)
			HEADLESS_VALUE=1
			shift
			;;
		--generator-source)
			(($# >= 2)) || die "--generator-source requires a path"
			GENERATOR_SOURCE="$2"
			shift 2
			;;
		--gazebo-source)
			(($# >= 2)) || die "--gazebo-source requires a path"
			GAZEBO_SOURCE="$2"
			shift 2
			;;
		--skip-python)
			SKIP_PYTHON=1
			shift
			;;
		--skip-patches)
			SKIP_PATCHES=1
			shift
			;;
		-h|--help)
			usage
			exit 0
			;;
		*)
			die "unknown option: $1"
			;;
	esac
done

[[ "$(uname -s)" == "Darwin" ]] || die "this compatibility script is for macOS"
[[ -d "$ROOT" ]] || die "PX4 root does not exist: $ROOT"
ROOT="$(cd "$ROOT" && pwd)"
[[ -f "$ROOT/package.xml" && -f "$ROOT/Makefile" ]] || die "not a PX4 checkout: $ROOT"
grep -Eq '<version>1\.14([.<-]|</version>)' "$ROOT/package.xml" ||
	die "the checkout does not identify itself as PX4 1.14"

for resource in "$COMMON_PATCH" "$ARUCO_PATCH" "$TCP_RECONNECT_PATCH" "$GIMBAL_PATCH"; do
	[[ -f "$resource" ]] || die "missing companion file: $resource"
done

PATCH_BIN="/usr/bin/patch"
[[ -x "$PATCH_BIN" ]] || die "macOS patch utility not found"

if [[ -x /usr/local/bin/python3.11 ]]; then
	HOST_PYTHON=/usr/local/bin/python3.11
elif [[ -x /usr/local/bin/python3 ]]; then
	HOST_PYTHON=/usr/local/bin/python3
elif command -v python3 >/dev/null 2>&1; then
	HOST_PYTHON="$(command -v python3)"
else
	die "Python 3 is required"
fi

STAMP="$(date +%Y%m%d-%H%M%S)"
BACKUP_ROOT="$ROOT/.px4-1.14-macos-sitl-backups/$STAMP"
TMP_ROOT="$(mktemp -d "${TMPDIR:-/tmp}/px4-sitl-compat.XXXXXX")"
trap 'rm -rf "$TMP_ROOT"' EXIT

split_patch() {
	local patch_file="$1"
	local output_dir="$2"
	mkdir -p "$output_dir"
	"$HOST_PYTHON" - "$patch_file" "$output_dir" <<'PY'
import pathlib
import re
import sys

source = pathlib.Path(sys.argv[1]).read_text()
output = pathlib.Path(sys.argv[2])
chunks = [chunk for chunk in re.split(r"(?=^diff --git )", source, flags=re.MULTILINE) if chunk.strip()]
for index, chunk in enumerate(chunks):
    (output / f"{index:04d}.patch").write_text(chunk)
PY
}

patch_original_path() {
	sed -n 's|^--- a/||p' "$1" | head -n 1
}

patch_chunk_semantically_satisfied() {
	local target_root="$1"
	local backup_group="$2"
	local chunk="$3"
	local relative
	relative="$(patch_original_path "$chunk")"

	# These files have equivalent, and in the pxh case safer, implementations
	# in the AAGS branch. A textual reverse patch cannot recognize them because
	# the surrounding source also contains MAVLink-M work. Keep these checks
	# deliberately narrow so an unrelated failed hunk is still a hard error.
	[[ "$backup_group" == "px4" ]] || return 1

	case "$relative" in
		platforms/common/px4_log.cpp)
			local file="$target_root/$relative"
			[[ -f "$file" ]] || return 1
			grep -Fq 'snprintf(buf + pos, math::max(max_length - pos, (ssize_t)0), "%s", __px4_log_level_color[level])' "$file" &&
				grep -Fq 'snprintf(buf + sz, math::max(max_length - sz, (ssize_t)0), "%s\n", PX4_ANSI_COLOR_RESET)' "$file" &&
				grep -Fq 'snprintf(buf + sz, math::max(max_length - sz, (ssize_t)0), "%s", PX4_ANSI_COLOR_RESET)' "$file" &&
				[[ "$(grep -Fc 'snprintf(buf + math::min(pos, max_length - (ssize_t)1), 2, "\n")' "$file")" -ge 2 ]]
			;;
		platforms/posix/src/px4/common/px4_daemon/pxh.cpp)
			local file="$target_root/$relative"
			[[ -f "$file" ]] || return 1
			grep -Fq 'std::vector<char *> args;' "$file" &&
				grep -Fq 'args.reserve(words.size() + 1);' "$file" &&
				grep -Fq 'args.push_back(nullptr);' "$file" &&
				grep -Fq '_apps[command](words.size(), args.data())' "$file"
			;;
		src/modules/mavlink/CMakeLists.txt)
			local file="$target_root/$relative"
			[[ -f "$file" ]] || return 1
			grep -Fq 'if(EXISTS "${MAVLINK_GIT_DIR}/.git")' "$file" &&
				grep -Fq 'px4_add_git_submodule(TARGET git_mavlink_v2 PATH "${MAVLINK_GIT_DIR}")' "$file" &&
				grep -Fq 'add_custom_target(git_mavlink_v2)' "$file"
			;;
		*)
			return 1
			;;
	esac
}

apply_patch_set() {
	local patch_file="$1"
	local target_root="$2"
	local backup_group="$3"
	local label="$4"
	local chunk_dir="$TMP_ROOT/$backup_group-chunks"
	local -a forward_chunks=()
	local -a conflict_chunks=()
	local -a equivalent_chunks=()

	split_patch "$patch_file" "$chunk_dir"

	local chunk
	for chunk in "$chunk_dir"/*.patch; do
		# Check reverse first. Pure insertion patches can sometimes also apply
		# forwards twice, while a reverse dry-run reliably identifies them as
		# already present.
		if "$PATCH_BIN" --dry-run -R -s -f -p1 -d "$target_root" <"$chunk" >/dev/null 2>&1; then
			:
		elif "$PATCH_BIN" --dry-run -N -s -f -p1 -d "$target_root" <"$chunk" >/dev/null 2>&1; then
			forward_chunks+=("$chunk")
		elif patch_chunk_semantically_satisfied "$target_root" "$backup_group" "$chunk"; then
			equivalent_chunks+=("$chunk")
		else
			conflict_chunks+=("$chunk")
		fi
	done

	if ((${#conflict_chunks[@]})); then
		echo "$label cannot be applied cleanly. No files from this patch set were changed." >&2
		for chunk in "${conflict_chunks[@]}"; do
			echo "  conflict: $(patch_original_path "$chunk")" >&2
			"$PATCH_BIN" --dry-run -N -f -p1 -d "$target_root" <"$chunk" >&2 || true
		done
		return 1
	fi

	if ((${#equivalent_chunks[@]})); then
		echo "$label: recognized ${#equivalent_chunks[@]} equivalent local compatibility change(s)"
	fi

	if ((${#forward_chunks[@]} == 0)); then
		echo "$label: already applied"
		return 0
	fi

	for chunk in "${forward_chunks[@]}"; do
		local relative
		relative="$(patch_original_path "$chunk")"
		if [[ -n "$relative" && -f "$target_root/$relative" ]]; then
			mkdir -p "$BACKUP_ROOT/$backup_group/$(dirname "$relative")"
			cp -p "$target_root/$relative" "$BACKUP_ROOT/$backup_group/$relative"
		fi
	done

	for chunk in "${forward_chunks[@]}"; do
		"$PATCH_BIN" -N -s -f -p1 -d "$target_root" <"$chunk"
	done

	echo "$label: applied ${#forward_chunks[@]} file patch(es)"
	echo "  backups: $BACKUP_ROOT/$backup_group"
}

main_checkout_root() {
	local git_marker="$ROOT/.git"
	local git_dir=""

	if [[ -f "$git_marker" ]]; then
		git_dir="$(sed -n 's/^gitdir: //p' "$git_marker" | head -n 1)"
	fi

	if [[ "$git_dir" == */.git/worktrees/* ]]; then
		echo "${git_dir%%/.git/worktrees/*}"
	elif [[ "$ROOT" == */.dmux/worktrees/* ]]; then
		echo "${ROOT%%/.dmux/worktrees/*}"
	else
		return 1
	fi
}

copy_gazebo_source_if_needed() {
	local target="$ROOT/Tools/simulation/gazebo-classic/sitl_gazebo-classic"
	if [[ -f "$target/CMakeLists.txt" ]]; then
		# px4_add_git_submodule lists this marker as a build dependency even
		# when GIT_SUBMODULES_ARE_EVIL disables all submodule operations.
		[[ -e "$target/.git" ]] || touch "$target/.git"
		return 0
	fi

	local gazebo_relative="Tools/simulation/gazebo-classic/sitl_gazebo-classic"
	local source=""
	local checkout_root=""
	local worktree_parent=""
	local -a candidates=()

	if [[ -n "$GAZEBO_SOURCE" ]]; then
		candidates+=("$GAZEBO_SOURCE")
	fi

	if [[ "$KIT_ROOT" != "$ROOT" ]]; then
		candidates+=("$KIT_ROOT/$gazebo_relative")
	fi

	checkout_root="$(main_checkout_root || true)"
	if [[ -n "$checkout_root" && "$checkout_root" != "$ROOT" ]]; then
		# A linked worktree often has an empty submodule directory even though
		# the primary checkout has the matching, populated Gazebo source.
		candidates+=("$checkout_root/$gazebo_relative")
		worktree_parent="$checkout_root/.dmux/worktrees"
	fi

	if [[ -n "${PX4_GAZEBO_SOURCE:-}" ]]; then
		candidates+=("$PX4_GAZEBO_SOURCE")
	fi

	local candidate
	for candidate in "${candidates[@]}"; do
		if [[ "$candidate" != "$target" && -f "$candidate/CMakeLists.txt" ]]; then
			source="$candidate"
			break
		fi
	done

	# The primary checkout is preferred, but another populated linked worktree
	# is still a useful automatic fallback when that checkout is sparse.
	if [[ -z "$source" && -d "$worktree_parent" ]]; then
		for candidate in "$worktree_parent"/*/"$gazebo_relative"; do
			if [[ "$candidate" != "$target" && -f "$candidate/CMakeLists.txt" ]]; then
				source="$candidate"
				break
			fi
		done
	fi

	[[ -n "$source" && -f "$source/CMakeLists.txt" ]] ||
		die "Gazebo Classic source is missing; use --gazebo-source with a populated sitl_gazebo-classic directory"

	mkdir -p "$target"
	tar -C "$source" --exclude='.git' -cf - . | tar -C "$target" -xf -
	# This is a local source copy, not a nested checkout. The empty marker is
	# only for PX4's generated build dependency; check_submodules.sh is skipped.
	touch "$target/.git"
	echo "Gazebo Classic source copied locally from $source"
}

if ((SKIP_PATCHES == 0)); then
	apply_patch_set "$COMMON_PATCH" "$ROOT" "px4" "PX4 1.14 macOS compatibility"
	copy_gazebo_source_if_needed
	GAZEBO_ROOT="$ROOT/Tools/simulation/gazebo-classic/sitl_gazebo-classic"
	apply_patch_set "$ARUCO_PATCH" "$GAZEBO_ROOT" "gazebo" "Gazebo OpenCV/Aruco compatibility"
	apply_patch_set "$TCP_RECONNECT_PATCH" "$GAZEBO_ROOT" "gazebo-tcp" "Gazebo TCP reconnect compatibility"
else
	GAZEBO_ROOT="$ROOT/Tools/simulation/gazebo-classic/sitl_gazebo-classic"
	[[ -f "$GAZEBO_ROOT/CMakeLists.txt" ]] || die "Gazebo Classic source is not populated"
fi

VENV="$ROOT/.venv"

prepare_python() {
	if ((SKIP_PYTHON)); then
		[[ -x "$VENV/bin/python" ]] || die "--skip-python was used but $VENV/bin/python is missing"
		return 0
	fi

	if [[ -x "$VENV/bin/python" ]] && ! file "$VENV/bin/python" | grep -q 'x86_64'; then
		mkdir -p "$BACKUP_ROOT/python"
		mv "$VENV" "$BACKUP_ROOT/python/venv-wrong-architecture"
	fi

	if [[ ! -x "$VENV/bin/python" ]]; then
		echo "Creating Intel Python environment: $VENV"
		arch -x86_64 "$HOST_PYTHON" -m venv "$VENV"
	fi

	arch -x86_64 "$VENV/bin/python" -m pip install future
	arch -x86_64 "$VENV/bin/python" -m pip install -r "$ROOT/Tools/setup/requirements.txt"
}

resolve_generator_source() {
	local candidate="$1"
	if [[ -d "$candidate/include_v2.0" ]]; then
		echo "$candidate"
	elif [[ -d "$candidate/generator/C/include_v2.0" ]]; then
		echo "$candidate/generator/C"
	elif [[ -d "$candidate/pymavlink/generator/C/include_v2.0" ]]; then
		echo "$candidate/pymavlink/generator/C"
	else
		return 1
	fi
}

prepare_mavlink_generator() {
	local target="$ROOT/src/modules/mavlink/mavlink/pymavlink/generator/C"
	[[ -d "$target/include_v2.0" ]] && {
		echo "MAVLink C generator: present; custom MAVLink files left unchanged"
		return 0
	}

	local source=""
	if [[ -n "$GENERATOR_SOURCE" ]]; then
		source="$(resolve_generator_source "$GENERATOR_SOURCE")" ||
			die "cannot find generator/C below --generator-source $GENERATOR_SOURCE"
	else
		local version_file="$ROOT/src/modules/mavlink/mavlink/pymavlink/__init__.py"
		[[ -f "$version_file" ]] || die "pymavlink version file is missing; use --generator-source"
		local version
		version="$(sed -n "s/^__version__ = ['\"]\([^'\"]*\)['\"].*/\1/p" "$version_file" | head -n 1)"
		[[ -n "$version" ]] || die "cannot determine the vendored pymavlink version; use --generator-source"

		local package_dir="$TMP_ROOT/pymavlink-$version"
		echo "MAVLink C generator is missing; obtaining matching pymavlink $version from PyPI"
		if ! arch -x86_64 "$VENV/bin/python" -m pip install --no-deps --target "$package_dir" "pymavlink==$version"; then
			die "could not obtain pymavlink $version; rerun with --generator-source pointing to a matching generator/C"
		fi
		source="$(resolve_generator_source "$package_dir")" ||
			die "the pymavlink $version package did not contain generator/C"
	fi

	mkdir -p "$(dirname "$target")"
	cp -R "$source" "$target"
	echo "MAVLink C generator restored without changing message definitions"
}

prepare_python
prepare_mavlink_generator

needs_gimbal_extensions() {
	grep -q 'gimbal_device_id' "$GAZEBO_ROOT/src/gazebo_gimbal_controller_plugin.cpp" 2>/dev/null ||
		grep -q 'gimbal_device_id' "$GAZEBO_ROOT/src/gazebo_camera_manager_plugin.cpp" 2>/dev/null
}

if ((SKIP_PATCHES == 0)) && needs_gimbal_extensions; then
	COMMON_XML="$ROOT/src/modules/mavlink/mavlink/message_definitions/v1.0/common.xml"
	MISSING_GIMBAL_FIELDS="$("$HOST_PYTHON" - "$COMMON_XML" <<'PY'
import sys
import xml.etree.ElementTree as ET

wanted = {
    "CAMERA_INFORMATION",
    "GIMBAL_DEVICE_INFORMATION",
    "GIMBAL_DEVICE_ATTITUDE_STATUS",
}
root = ET.parse(sys.argv[1]).getroot()
missing = []
for message in root.findall(".//message"):
    name = message.get("name")
    if name in wanted and not any(field.get("name") == "gimbal_device_id" for field in message.findall("field")):
        missing.append(name)
print(" ".join(sorted(missing)))
PY
)"
	if [[ -n "$MISSING_GIMBAL_FIELDS" ]]; then
		if [[ "$MISSING_GIMBAL_FIELDS" == "CAMERA_INFORMATION GIMBAL_DEVICE_ATTITUDE_STATUS GIMBAL_DEVICE_INFORMATION" ]]; then
			apply_patch_set "$GIMBAL_PATCH" "$ROOT" "mavlink-gimbal" "MAVLink fields required by the local Gazebo plugins"
		else
			die "custom common.xml has a partial gimbal update ($MISSING_GIMBAL_FIELDS missing); merge the three extension fields manually"
		fi
	else
		echo "MAVLink gimbal extensions: already compatible"
	fi
fi

if [[ "$ACTION" == "apply" ]]; then
	echo "PX4 1.14 local SITL preparation complete: $ROOT"
	exit 0
fi

BUILD_DIR="$ROOT/build/px4_sitl_default"
if [[ -f "$BUILD_DIR/CMakeCache.txt" ]]; then
	CACHE_ARCH="$(sed -n 's/^CMAKE_OSX_ARCHITECTURES:[^=]*=//p' "$BUILD_DIR/CMakeCache.txt" | head -n 1)"
	CACHE_GENERATOR="$(sed -n 's/^CMAKE_GENERATOR:[^=]*=//p' "$BUILD_DIR/CMakeCache.txt" | head -n 1)"
	if [[ "$CACHE_ARCH" != "x86_64" || "$CACHE_GENERATOR" != "Unix Makefiles" ]]; then
		mkdir -p "$ROOT/build"
		mv "$BUILD_DIR" "$ROOT/build/px4_sitl_default.incompatible-$STAMP"
		echo "Preserved incompatible build cache as build/px4_sitl_default.incompatible-$STAMP"
	fi
fi

prepare_gazebo_mavlink_dialect() {
	local mavlink_root="$ROOT/src/modules/mavlink/mavlink"
	local development_xml="$mavlink_root/message_definitions/v1.0/development.xml"
	local output_dir="$BUILD_DIR/mavlink"

	[[ -f "$development_xml" ]] || die "Gazebo Classic requires the MAVLink development dialect: $development_xml"
	mkdir -p "$output_dir"

	# The AAGS firmware keeps mavlink_m as its primary dialect. Gazebo's
	# camera/gimbal plugins independently use commands from development.xml,
	# so generate that companion header without changing the board dialect.
	arch -x86_64 env PYTHONHASHSEED=0 \
		"$VENV/bin/python" "$mavlink_root/pymavlink/tools/mavgen.py" \
		--lang C --wire-protocol 2.0 \
		--output "$output_dir" \
		"$development_xml" >"$BUILD_DIR/mavgen_development.log"

	[[ -f "$output_dir/development/mavlink.h" ]] ||
		die "MAVLink development dialect generation did not produce its header"
	echo "Gazebo MAVLink development dialect: generated alongside primary mavlink_m"
}

prepare_gazebo_mavlink_dialect

CMAKE_ARGS_VALUE="${CMAKE_ARGS:-} -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -Wno-dev -DCMAKE_OSX_ARCHITECTURES=x86_64"
RUN_ENV=(
	"PATH=/usr/local/bin:/usr/local/sbin:/usr/bin:/bin"
	"PKG_CONFIG=/usr/local/bin/pkg-config"
	"GIT_SUBMODULES_ARE_EVIL=1"
	"NO_NINJA_BUILD=1"
	"CMAKE_ARGS=$CMAKE_ARGS_VALUE"
	"PX4_HOME_LAT=$HOME_LAT"
	"PX4_HOME_LON=$HOME_LON"
	"PX4_HOME_ALT=$HOME_ALT"
)

if ((HEADLESS_VALUE)); then
	RUN_ENV+=("HEADLESS=1")
fi

if [[ "$ACTION" == "build" ]]; then
	RUN_ENV+=("DONT_RUN=1")
fi

echo "PX4 root: $ROOT"
echo "Target: gazebo-classic_$MODEL"
echo "Architecture: x86_64"

cd "$ROOT"
exec arch -x86_64 env "${RUN_ENV[@]}" make px4_sitl "gazebo-classic_$MODEL"
