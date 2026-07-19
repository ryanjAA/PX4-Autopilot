#! /usr/bin/env bash

# script directory
DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

# Reinstall if --reinstall set
REINSTALL_FORMULAS=""
# Install simulation tools?
INSTALL_SIM=""

# Parse arguments
for arg in "$@"
do
	if [[ $arg == "--reinstall" ]]; then
		REINSTALL_FORMULAS=$arg
	elif [[ $arg == "--sim-tools" ]]; then
		INSTALL_SIM=$arg
	fi
done

if ! command -v brew &> /dev/null
then
	# install Homebrew if not installed yet
	/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install.sh)"
fi

# Install px4-dev formula
if [[ $REINSTALL_FORMULAS == "--reinstall" ]]; then
	echo "Re-installing PX4 general dependencies (homebrew px4-dev)"

	# confirm Homebrew installed correctly
	brew doctor

	brew tap PX4/px4
	brew reinstall px4-dev
	brew install ncurses
else
	if brew ls --versions px4-dev > /dev/null; then
		echo "px4-dev already installed"
	else
		echo "Installing PX4 general dependencies (homebrew px4-dev)"
		brew tap PX4/px4
		brew install px4-dev
		brew install ncurses
	fi
fi

# Python dependencies
echo "Installing PX4 Python3 dependencies"
ROOT_DIR="$(git -C "$DIR" rev-parse --show-toplevel 2>/dev/null || echo "$DIR")"
VENV_DIR="$ROOT_DIR/.venv"

if [[ ! -d "$VENV_DIR" ]]; then
	echo "Creating Python virtual environment at $VENV_DIR"
	python3 -m venv "$VENV_DIR"
fi

# We need to have future to install pymavlink later.
"$VENV_DIR/bin/python" -m pip install future
"$VENV_DIR/bin/python" -m pip install -r "${DIR}/requirements.txt"

# Optional, but recommended additional simulation tools:
if [[ $INSTALL_SIM == "--sim-tools" ]]; then
	if brew ls --versions px4-sim > /dev/null; then
		brew install px4-sim
	elif [[ $REINSTALL_FORMULAS == "--reinstall" ]]; then
		brew reinstall px4-sim
	fi
fi

echo "All set! PX4 toolchain installed!"
