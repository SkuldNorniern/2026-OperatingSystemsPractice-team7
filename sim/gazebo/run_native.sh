#!/bin/zsh
set -e
cd "${0:A:h}"
if [[ ! -x .venv/bin/python ]]; then
  print -u2 "Run ./setup_native.sh first to install simulator dependencies."
  exit 1
fi
exec .venv/bin/python native_sim/run.py "$@"
