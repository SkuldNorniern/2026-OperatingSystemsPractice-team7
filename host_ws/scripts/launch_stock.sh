#!/usr/bin/env bash
# usage: launch_stock.sh [same options as run_demo.sh]   stock run: run_demo.sh --stock
exec "$(dirname "${BASH_SOURCE[0]}")/run_demo.sh" --stock "$@"
