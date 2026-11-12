#!/usr/bin/env bash
# stock run: the committed rust_base HEAD (or TEAM7_REV), with the stock launch defaults.
# usage: launch_stock.sh [--replay BAG_DIR] [name:=value ...]   same options as launch_team7.sh
exec "$(dirname "${BASH_SOURCE[0]}")/launch_team7.sh" --stock "$@"
