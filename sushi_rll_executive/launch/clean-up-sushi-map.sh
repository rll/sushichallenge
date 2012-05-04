#!/bin/bash

set -u
set -e
set -x

# USAGE: ./bin/clean-up-sushi-map.sh ~/ros/full-body-nav/octomap_server/

octomap_server_dir="$1"

"$octomap_server_dir/src/octomap_eraser_cli.py" -2.0 -2.0 -1.0 -1.0 0.25 1.0 # by door to outside
"$octomap_server_dir/src/octomap_eraser_cli.py" 1.0 -1.85 -1.0 3.0 0.25 1.0 # by door to outside

