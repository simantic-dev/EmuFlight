#!/bin/sh
# usage: run.sh [extra sim args]   (pty/CLI use: see cli_pty.py)
set -e
cd "$(dirname "$0")"
sed "s#PEERS#$PWD/peers#" f405.board > f405.run.board
exec sim run --scenario scenario.yaml "$@"
