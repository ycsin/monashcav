#!/bin/sh

SCRIPTDIR=$(dirname "$(readlink -f "$0")")
grep -rl "static const bool debug = false;" "$SCRIPTDIR/.." | xargs sed -i 's/static const bool debug = false;/static const bool debug = false;/g'
