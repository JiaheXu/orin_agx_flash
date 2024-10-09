#!/usr/bin/bash
eval "$(cat $(dirname "${BASH_SOURCE[0]}")/header.sh)"

$SCRIPTS_DIR/clone.sh
$SCRIPTS_DIR/apply_patches.sh
$SCRIPTS_DIR/build_binaries.sh
$SCRIPTS_DIR/apply_binaries.sh
$SCRIPTS_DIR/flash.sh

cd $TOP_DIR
