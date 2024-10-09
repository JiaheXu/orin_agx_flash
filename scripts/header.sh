#!/usr/bin/bash

SCRIPTS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TOP_DIR=$SCRIPTS_DIR/../

source $TOP_DIR/setup.config
source $TOP_DIR/config/$version.env
