#!/usr/bin/bash
eval "$(cat $(dirname "${BASH_SOURCE[0]}")/header.sh)"

PATCHES_DIR=$TOP_DIR/patches/

CHECK_STRING="already_patched"

cd $WORKSPACE_DIR

echo "Applying Patches..."

for f in $PATCHES_DIR/*.patch; do
    FILE_TO_CHECK=$(grep +++ $f -r | awk '{print $2}' | head -n 1)
    if grep -q $CHECK_STRING $FILE_TO_CHECK; then
	echo "Patch already applied, skipping $f"
    else
	echo "Patching $f"
	patch -p0 < $f
    fi
done

cd $TOP_DIR
