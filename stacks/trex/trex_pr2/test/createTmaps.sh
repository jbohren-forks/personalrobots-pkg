#!/bin/bash

echo "Create topomap symlinks."

FILES="willow.tmap.outlet_overrides.xml willow.tmap.door_overrides.xml willow.tmap"

for file in $FILES ; do
    rm -f `rospack find trex_pr2`/test/$file
    echo "ln -s `rospack find willow_maps`/$file `rospack find trex_pr2`/test/$file"
    ln -s `rospack find willow_maps`/$file `rospack find trex_pr2`/test/$file
done
