#!/bin/bash

PKGS="rosstage map_server fake_localization bullet nav_view"

echo ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>" | tee -a build_for_stage.log
date | tee -a build_for_stage.log
echo "going to build: $PKGS" | tee -a build_for_stage.log
echo "--------------------------------------------------" | tee -a build_for_stage.log

rm -f build_for_stage.failed
for pkg in $PKGS
  do
  (if ! rosmake $pkg; then echo "$pkg FAILED" && touch build_for_stage.failed; fi) 2>&1 | tee -a build_for_stage.log
  test -f build_for_stage.failed && exit 42
  echo "FINISHED $pkg" | tee -a build_for_stage.log
  echo "--------------------------------------------------" | tee -a build_for_stage.log
done

echo "--------------------------------------------------" | tee -a build_for_stage.log
date | tee -a build_for_stage.log
echo "finished building: $PKGS" | tee -a build_for_stage.log
echo "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" | tee -a build_for_stage.log
