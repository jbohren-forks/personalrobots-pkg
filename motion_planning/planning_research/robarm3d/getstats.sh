#!/bin/bash


for i in $(seq 51 200); 
    do
        echo "=================================="
        echo "           Run: $i"
        echo "=================================="
        rosrun robarm3d planPathShell env_examples/robarm/stats.cfg $i
done

echo "DONE"

exit 0

