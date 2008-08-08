#!/bin/bash
for i in `ls -d 2008*`; do
  echo "tarring up $i"
  tar cf tars/$i.tar $i
done
