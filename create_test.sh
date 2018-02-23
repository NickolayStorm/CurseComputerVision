# /bin/bash

FILES=data/*

REPORT=report.csv

touch $REPORT

for file in $FILES 
do
    name=$(basename $file)
    echo "Work with $name"
    d=$(cmake-build-debug/OpenCVCurse --data "data/$name")
    t=$(cmake-build-debug/OpenCVCurseTst --data "tested_data/$name")
    echo $name';'$d';'$t >> $REPORT
done


