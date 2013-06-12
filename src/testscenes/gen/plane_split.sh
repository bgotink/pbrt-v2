#!/bin/bash

SIZE=${size:-100};

point1x=$1;shift
point1y=$1;shift
point1z=$1;shift

point2x=$1;shift
point2y=$1;shift
point2z=$1;shift

point3x=$1;shift
point3y=$1;shift
point3z=$1;shift

log() {
    echo -e $* >&2;
}

result() {
    echo -e $*;
}

calc() {
    echo $(( $1 ));
}

max() {
    if [ $1 -gt $2 ]; then
        echo $1;
    else
        echo $2;
    fi
}

dir1x=$(calc "$point2x - $point1x");
dir1y=$(calc "$point2y - $point1y");
dir1z=$(calc "$point2z - $point1z");

dir2x=$(calc "$point3x - $point1x");
dir2y=$(calc "$point3y - $point1y");
dir2z=$(calc "$point3z - $point1z");

unset point2x point2y point2z
unset point3x point3y point3z

log "dir1 = [$dir1x $dir1y $dir1z]"
log "dir2 = [$dir2x $dir2y $dir2z]"

log

log "["
log "\t$point1x $point1y $point1z"
log "\t"$(calc "$point1x + $dir1x")" "$(calc "$point1y + $dir1y")" "$(calc "$point1z + $dir1z")
log "\t"$(calc "$point1x + $dir1x + $dir2x")" "$(calc "$point1y + $dir1y + $dir2y")" "$(calc "$point1z + $dir1z + $dir2z")
log "\t"$(calc "$point1x + $dir2x")" "$(calc "$point1y + $dir2y")" "$(calc "$point1z + $dir2z")
log "]"

log

count_x=$(max $(calc "$dir1x / $SIZE") $(calc "$dir2x / $SIZE"));
count_y=$(max $(calc "$dir1y / $SIZE") $(calc "$dir2y / $SIZE"));
count_z=$(max $(calc "$dir1z / $SIZE") $(calc "$dir2z / $SIZE"));

log "count_x=$count_x"
log "count_y=$count_y"
log "count_z=$count_z"

count=$(max $count_x $(max $count_y $count_z))

step1x=$(calc "$dir1x / $count")
step1y=$(calc "$dir1y / $count")
step1z=$(calc "$dir1z / $count")

step2x=$(calc "$dir2x / $count")
step2y=$(calc "$dir2y / $count")
step2z=$(calc "$dir2z / $count")

log "count=$count";

log

log "step1 = [$step1x $step1y $step1z]"
log "step2 = [$step2x $step2y $step2z]"

x=0;

result "Shape \"trianglemesh\""

result "\t\"point P\" ["
while [ $x -le $count ]; do
    y=0;
    while [ $y -le $count ]; do
        result "\t\t"$(calc "$point1x + ($x * $step1x) + ($y * $step2x)")" "$(calc "$point1y + ($x * $step1y) + ($y * $step2y)")" "$(calc "$point1z + ($x * $step1z) + ($y * $step2z)")
        y=$(calc "$y+1");
    done
    x=$(calc "$x+1");
done
result "\t]"

x=0;

result "\t\"integer indices\" ["
while [ $x -lt $count ]; do
    y=0;
    while [ $y -lt $count ]; do
        idx0=$(calc "$x * ($count + 1) + $y");
        idx1=$(calc "($x + 1) * ($count + 1) + $y");
        idx2=$(calc "($x + 1) * ($count + 1) + ($y + 1)");
        idx3=$(calc "$x * ($count + 1) + ($y + 1)");
        
        result "\t\t$idx0 $idx1 $idx2 $idx2 $idx3 $idx0"
        
        y=$(calc "$y + 1");
    done
    x=$(calc "$x + 1");
done
result "\t]"
