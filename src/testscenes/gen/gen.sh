#!/bin/sh

EXPATH=$(dirname "$0")

runSplit() {
if [ -f "$EXPATH"/plane_split ]; then
	p1="$1,$2,$3"
	shift;shift;shift;
	p2="$1,$2,$3"
	shift;shift;shift;
	p3="$1,$2,$3"
	shift;shift;shift;

	"$EXPATH"/plane_split "$p1" "$p2" "$p3" $size
else
	"$EXPATH"/plane_split.sh "$@"
fi
}

FILE=${1:-killeroo}
export size=${2:-100}

if [[ $FILE = "killeroo" ]]; then
    cat "$EXPATH"/killeroo-shaft.smallplanes.before.pbrt  > killeroo-shaft.smallplanes.pbrt

    runSplit -1000 -1000 0      1000 -1000 0      -1000 1000 0    >> killeroo-shaft.smallplanes.pbrt
    runSplit -400 -1000 -1000   -400 1000 -1000   -400 -1000 1000 >> killeroo-shaft.smallplanes.pbrt

    cat "$EXPATH"/killeroo-shaft.smallplanes.after.pbrt >> killeroo-shaft.smallplanes.pbrt
else
    cat "$EXPATH"/bunny-shaft.before.pbrt  > bunny-shaft.smallplanes.pbrt

    runSplit -1000 0 -1000      -1000 0 1000      1000 0 -1000    >> bunny-shaft.smallplanes.pbrt

    cat "$EXPATH"/bunny-shaft.after.pbrt >> bunny-shaft.smallplanes.pbrt  
fi