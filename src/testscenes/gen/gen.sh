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

if [ -f "$EXPATH/plane_split" ]; then
	EXEC="$EXPATH/plane_split"
else
	EXEC="$EXPATH/plane_split.sh"
fi

export size=${1:-100}

cat "$EXPATH"/killeroo-shaft.smallplanes.before.pbrt  > killeroo-shaft.smallplanes.pbrt

runSplit -1000 -1000 0      1000 -1000 0      -1000 1000 0    >> killeroo-shaft.smallplanes.pbrt
runSplit -400 -1000 -1000   -400 1000 -1000   -400 -1000 1000 >> killeroo-shaft.smallplanes.pbrt

cat "$EXPATH"/killeroo-shaft.smallplanes.after.pbrt >> killeroo-shaft.smallplanes.pbrt
