#!/bin/sh

EXPATH=$(dirname "$0")

export size=${1:-100}

cat "$EXPATH"/killeroo-shaft.smallplanes.before.pbrt  > killeroo-shaft.smallplanes.pbrt

"$EXPATH"/plane_split.sh -1000 -1000 0      1000 -1000 0      -1000 1000 0    >> killeroo-shaft.smallplanes.pbrt
"$EXPATH"/plane_split.sh -400 -1000 -1000   -400 1000 -1000   -400 -1000 1000 >> killeroo-shaft.smallplanes.pbrt

cat "$EXPATH"/killeroo-shaft.smallplanes.after.pbrt >> killeroo-shaft.smallplanes.pbrt
