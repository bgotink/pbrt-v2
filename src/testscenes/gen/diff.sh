#!/bin/bash


getsize() {
	shift;shift;shift;
	shift;shift;shift;
	shift;shift;shift;
	echo "$1";
}

call_c() {
	if [ ! -f ./plane_split ]; then
		echo "no compiled C version available" >&2;
		return;
	fi

	p1="$1,$2,$3"; shift; shift; shift;
	p2="$1,$2,$3"; shift; shift; shift;
	p3="$1,$2,$3"; shift; shift; shift;

	echo "C" >&2
	time ./plane_split "$p1" "$p2" "$p3" "$1" > /dev/null
}

call_java() {
	if [ ! -f ./PlaneSplit.class ]; then
		echo "No compiled Java version available" >&2;
		return;
	fi

	p1="$1,$2,$3"; shift; shift; shift;
	p2="$1,$2,$3"; shift; shift; shift;
	p3="$1,$2,$3"; shift; shift; shift;

	echo "Java" >&2
	time java PlaneSplit "$p1" "$p2" "$p3" "$1" > /dev/null
}

call_bash() {
	if [ ! -f ./plane_split.sh ]; then
		echo "No BASH version available" >&2
		return
	fi

	echo "BASH" >&2
	export size=$(getsize "$@")
	time ./plane_split.sh "$@" > /dev/null
}

call_c "$@"
echo;echo;
call_java "$@"
echo;echo;
call_bash "$@"
