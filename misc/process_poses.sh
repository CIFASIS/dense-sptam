#!/bin/bash
POSES_FILE=poses.txt
# Always begin at 1 - Dense node assumption
# NOTE: rosbag start with seq=1 but gt-data starts at 0
FIRST=1

usage () {
	echo "$0 { merge_poses | rename_clouds }"
	exit 1
}

merge_poses () {
	last=$1
	for i in $(seq --format="%06.0f" $FIRST $last); do
		pose=cloud_${i}.txt
		if [ -f $pose ]; then
			cat $pose >> $POSES_FILE
		else
			echo "0 0 0 0 0 0 0 0 0 0 0 0" >> $POSES_FILE
		fi
	done
}

rename_clouds () {
	last=$1
	for i in $(seq $FIRST $last); do
		from=cloud_$(printf "%06.0f" ${i}).pcd
		to_id=$((i - 1))
		to=$(printf "%06.0f" ${to_id}).pcd
		[ -f $from ] && mv $from $to
	done
}

[ $# -lt 2 ] && usage

case "$1" in
  merge_poses)
	merge_poses $2
	;;
  rename_clouds)
	rename_clouds $2
	;;
  all)
	rename_clouds $2
	merge_poses $2
	;;
  *)
	usage
	;;
esac
