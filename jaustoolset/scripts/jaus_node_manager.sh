#!/bin/bash

_term() {
	echo "Caught SIGTERM signal!"
	kill -TERM "$child" 2>/dev/null
	echo "exit $child"
}

JTS_RUNNING="$( ps h -C JTSNodeManager )"
if [ "$1" == "start" ]; then
	if [ "$JTS_RUNNING" ]; then
		echo "already running, exit!"
		exit
	fi
	# go back to root of jaustoolset
	JTS_BIN_PATH="$( dirname "$0" )"
	JTS_BIN_PATH="$( dirname "$JTS_BIN_PATH" )"
	trap '_term' TERM INT
	echo "rosrun jaustoolset JTSNodeManager $JTS_BIN_PATH/cfg/nm.cfg"
	cd $JTS_BIN_PATH
	rosrun jaustoolset JTSNodeManager $JTS_BIN_PATH/cfg/nm.cfg &
	child=$!
	echo "JTSNodeManager process id: $child"
	wait "$child"
	exit
fi

if [ "$1" == "stop" ]; then
	if [ "$JTS_RUNNING" ]; then
		echo "kill $JTS_RUNNING"
		echo $JTS_RUNNING | awk '{print $1}' | xargs kill
	else
		echo "no running JAUS NodeManager detected"
	fi
	exit
fi

if [ "$1" == "list" ]; then
	if [ "$JTS_RUNNING" ]; then
		echo "Running JAUS NodeManager:"
		echo "$(ps -f -C JTSNodeManager)"
	else
		echo "no running JAUS NodeManager detected"
	fi
	exit
fi

if [ "$1" == "screen" ]; then
	if [ "$JTS_RUNNING" ]; then
		echo "already running, exit!"
		exit
	fi
	echo "run '$( readlink -f $0 ) start' in an screen"
	screen -L -dmS iop_nodemanager $( readlink -f $0 ) start
	exit

fi

if [ "$1" == "" ] || [ "$1" == "-h" ] || [ "$1" == "--help" ]; then
	echo "Usage:"
	echo "	$0 screen|start|stop|list"
	exit
fi

