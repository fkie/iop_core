#!/bin/bash

if [ ! $JTS_COMMON_PATH ]; then
    echo "JTS_COMMON_PATH is not set!"
else
	export JTS_RUNNING="$( ps h -C NodeManager )"
	if [ "$1" == "start" ]; then
		if [ "$JTS_RUNNING" ]; then
			echo "already running, exit!"
			exit
		fi
		# go back from JTS_ROOT/GUI/templates/Common to root
		export JTS_ROOT="$( dirname "$JTS_COMMON_PATH" )"
		export JTS_ROOT="$( dirname "$JTS_ROOT" )"
		export JTS_ROOT="$( dirname "$JTS_ROOT" )"
		echo "start from $JTS_ROOT/nodeManager/bin"
		export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$JTS_ROOT/nodeManager/bin
		cd $JTS_ROOT/nodeManager/bin && ./NodeManager nm.cfg
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
			echo "$(ps -f -C NodeManager)"
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
fi

