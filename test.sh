#!/bin/bash

getcap /usr/bin/ip | grep cap_net_admin1
if [ "$?" != "0" ]
then
	sudo setcap cap_net_admin=pe /usr/bin/ip
fi


kill_slcand(){
	pids=$(pgrep slcand)
	if [ "$pids" != "" ]
	then
		#echo kill slcand
		kill $pids
		sleep 2
	fi
}

start_slcand(){
	if [ -c /dev/ttyACM0 ] && [ -c /dev/ttyACM1 ]
	then
		#echo start slcand
		slcand -t hw -S 1000000 /dev/ttyACM0 can0
		slcand -t hw -S 1000000 /dev/ttyACM1 can1
		sleep 1
	fi
}

#(1694260187.809712) can0 001#2A265FDD5A1A22C5
generate_msgs(){
	echo generating $1 msgs ...
	rm -f can.txt
	for ((i = 1; i <= $1; i++))
	do
		#id=$(printf "%03X" $((RANDOM % 2048)))
		if ((i > 2047)); then
			id=$(printf "%08X" $i)
		else
			id=$(printf "%03X" $i)
		fi
		data=$(printf "%04X%04X%04X%04X" $((RANDOM)) $((RANDOM)) $((RANDOM)) $((RANDOM)))
		#data="0123456789abcdef"
		#echo $id $data
		echo "(0000000000.000000) can0 $id#$data" >> can.txt
	done
	echo
}

can_up(){
#echo can0 and can1 up speed $1
ip link set down can0
ip link set down can1

ip link set can0 type can bitrate $1
ip link set can0 txqueuelen $2
ip link set up can0

ip link set can1 type can bitrate $1
ip link set can1 txqueuelen $2
ip link set up can1
}

can_down(){
#echo can0 and can1 down
ip link set down can0
ip link set down can1
}

start_dump(){
#echo start dumping on can0 and can1
candump can0 -n $1 -ta -T 1000 > can0.txt &
candump0_pid=$!
candump can1 -n $1 -ta -T 1000 > can1.txt &
candump1_pid=$!
}

report(){
	t1=`head -1 can0.txt | awk '{ print $1}' | sed 's/(//' | sed 's/)//'`
	t2=`tail -1 can0.txt | awk '{ print $1}' | sed 's/(//' | sed 's/)//'`
	ttx=`echo $t2 - $t1 | bc`

	t1=`head -1 can1.txt | awk '{ print $1}' | sed 's/(//' | sed 's/)//'`
	t2=`tail -1 can1.txt | awk '{ print $1}' | sed 's/(//' | sed 's/)//'`
	trx=`echo $t2 - $t1 | bc`

	awk '{$1=""; $2=""; print $0}' can0.txt > can0.mtxt
	awk '{$1=""; $2=""; print $0}' can1.txt > can1.mtxt

	TXNUMS=`cat can0.txt | wc -l `
	RXNUMS=`cat can1.txt | wc -l `
	diff -u can0.mtxt can1.mtxt > can.diff
	diffret=$?
	#cat can0.txt
	#cat can1.txt
	cat can.diff
	rm -f can0.txt can1.txt can0.mtxt can1.mtxt can.diff

	if [ "$diffret" == "0" ] && [ "$1" == "$TXNUMS" ] && [ "$1" == "$RXNUMS" ] 
	then
		echo OK: All $1 msgs sent in $ttx secs and received in $trx secs
	else
		echo ERROR: NUMS=$1, TXNUMS=$TXNUMS in $ttx secs, RXNUMS=$RXNUMS in $trx secs
	fi
	echo
}

NUMS=5000
#slcand dosn't control tx flow
if [ -c /dev/ttyACM0 ] && [ -c /dev/ttyACM1 ]
then
	NUMS=100
fi

generate_msgs $NUMS

for SPEED in 2000000 1000000 500000 250000 125000 100000 50000 20000
do
	if [ -c /dev/ttyACM0 ] && [ -c /dev/ttyACM1 ]
	then

		if [ "$SPEED" == "2000000" ] || [ "$SPEED" == "20000" ]
		then
			continue
		fi
	fi

	kill_slcand
	start_slcand

	can_up $SPEED $NUMS
	sleep 0.5

	start_dump $NUMS
	sleep 0.5

	echo sending $NUMS msgs to can0 at $SPEED bit/sec ...
	canplayer -t -I can.txt

	echo "wait ..."
	wait $candump0_pid
	wait $candump1_pid

	can_down
	sleep 0.5

	kill_slcand

	report $NUMS
done

rm -f can.txt


