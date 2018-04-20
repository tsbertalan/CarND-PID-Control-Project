#!/bin/bash
DELAY=10
while true
do
    last="`ls -t ../build/twiddle_* | head -n 1`" 
    python plot.py "$last" &
    child_pid=$!
    echo "Opened $last with PID $child_pid."
    sleep $DELAY
    kill $child_pid
done

