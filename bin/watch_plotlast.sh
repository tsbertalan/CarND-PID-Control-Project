#!/bin/bash
DELAY=16
while true
do
    # Get the former active window id.
    active_wid=`xprop -root 32x '\t$0' _NET_ACTIVE_WINDOW | cut -f 2`

    # Make the plot.
    last="`ls -t ../build/twiddle_* | head -n 1`" 
    python plot.py "$last" &
    child_pid=$!
    sleep 4
    echo "Opened $last with PID $child_pid."

    # Maximize.
    # xdotool keydown alt key space
    # sleep .1
    # xdotool keyup alt
    # xdotool key x
    IFS='\n'
    for plot_wid in `wmctrl -l -p | grep $child_pid | cut -f 1 -d " "`
    do
        # Move to the left monitor/side of screen.
        wmctrl -ir $plot_wid -e 0,10,10,-1,-1
        # sleep .1

        # Maximize
        wmctrl -ir $plot_wid -b add,maximized_vert,maximized_horz
    done

    # Give the original window the focus again.
    wmctrl -iR $active_wid
    
    # Wait, then kill the plot window.
    sleep $DELAY
    kill $child_pid
done

