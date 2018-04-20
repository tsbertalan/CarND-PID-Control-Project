#!/bin/bash
lastLog="`ls -t ../build/twiddle_* | head -n 1`"
echo "Loading log file $lastLog."
python plot.py "$lastLog"
