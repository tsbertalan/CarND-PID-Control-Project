#!/bin/bash
watch "grep succeeded \"\`ls -t ../build/twiddle_* | head -n 1\`\""
