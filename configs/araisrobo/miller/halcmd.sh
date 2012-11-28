#!/bin/bash

# echo "homing:           " `halcmd getp wou.motion.homing`
# 
# for a in 0 1 2 3; do
# echo "joint.$a.risc-probe-vel:      " `halcmd getp joint.$a.risc-probe-vel`
# echo "joint.$a.risc-probe-pin:      " `halcmd getp joint.$a.risc-probe-pin`
# echo "joint.$a.risc-probe-type:     " `halcmd getp joint.$a.risc-probe-type`
# echo "joint.$a.home-sw-id:          " `halcmd getp joint.$a.home-sw-id`
# echo "joint.$a.homing:              " `halcmd getp joint.$a.homing`
# done
# 

for a in 0; do
echo "pulse_pos[$a]:    " `halcmd getp wou.stepgen.$a.pulse_pos`
echo "enc_pos[$a]:      " `halcmd getp wou.stepgen.$a.enc_pos`
echo "rawcount32[$a]:   " `halcmd getp wou.stepgen.$a.rawcount32`
echo "cmd-fbs[$a]:      " `halcmd getp wou.stepgen.$a.cmd-fbs`
done

for a in 0 1 2 3 4 5 6 7; do
echo "debug-$a:         " `halcmd getp wou.debug.value-$a`
done
