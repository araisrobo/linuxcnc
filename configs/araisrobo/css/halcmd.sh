#!/bin/bash

#  tclsh ${HOME}/proj/emc2-dev/tcl/bin/halshow.tcl

echo "bp-tick: " `halcmd getp wou.bp-tick`

# for a in 0 1 2 3 4 5; do
for a in 0; do
echo "pulse_pos[$a]: " `halcmd getp wou.stepgen.$a.pulse_pos`
echo "enc_pos[$a]:   " `halcmd getp wou.stepgen.$a.enc_pos`
echo "ferror[$a]:    " `halcmd getp wou.stepgen.$a.ferror-flag`
echo "cmd_fbs[$a]:   " `halcmd getp wou.stepgen.$a.cmd-fbs`
echo "joint.$a.pos-cmd:         " `halcmd getp joint.$a.pos-cmd`
echo "joint.$a.pos-fb:          " `halcmd getp joint.$a.pos-fb`
echo "joint.$a.motor-pos-cmd:   " `halcmd getp joint.$a.motor-pos-cmd`
echo "joint.$a.motor-pos-fb:    " `halcmd getp joint.$a.motor-pos-fb`
echo "joint.$a.probed-pos:      " `halcmd getp joint.$a.probed-pos`
done

echo "wou.sync.analog_ref_level:    " `halcmd getp wou.sync.analog_ref_level`
echo "wou.sync.in.timeout:          " `halcmd getp wou.sync.in.timeout`
echo "wou.sync.in.wait_type:        " `halcmd getp wou.sync.in.wait_type`
echo "wou.sync.in.index:            " `halcmd getp wou.sync.in.index`
echo "wou.sync.in.trigger:          " `halcmd getp wou.sync.in.trigger`

# for a in 0 1 2 3 4 5 6 7; do
# echo "debug-$a:      " `halcmd getp wou.debug.value-$a`
# done

echo "debug-0,  j[3]req_vel       ......" `halcmd getp wou.debug.value-0`
echo "debug-1,  j[3]jog_vel       ......" `halcmd getp wou.debug.value-1`
echo "debug-2,  risc_cmd      ......" `halcmd getp wou.debug.value-2`
echo "debug-3,  rcmd_seq_num  ......" `halcmd getp wou.debug.value-3`
echo "debug-4,  probe_cmd     ......" `halcmd getp wou.debug.value-4`
echo "debug-5,  probe_status  ......" `halcmd getp wou.debug.value-5`
echo "debug-6,  machine_status......" `halcmd getp wou.debug.value-6`
echo "debug-7,  usb_status    ...   " `halcmd getp wou.debug.value-7`
# echo "debug-5,  j[1].vel_err  ......" `halcmd getp wou.debug.value-5`
# echo "debug-6,  j[2].req_vel        " `halcmd getp wou.debug.value-6`
# echo "debug-7,  j[2].cur_vel  ......" `halcmd getp wou.debug.value-7`
# echo "debug-8,  j[2].cur_acc        " `halcmd getp wou.debug.value-8`
# echo "debug-9,  j[2].acc_req  ......" `halcmd getp wou.debug.value-9`
# echo "debug-10, j[2].vel_err        " `halcmd getp wou.debug.value-10`
# echo "debug-11, j[1].max_vel  ......" `halcmd getp wou.debug.value-11`
# echo "debug-12, j[1].max_acc        " `halcmd getp wou.debug.value-12`
# echo "debug-13, j[1].max_jerk ......" `halcmd getp wou.debug.value-13`
# echo "debug-14, j[2].max_vel        " `halcmd getp wou.debug.value-14`
# echo "debug-15, j[2].max_acc  ......" `halcmd getp wou.debug.value-15`
# echo "debug-16, j[2].max_jerk       " `halcmd getp wou.debug.value-16`
# echo "debug-17, j[1].ferror         " `halcmd getp wou.debug.value-17`
# echo "debug-18, j[1].max_ferror     " `halcmd getp wou.debug.value-18`
# echo "debug-19, j[2].ferror         " `halcmd getp wou.debug.value-19`
# echo "debug-20, j[2].max_ferror     " `halcmd getp wou.debug.value-20`
# echo "debug-21, AHC_STATE           " `halcmd getp wou.debug.value-21`
# echo "debug-22, ctrl_state          " `halcmd getp wou.debug.value-22`
# echo "debug-23, start_i             " `halcmd getp wou.debug.value-23`
# echo "debug-24, test_run_i          " `halcmd getp wou.debug.value-24`
# echo "debug-25, motion_blending_o   " `halcmd getp wou.debug.value-25`

