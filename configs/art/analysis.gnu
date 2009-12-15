plot \
    "wou_steps.log" using 1:2 title "pos_cmd[0]", \
    "wou_steps.log" using 1:3 title "cur_pos[0]", \
    "wou_steps.log" using 1:6 title "pos_cmd[1]", \
    "wou_steps.log" using 1:7 title "cur_pos[1]", \
    "wou_steps.log" using 1:10 title "pos_cmd[2]", \
    "wou_steps.log" using 1:11 title "cur_pos[2]", \
    "wou_steps.log" using 1:14 title "pos_cmd[3]", \
    "wou_steps.log" using 1:15 title "cur_pos[3]"

plot \
    "wou_steps.log" using 1:2 title "pos_cmd[0]", \
    "wou_steps.log" using 1:3 title "cur_pos[0]", \
    "wou_steps.log" using 1:($4/100) title "match_ac[0]", \
    "wou_steps.log" using 1:($5/10) title "curr_vel[0]"

plot \
    "wou_steps.log_1209" using 1:2 title "pos_cmd[0]", \
    "wou_steps.log_1209" using 1:3 title "cur_pos[0]", \
    "wou_steps.log_1209" using 1:($4/100) title "match_ac[0]", \
    "wou_steps.log_1209" using 1:($5/10) title "curr_vel[0]"


plot \
    "wou_steps.log" using 1:6 title "pos_cmd[1]", \
    "wou_steps.log" using 1:7 title "cur_pos[1]", \
    "wou_steps.log" using 1:($8/100) title "match_ac[1]", \
    "wou_steps.log" using 1:($9/10) title "curr_vel[1]"

plot \
    "wou_steps.log" using 1:10 title "pos_cmd[2]", \
    "wou_steps.log" using 1:11 title "cur_pos[2]", \
    "wou_steps.log" using 1:($12/100) title "match_ac[2]", \
    "wou_steps.log" using 1:($13/10) title "curr_vel[2]"

plot \
    "wou_steps.log" using 1:14 title "pos_cmd[3]", \
    "wou_steps.log" using 1:15 title "cur_pos[3]", \
    "wou_steps.log" using 1:($16/100) title "match_ac[3]", \
    "wou_steps.log" using 1:($17/10) title "curr_vel[3]"

# to plot trajectory planning trace dumped by src/emc/motion/control.c
plot \
    "control.log" using 1:2 title "x", \
    "control.log" using 1:3 title "y", \
    "control.log" using 1:6 title "tp_pos[0]", \
    "control.log" using 1:7 title "tp_pos[1]", \
    "control.log" using 1:10 title "cubic_pos[0]", \
    "control.log" using 1:11 title "cubic_pos[1]"

# to plot trajectory planning trace dumped by src/emc/motion/control.c
plot \
    "control.log" using 1:2 title "x", \
    "control.log" using 1:3 title "y", \
    "control.log" using 1:4 title "z", \
    "control.log" using 1:5 title "a", \
    "control.log" using 1:6 title "tp_pos[0]", \
    "control.log" using 1:7 title "tp_pos[1]", \
    "control.log" using 1:8 title "tp_pos[2]", \
    "control.log" using 1:9 title "tp_pos[3]", \
    "control.log" using 1:10 title "cubic_pos[0]", \
    "control.log" using 1:11 title "cubic_pos[1]", \
    "control.log" using 1:12 title "cubic_pos[2]", \
    "control.log" using 1:13 title "cubic_pos[3]"

# to plot trace dumped by src/emc/kinematics/tp.c
plot \
    "tp.log" using 1:2 title "newaccel", \
    "tp.log" using 1:3 title "newvel", \
    "tp.log" using 1:4 title "cur_vel", \
    "tp.log" using 1:5 title "progress"

