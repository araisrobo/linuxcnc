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
    "wou_steps.log" using 1:($4/10000) title "match_ac[0]", \
    "wou_steps.log" using 1:($5/10) title "curr_vel[0]"

plot \
    "wou_steps.log" using 1:6 title "pos_cmd[1]", \
    "wou_steps.log" using 1:7 title "cur_pos[1]", \
    "wou_steps.log" using 1:($8/10000) title "match_ac[1]", \
    "wou_steps.log" using 1:($9/10) title "curr_vel[1]"

plot \
    "wou_steps.log" using 1:10 title "pos_cmd[2]", \
    "wou_steps.log" using 1:11 title "cur_pos[2]", \
    "wou_steps.log" using 1:($12/10000) title "match_ac[2]", \
    "wou_steps.log" using 1:($13/10) title "curr_vel[2]"

plot \
    "wou_steps.log" using 1:14 title "pos_cmd[3]", \
    "wou_steps.log" using 1:15 title "cur_pos[3]", \
    "wou_steps.log" using 1:($16/10000) title "match_ac[3]", \
    "wou_steps.log" using 1:($17/10) title "curr_vel[3]"
