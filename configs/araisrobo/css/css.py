#!/usr/bin/python
import re
import math

f1 = open('./css.gnuplot', 'w+')
for line in open('tp_css.log'):
    line.strip()
    a_r = 50
    a = re.split('\s+', line)
    if (len(a) != 8):
        print "in-complete input list, a[]: ", a
        continue
    dt = int(a[1])
    c_r = float(a[2])
    c_rev = float(a[3])
    x = c_r * math.cos(c_rev * 2 * math.pi)
    y = c_r * math.sin(c_rev * 2 * math.pi)
    css_cmd = float(a[4])
    css_c_vel = float(a[5]) # css contributed by AXIS_C
    css_a_vel = float(a[6]) # css contributed by AXIS_A
    if c_r == 0: 
        c_rps = 0
    else:
        c_rps = css_c_vel / (2 * math.pi * c_r)
    a_rps = css_a_vel / (2 * math.pi * a_r)
    
    print >> f1, dt, x, y, css_cmd, css_c_vel, css_a_vel, c_rps, a_rps
