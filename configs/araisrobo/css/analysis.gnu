# to plot CSS trace with (X, Y) positions
plot "css.gnuplot" using 2:3 title "(X, Y)"
# to set the equal scale for X and Y 
set size square

# to plot 
plot "css.gnuplot" using 1:4 title "css_cmd", \
     "css.gnuplot" using 1:5 title "css_c", \
     "css.gnuplot" using 1:6 title "css_a"

# to plot 
plot "css.gnuplot" using 1:7 title "c_rps", \
     "css.gnuplot" using 1:8 title "a_rps"
