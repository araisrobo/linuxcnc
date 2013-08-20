# to plot CSS trace with (X, Y) positions
plot "css.gnuplot" using 2:3 title "(X, Y)"
# to set the equal scale for X and Y 
set size square

# to plot 
plot "css.gnuplot" using 1:4 title "Requested Constant Surface Speed", \
     "css.gnuplot" using 1:5 title "Surface Speed of Spindle-C", \
     "css.gnuplot" using 1:6 title "Surface Speed of Spindle-A"

# to plot 
plot "css.gnuplot" using 1:7 title "Rotation speed of Spindle-C", \
     "css.gnuplot" using 1:8 title "Rotation speed of Spindle-A"
