set terminal fig color
set size 3,1
set title "Distance versus error" 
set key top right 
set ytics 0.1 
set xlabel "Distance [m]"
set ylabel "Error [m]"
plot "sensor_fusion-9-stdout.log" using 11:3 
#w p
