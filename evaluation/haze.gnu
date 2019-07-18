set terminal fig color
set size 0.9,0.7
set title "Average pedestrian localisation error over time" 
set key top left 
set ytics 0.1 
set xlabel "Time [s]"
set ylabel "Average error [m]"
set label "Normal conditions" at 6.5,0.05 
set label "Hazy conditions" at 16,0.05
plot [0:30] [0:0.22] \
'sum.tmp'using (($0)/30-7):3 with lines title 'Lidar estimate [m]' lw 2,\
'' using (($0)/30-7):4 with lines title 'Radar estimate [m]' lw 2,\
'' using (($0)/30-7):5 with lines title 'Weighting filter [m]' lw 2,\
'' using (($0)/30-7):6 with lines title 'Switching filter [m]' lw 2,\
'fog.lin' using 1:2 with lines lt 2 lw 2 lc 0 notitle
