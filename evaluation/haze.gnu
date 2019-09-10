set terminal fig color
set size 0.9,0.7
set title "Average pedestrian localisation error over time" 
set key top left 
set ytics 0.1 
set xlabel "Time [s]"
set ylabel "Average error [m]"
set label "Normal conditions" at 126.5,0.05 
set label "Hazy conditions" at 136,0.05
plot [120:150] [0:0.3] \
'sum.tmp'using (($0)/30-7+120):3 with lines title 'Lidar estimate [m]' lw 2,\
'' using (($0)/30-7+120):4 with lines title 'Radar estimate [m]' lw 2,\
'' using (($0)/30-7+120):5 with lines title 'Weighted filter [m]' lw 2,\
'' using (($0)/30-7+120):6 with lines title 'Switching filter [m]' lw 2,\
'' using (($0)/30-7+120):7 with lines title 'Deep radar [m]' lw 2,\
'' using (($0)/30-7+120):8 with lines title 'Weighted Deep filter [m]' lw 2,\
'' using (($0)/30-7+120):9 with lines title 'Switching Deep filter [m]' lw 2,\
'fog.lin' using ($1+120):2 with lines lt 2 lw 2 lc 0 notitle
