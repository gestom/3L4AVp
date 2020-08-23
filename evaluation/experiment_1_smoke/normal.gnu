set terminal fig color
set size 0.9,0.7
set title "Average pedestrian localisation error over time" 
set key top right 
set ytics 0.1 
set xlabel "Time [s]"
set ylabel "Average error [m]"
#set label "Normal conditions" at 30,0.05 
#set label "Hazy conditions" at 55,0.05
plot [0:110] [0:0.28] 'sum.tmp' using ($0/30):3 with lines title 'Lidar estimate [m]' lw 2,\
'' using ($0/30):4 with lines title 'Radar estimate [m]' lw 2,\
'' using ($0/30):5 with lines title 'Weighted filter [m]' lw 2,\
'' using ($0/30):6 with lines title 'Switching filter [m]' lw 2,\
'' using ($0/30):7 with lines title 'Deep radar [m]' lw 2,\
'' using ($0/30):8 with lines title 'Weighted Deep filter [m]' lw 2,\
'' using ($0/30):9 with lines title 'Switching Deep filter [m]' lw 2,\
#'fog.txt' using 1:2 with lines lt 2 lw 2 lc 0 notitle
