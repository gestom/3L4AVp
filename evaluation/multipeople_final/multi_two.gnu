set terminal fig color
set size 0.9,0.7
set title "Average pedestrian localisation error over time - second person" 
set key spacing 0.9
set key font ",10"
set key bottom right
set ytics 0.1 
set xlabel "Time [s]"
set ylabel "Average error [m]"
#set label "Normal conditions" at 30,0.05 
#set label "Hazy conditions" at 55,0.05
plot [0:33] [0.0:0.95] 'sum.tmp' using ($0/30):3 with lines title 'Lidar estimate [m]' lw 2,\
'' using ($0/30):4 with lines title 'SVM estimate [m]' lw 2,\
'' using ($0/30):5 with lines title 'PointNet estimate [m]' lw 2,\
'' using ($0/30):6 with lines title 'Weighted SVM filter [m]' lw 2,\
'' using ($0/30):7 with lines title 'Switching SVM filter  [m]' lw 2,\
'' using ($0/30):8 with lines title 'Weighted PN filter [m]' lw 2,\
'' using ($0/30):9 with lines title 'Switching PN filter [m]' lw 2,\
#'fog.txt' using 1:2 with lines lt 2 lw 2 lc 0 notitle
