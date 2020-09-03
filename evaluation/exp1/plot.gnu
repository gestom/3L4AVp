set terminal fig color
set size 0.9,0.7
set title "Pedestrian localisation error over time (10s sliding average)" 
set key top left 
set ytics 0.2 
set xlabel "Time [s]"
set ylabel "Average error [m]"
set label "Normal conditions" at 50,0.2 
set label "Hazy conditions" at 95,0.2
plot [0:139] [0:.95] 'cnn.tmp'using ($0/30):3 with lines title 'Standalone lidar' lw 2,\
'' using ($0/30):4 with lines title 'Standalone PointNet' lw 2,\
'svm.tmp' using ($0/30):4 with lines title 'Standalone SVM' lw 2,\
'cnn.tmp' using ($0/30):5 with lines title 'PointNet + weighted filter' lw 2,\
'fog2.lin' using 1:2 with lines lt 2 lw 2 lc 0 notitle
