set terminal fig color
set size 0.9,0.7
set title "Localisation error over time (second pedestrian, sliding average)" 
set key top right 
set ytics 0.2 
set xlabel "Time [s]"
set ylabel "Average error [m]"
plot [0:19.9] [0:0.79] 'cnn.tmp'using ($0/30):3 with lines title 'Standalone lidar' lw 2,\
'' using ($0/30):4 with lines title 'Standalone PointNet' lw 2,\
'svm.tmp' using ($0/30):4 with lines title 'Standalone SVM' lw 2,\
'cnn.tmp' using ($0/30):5 with lines title 'PointNet + weighted filter' lw 2,\
