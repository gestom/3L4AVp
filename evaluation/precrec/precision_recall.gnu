set terminal fig color
set size 0.9,0.7
set title "Precision/recall of the SVM and PointNet in experiment I and II" 
set key bottom left 
set ytics 0.2 
set ylabel "Precision [-]"
set xlabel "Recall [-]"
plot [0:1] [0.5:1.05] \
'data_exp_1-prpn.txt' using 2:1 with line lw 2 title 'PointNet - experiment I',\
'data_exp_1-prsvm.txt' using 2:1 with line lw 2 title 'SVM - experiment I',\
'data_exp_2-prpn.txt' using 2:1 with line lw 2 title 'PointNet - experiment II',\
'data_exp_2-prsvm.txt' using 2:1 with line lw 2 title 'SVM - experiment II',\
