function process()
{
	a=60			#system initialisation
	cut -f 1,3,4,5,6,7,8,9,10 -d ' '  $1 |sed -n $a,100000p >svm.tp		
	cat $1 |awk '{print $1,$3,$4,$11,$12,$13,$8,$9,$10}'|sed -n $a,100000p >cnn.tp 

	./compute cnn.tp cnn.tm 300
	./compute svm.tp svm.tm 300

	cat cnn.tm |grep SLW >cnn.tmp
	cat svm.tm |grep SLW >svm.tmp

	gnuplot plot.gnu >plot.fig
	fig2dev -Lpdf plot.fig >plot.pdf

	rm tabledata.tmp
	grep ERR cnn.tm|sed -n 900,3780p |grep -v nan|awk '{i=i+1;a=a+$3;b=b+$4;c=c+$5;d=d+$6}END{printf "%.1f %.1f %.1f %.1f\n", 100*a/i,100*b/i,100*c/i,100*d/i}' >>tabledata.tmp
	grep ERR cnn.tm|sed -n 3780,6000p|grep -v nan|awk '{i=i+1;a=a+$3;b=b+$4;c=c+$5;d=d+$6}END{printf "%.1f %.1f %.1f %.1f\n", 100*a/i,100*b/i,100*c/i,100*d/i}' >>tabledata.tmp
	grep ERR svm.tm|sed -n 900,3780p |grep -v nan|awk '{i=i+1;a=a+$3;b=b+$4;c=c+$5;d=d+$6}END{printf "%.1f %.1f %.1f %.1f\n", 100*a/i,100*b/i,100*c/i,100*d/i}' >>tabledata.tmp
	grep ERR svm.tm|sed -n 3780,6000p|grep -v nan|awk '{i=i+1;a=a+$3;b=b+$4;c=c+$5;d=d+$6}END{printf "%.1f %.1f %.1f %.1f\n",  100*a/i,100*b/i,100*c/i,100*d/i}' >>tabledata.tmp

	e=1;
	cp table.txt table.tmp
	for i in RAW.LIDAR.NORMAL RAW.PN.NORMAL KF.PN.NORMAL SF.PN.NORMAL RAW.LIDAR.HAZY RAW.PN.HAZY KF.PN.HAZY SF.PN.HAZY RAW.LIDAR.NORMAL RAW.SVM.NORMAL KF.SVM.NORMAL SF.SVM.NORMAL RAW.LIDAR.HAZY RAW.SVM.HAZY KF.SVM.HAZY SF.SVM.HAZY;
	do 
		v=$(cat tabledata.tmp |sed s/\ /\\n/g|sed -n "$e"p);e=$(($e+1));
		cat table.tmp|sed s/"$i"/"$v"/ >table.tm
		mv table.tm table.tmp
	done
	cp table.tmp table.tex
	rm *.tmp
	rm *.tm
	rm *.tp
}
make
process $1.log 
