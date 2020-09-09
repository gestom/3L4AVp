function process()
{
	a=1
	cut -f 1,3,4,5,6,7,8,9,10 -d ' '  $1.txt |sed -n $a,1000000000p >svm.tp		
	cat $1.txt |awk '{print $1,$3,$4,$11,$12,$13,$8,$9,$10}'|sed -n $a,1000000000p >cnn.tp 

	./compute cnn.tp cnn.tm 150 0 $(cat cnn.tp|wc -l)
	./compute svm.tp svm.tm 150 0 $(cat cnn.tp|wc -l)

	cat cnn.tm |grep SLW >cnn.tmp
	cat svm.tm |grep SLW >svm.tmp

	gnuplot $1.gnu >$1.fig
	fig2dev -Lpdf $1.fig >$1.pdf

	rm *.tmp
	rm *.tm
	rm *.tp
}

process first
process second 
