
function compute()
{
  cut -f 1-9 -d ' ' $1.txt > $1.tmp
	./multipeople_error $1.tmp a.tmp
	cat a.tmp |grep SUM >sum.tmp
	gnuplot $1.gnu >$1.fig
	fig2dev -Lpdf $1.fig >$1.pdf
	okular $1.pdf
}

make
compute multi_one
compute multi_two
#compute normal  
#compute haze 
#rm *.tmp
