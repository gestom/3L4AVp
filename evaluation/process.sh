function compute()
{
	./sensor_fusion $1.txt a.tmp
	cat a.tmp |grep SUM >sum.tmp
	gnuplot $1.gnu >$1.fig
	fig2dev -Lpdf $1.fig >$1.pdf
	okular $1.pdf
}

make
compute normal
compute haze
