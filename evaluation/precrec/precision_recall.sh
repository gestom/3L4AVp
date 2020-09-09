function process()
{
	mkdir tmp
	rm tmp/*
	for i in $(ls $1/gt*|sed s/$1.//);do ./pn_pr $1/$i >tmp/$i;done
	for n in $(seq 1 1000);do for i in $(ls tmp/gt*);do sed -n "$n"p $i;done|awk '{tp=tp+$2;fp=fp+$3;fn=fn+$4}END{printf "%.3f %.3f %i %i %i %.3f\n",(tp+0.000001)/(tp+fp+0.000001),tp/(tp+fn),tp,fp,fn,$7}';done|unbuffer -p awk '(p<$1){p=i$1}{print p,$2,$3,$4,$5,$6}'|tee $1-prpn.txt
		cat $1/sgt*|sort -n >tmp/svm.txt
		./svm_pr tmp/svm.txt >$1-prsvm.txt
		s=$(cat $1-prsvm.txt |cut -f 1,2 -d ' '|uniq|tr . ,|sort -k 2 -n|tr , .|awk '(NR==1){a=0;b=0}{s=s+(a+$1)*($2-b)/2;a=$1;b=$2}END{print s}')
		p=$(cat $1-prpn.txt |cut -f 1,2 -d ' '|uniq|tr . ,|sort -k 2 -n|tr , .|awk '(NR==1){a=0;b=0}{s=s+(a+$1)*($2-b)/2;a=$1;b=$2}END{print s}')
		echo AvgPrec: PN: $p SVM: $s
	}

process data_exp_1 
process data_exp_2

gnuplot precision_recall.gnu >tmp/precrec.fig
fig2dev -Lpdf tmp/precrec.fig >precrec.pdf
rm tmp/*
