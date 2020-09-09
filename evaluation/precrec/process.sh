rm tmp/*
for i in $(ls $1/gt*|sed s/$1.//);do ./main $1/$i >tmp/$i;done
for n in $(seq 1 1000);do for i in $(ls tmp/gt*);do sed -n "$n"p $i;done|awk '{tp=tp+$2;fp=fp+$3;fn=fn+$4}END{print (tp+0.000001)/(tp+fp+0.000001),tp/(tp+fn),tp,fp,fn,$7}';done|unbuffer -p awk '(p<$1){p=i$1}{print p,$2,$3,$4,$5,$6}'|tee $1-prpn.txt
cat $1/sgt*|sort -n >tmp/svm.txt
./simple tmp/svm.txt >$1-prsvm.txt
