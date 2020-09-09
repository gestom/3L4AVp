CXX := c++
FLAGS := -std=c++11 -ggdb 
OC := `pkg-config opencv --libs` 

all: pn_pr svm_pr

pn_pr: dbscan
	g++ $(FLAGS) pn_pr.cpp -o pn_pr dbscan.o

svm_pr:
	g++ $(FLAGS) svm_pr.cpp -o svm_pr 

dbscan:
	g++ $(FLAGS) -c -o dbscan.o dbscan.cpp

clean:
	rm -f *.log *.tmp *.tp *. *.tm *.fig *.pdf *.o *.fig *.pdf
	rm svm_pr pn_pr 
