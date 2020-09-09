FLAGS = -ggdb

main: dbscan
	g++ $(FLAGS) main.cpp -o main dbscan.o

simple:
	g++ $(FLAGS) simple.cpp -o simple 

dbscan:
	g++ $(FLAGS) -c -o dbscan.o dbscan.cpp
