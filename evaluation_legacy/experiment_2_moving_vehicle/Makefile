appname := moving_error 

CXX := c++
CXXFLAGS := -std=c++11 -ggdb 
LDLIBS := `pkg-config opencv --libs` 

srcfiles := $(shell find . -name "*.cpp")
objects  := $(patsubst %.cpp, %.o, $(srcfiles))

all: $(appname)

$(appname): $(objects)
	$(CXX) $(CXXFLAGS) $(LDFLAGS) -o $(appname) $(objects) $(LDLIBS)

depend: .depend

.depend: $(srcfiles)
	rm -f ./.depend
	$(CXX) $(CXXFLAGS) -MM $^>>./.depend;

clean:
	rm -f $(objects)
	rm -f *.pdf *.fig *.tmp haze.txt normal.txt
	rm $(appname)

dist-clean: clean
	rm -f *~ .depend

include .depend
