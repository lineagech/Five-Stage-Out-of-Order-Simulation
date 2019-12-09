SIMULATOR = simulator

SIMULATOR_SOURCES := $(wildcard src/*.cpp)
SIMULATOR_OBJS := $(SIMULATOR_SOURCES:%.cpp=%.o)

JSON_LIB=libjsoncpp.a

CPPFLAGS= -std=c++11 -g -D_GLIBCXX_DEBUG

%.o: %.cpp Makefile
	$(CXX) $(CPPFLAGS) -c  -I include/ $< -o $@
all: $(SIMULATOR)

$(SIMULATOR): $(SIMULATOR_OBJS)
	$(CXX) $(CPPFLAGS)  $^ -o $@ libjsoncpp.a -I include/

debug:
	$(CXX) -std=c++11 -g  $^ -o $@ libjsoncpp.a -I include/

clean:
	rm -rf *~ src/*.o $(SIMULATOR) *.out
