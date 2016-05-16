
LIBS= -lm -largtable2
CXXFLAGS= -fPIC -I. -I./srcChroma -I./ext/include -O3 -Wall -pedantic -std=c++11
#CXXFLAGS= -fPIC -g -ggdb -I. -Wall -W -lm -std=c++11
RM=rm -f
DEMOFLAGS= -lGL -lGLEW `sdl-config --cflags --libs`
#-Wall -pedantic -W

SRCS = $(wildcard src/*.h) $(wildcard src/util/*.cpp) $(wildcard src/util/*.h) $(wildcard src/*.cpp) $(wildcard src/*.c)

ARM_FLAGS=-march=armv6 -marm -mfloat-abi=softfp -mfpu=vfp

FW_DEPS_LIBS=`pkg-config --libs libdc1394-2`
FW_DEPS_CFLAGS=`pkg-config --cflags libdc1394-2` -DHAVE_FIREWIRE_LIB

CV_DEPS_LIBS=`pkg-config --libs opencv`
CV_DEPS_CFLAGS=`pkg-config --cflags opencv` -DUMF_USE_OPENCV

bins: bin/libumf.so bin/libumf.a bin/detect

all-cv-fw: bins

all-cv: LIBS+=$(CV_DEPS_LIBS)
all-cv:	CXXFLAGS+=$(CV_DEPS_CFLAGS)

all-cv-fw: LIBS+= $(CV_DEPS_LIBS) $(FW_DEPS_LIBS)
all-cv-fw: CXXFLAGS+= $(CV_DEPS_CFLAGS) $(FW_DEPS_CFLAGS)

depend: $(SRCS)
	$(CXX) -I./srcChroma -std=c++0x -MM $^ | sed 's/^\(.*\).o: \([^/]*\/\)\1/\2\1.o: \2\1/g' > ./.depend
	
.PHONY: depend all-cv
all: depend all-cv

include .depend

all-cv: bins

#%.o: %.cpp
#	g++ -c $(CXXFLAGS) $^
#
#%.o: %.c
#	gcc -c $(CXXFLAGS) $^

UTIL_SOURCES=$(wildcard src/util/*.cpp)
UTIL_OBJECTS=$(UTIL_SOURCES:.cpp=.o)

LIB_OBJECTS=$(UTIL_OBJECTS) src/decisiontree.o src/edge_dir_detector.o src/edgel_detector.o src/grid_detector.o src/model.o \
src/umf.o src/tracker.o src/marker.o srcChroma/chroma.o

bin/libumf.so: $(LIB_OBJECTS)
	$(CXX) -shared -o $@ $^

bin/libumf.a: $(LIB_OBJECTS)
	ar rcs $@ $^

bin/detect: src/main.o bin/libumf.a
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LIBS)

clean:
	$(RM) src/*.o src/*~ src/util/*~ src/*.gch bin/detect bin/*.so bin/*.a src/util/*.o srcChroma/*.o
