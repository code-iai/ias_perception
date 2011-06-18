ifeq ($(shell arch), i686)
BIT = 32
endif
ifeq ($(shell arch), x86_64)
BIT = 64
endif

CC = g++
GCC= gcc
EXTRA = 
INCDIR =-I. `pkg-config --cflags opencv` -I../include -I../include
LDDIR = -L./lib
CFLAGS = -O2 -g3 -Wall -DLINUX $(EXTRA) $(LDDIR)
LDLIBS =
LDFLAGS = $(LDLIBS) -lm `pkg-config --libs opencv` `pkg-config --libs gtk+-2.0` 
OBJECTS=bspline.o ccd.o
$(OBJECTS): %.o: %.cpp
	$(CC) $(INCDIR) -c $(CFLAGS) $< -o $@
PROGRAMS=snake example ros_to_openCv ccd2 ccd_panin
all:$(PROGRAMS)
clean:
	@rm -rf $(PROGRAMS) *.o $(PROGRAMS) ../data/bspline*
ros_to_openCv:ros_to_openCv.cpp
	$(CC) $(INCDIR) $(CFLAGS)  -o $@ $< $(LDFLAGS)
snake:snake.cpp
	$(CC) $(INCDIR) $(CFLAGS)  -o $@ $< $(LDFLAGS)
em:em.cpp
	$(CC) $(INCDIR) $(CFLAGS)  -o $@ $< $(LDFLAGS)
curve:curve.cpp $(OBJECTS)
	$(CC) $(INCDIR) $(CFLAGS)  -o $@ $(OBJECTS) $< $(LDFLAGS)
kmeans:kmeans.cpp
	$(CC) $(INCDIR) $(CFLAGS)  -o $@ $< $(LDFLAGS)
example:example.cpp $(OBJECTS) auto_init.o
	$(CC) $(INCDIR) $(CFLAGS)  -o $@ $@.cpp auto_init.o $(OBJECTS) $(LDFLAGS)
ccd_tracker:ccd_tracker.cpp $(OBJECTS) auto_init.o
	$(CC) $(INCDIR) $(CFLAGS)  -o $@ $@.cpp auto_init.o $(OBJECTS) $(LDFLAGS)
contour:contour.cpp auto_init.o
	$(CC) $(INCDIR) $(CFLAGS)  -o $@ $@.cpp auto_init.o  $(LDFLAGS)

auto_init.o:auto_init.cpp
	$(CC) $(INCDIR) $(CFLAGS) -c -o auto_init.o auto_init.cpp
sift:sift.c auto_init.o
	$(GCC) $(INCDIR) $(CFLAGS)  -o $@ $@.c auto_init.o $(LDFLAGS)
ccd:ccd.cpp ccd.h $(OBJECTS)
	$(CC) $(INCDIR) $(CFLAGS)  -o $@ $@.cpp $(OBJECTS) $(LDFLAGS)
matrix:matrix.cpp $(OBJECTS)
	$(CC) $(INCDIR) $(CFLAGS)  -o $@ $@.cpp $(OBJECTS) $(LDFLAGS)

recompile:clean all

.PHONY: check-syntax
check-syntax:
	$(CC) $(INCDIR) $(CFLAGS) -Wall -Wextra -pedantic -fsyntax-only $(CHK_SOURCES)
