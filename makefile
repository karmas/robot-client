CC= g++
INCLUDES_PCL= -I/usr/include/pcl-1.6/ -I/usr/include/eigen3/  \
	      -I/usr/include/vtk-5.8/
INCLUDES= -I/usr/local/Aria/include -I/usr/local/Aria/ArNetworking/include
CPPFLAGS= -Wall -g $(INCLUDES) $(INCLUDES_PCL)
LIBS_PCL= -lpcl_common -lpcl_features -lpcl_search -lpcl_kdtree \
	 -lpcl_io -lpcl_filters -lpcl_visualization \
	 -lvtkCommon -lvtkFiltering -lvtkRendering \
	 -lboost_thread
LDLIBS= -lpthread -L/usr/local/Aria/lib/ -lAria -lArNetworking -lrt -ldl

SRC = helpers.cc ConfigFileReader.cc MoveRobot.cc OutputHandler.cc \
      client.cc
OBJ = $(SRC:.cc=.o)
HDR = helpers.h ConfigFileReader.h MoveRobot.h OutputHandler.h
PROG = client

$(PROG): $(OBJ) $(HDR) tags
	$(CC) $(OBJ) $(LDLIBS) $(LIBS_PCL) -o $@

clean:
	rm -f *.o a.out $(PROG) tags

tags: $(SRC)
	ctags $(SRC) $(HDR)
