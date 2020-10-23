# Name: Nibras Alam & Al Hisham Anik
# ID: 1617818 & 1585385
# CMPUT 275, Winter 2020
#
# Assignment 2: Driving Route Finder Part 2
#
# Consulted C++ Documentation; Used Lecture Slides

CC=g++
OBJS=dijkstra.o server.o digraph.o serialport.o
CFLAGS=-c -Wall -O2
LFLAGS=-static
PROGRAM=server

$(PROGRAM): $(OBJS)
	$(CC) $(OBJS) -o $(PROGRAM) $(LFLAGS)

dijkstra.o: dijkstra.cpp
	$(CC) dijkstra.cpp -o dijkstra.o $(CFLAGS)

server.o: server.cpp
	$(CC) server.cpp -o server.o $(CFLAGS)

digraph.o: digraph.cpp
	$(CC) digraph.cpp -o digraph.o $(CFLAGS)

serialport.o: serialport.cpp
	$(CC) serialport.cpp -o serialport.o $(CFLAGS)	

clean:
	@rm -f $(OBJS)
	@rm -f $(PROGRAM)
