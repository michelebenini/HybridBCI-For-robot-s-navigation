OBJS = src/main.o
HEADERS = include/parameters.h
EXE = bin/sim.o
all: $(EXE) 
setting = -1   
OS := $(shell uname)




setting = 1
CC = g++ -std=c++11 
AR = ar rc
LIBS = `pkg-config --cflags --libs opencv`
INC = -Iinclude/


# ---------------------------------------------------------------------
# Rules
# ---------------------------------------------------------------------
CFLAGS = 
##CFLAGS = -Wall -g -O0 
RM = rm -rf

.SUFFIXES:
.SUFFIXES: .o .c .cpp
.c.o :
	$(CC) $(CFLAGS) $(INC) -c $< -o $@
.cpp.o :
	$(CC) $(CFLAGS) $(INC) -c $< -o $@

$(EXE): $(OBJS) $(LIBUTILS)
	$(CC) $(CFLAGS) -o $(EXE) $(OBJS) $(LIBS)

$(OBJS) : $(HEADERS)

$(LIBUTILS): $(OBJS_LIBUTILS)
	$(AR) $(LIBUTILS) $(OBJS_LIBUTILS)

$(LIBUTILS) : $(HEADERS_LIBUTILS)

clean:
	$(RM) $(OBJS)
	$(RM) $(OBJS_LIBUTILS)
	$(RM) $(LIBUTILS)
	$(RM) $(EXE) 
	
again:                                                               
	make clean
	make    
	
wow:
	@echo "                                      W O W W W W WWWWWWWWWWWWWWWWWWW"

who:
	@echo "you are user $(USER) with uname `uname` (OS = $(OS)) and you working with compiler setting $(setting)" 


