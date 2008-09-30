
FLAGS=-fPIC -ggdb -Wall 
LIBS=#-lreadline

all: power

power: power.o power_comm.h	
	g++ $(FLAGS) $(LINKINCLUDES) $(LIBS) -o $@ $^ 
