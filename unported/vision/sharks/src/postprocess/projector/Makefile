SRC = borg.cpp
OBJ = $(SRC:.cpp=.o)
OUT = borg

CC=g++
LIBS=-lm
CFLAGS= -g -O2 -Wall
default: depend $(OUT)

$(OUT): $(OBJ)
	${CC} $(OBJ) -o $(OUT) $(LIBS)
.c.o:
	${CC} ${CFLAGS} ${INCLUDES} -c $< -o $@

depend: 
	gcc $(INCLUDES) -MM $(SRC) >depend
	
clean:
	-rm depend $(OBJ) $(OUT)

-include depend

