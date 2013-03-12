CC	= g++
CFLAGS  = -Wall

OBJ	= slam.o pid.o srf.o middle.o
NAME	= middle

prog: $(OBJ)
	$(CC) $(CFLAGS) -o $(NAME) $(OBJ) -lm -lfftw3f `pkg-config opencv --cflags --libs`

%.o: %.cpp
	$(CC) $(CFLAGS) -c $<

clean:
	rm -rf $(OBJ) $(NAME)
