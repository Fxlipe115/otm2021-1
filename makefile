all:
	g++ -o tabu tabu.cpp -std=c++11 -O2 -ggdb3 -lemon

clean:
	rm tabu
