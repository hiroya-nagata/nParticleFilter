all:
	g++ sample.cpp nParticleFilter.cpp -std=c++11 -O2 `pkg-config opencv --cflags --libs` -o sample

clean:
	rm sample
