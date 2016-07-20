#include "nParticleFilter.hpp"

namespace nP{
	const int ParticleFilter::particleCount = 300;

	const double ParticleFilter::xMin = 0.0;
	const double ParticleFilter::xMax = 400.0;
	const double ParticleFilter::yMin = 0.0;
	const double ParticleFilter::yMax = 400.0;
	const double ParticleFilter::dMin = 0.0;
	const double ParticleFilter::dMax = 2.0 * M_PI;

	ParticleFilter::ParticleFilter() : particles(particleCount){
		std::random_device rd;
		rnd = std::mt19937(rd()); // Seed

		randomizeParticles();
	}

	void ParticleFilter::randomizeParticles(){
		std::uniform_real_distribution<> xDist(xMin, xMax);
		std::uniform_real_distribution<> yDist(yMin, yMax);
		std::uniform_real_distribution<> dDist(dMin, dMax);
		
		for(int i = 0; i < particleCount; i++){
			particles[i].x = xDist(rnd);
			particles[i].y = yDist(rnd);
			particles[i].d = dDist(rnd);
			particles[i].w = 0.0;
		}
	}

	std::vector<Particle>& ParticleFilter::getParticles(){
		return particles;
	}

	void ParticleFilter::filter(double camX, double camY, double camD, double* resX, double* resY, double* resD){
		resample();
		predict();
		weight(camX, camY, camD);
		measure(resX, resY, resD);
	}
	
	void ParticleFilter::measure(double* x, double* y, double* d){
		// Return weighted-Avg

		double dx = 0.0;
		double dy = 0.0;
		double sumX = 0.0;
		double sumY = 0.0;

		for(int i = 0; i < particleCount; i++){
			sumX += particles[i].x * particles[i].w;
			sumY += particles[i].y * particles[i].w;
			dx += cos(particles[i].d) * particles[i].w;
			dy += sin(particles[i].d) * particles[i].w;
		}

		*x = sumX;
		*y = sumY;
		*d = atan2(dy, dx);
		lastX = *x;
		lastY = *y;
		lastD = *d;
	}

	void ParticleFilter::weight(double x, double y, double d){
		// Calculate likelihood
		double sumWeight = 0.0;
		for(int i = 0; i < particleCount; i++){
			double dx = (x - particles[i].x) / 20.0;
			double dy = (y - particles[i].y) / 20.0;
			double dd = dirDiff(d, particles[i].d);

			particles[i].w = exp(-(dx*dx + dy*dy + dd*dd)) + 0.00001;
			sumWeight += particles[i].w;
		}

		// Normalize weights
		for(int i = 0; i < particleCount; i++){
			particles[i].w /= sumWeight;
		}
	}

	void ParticleFilter::predict(){
		// Noise, Linear motion ... etc

		std::normal_distribution<> xyDist(0.0, 10.0);
		std::normal_distribution<> dDist(0.0, 10.0*M_PI/180.0);

		for(int i = 0; i < particleCount; i++){
			particles[i].x += xyDist(rnd);
			particles[i].y += xyDist(rnd);
			particles[i].d += dDist(rnd);

			particles[i].x += cos(lastD);
			particles[i].y += sin(lastD);

			if(particles[i].x < xMin)particles[i].x = xMin;
			if(xMax < particles[i].x)particles[i].x = xMax;
			if(particles[i].y < yMin)particles[i].y = yMin;
			if(yMax < particles[i].y)particles[i].y = yMax;
			if(particles[i].d < 0.0)particles[i].d += 2.0*M_PI;
			if(2.0*M_PI < particles[i].d)particles[i].d -= 2.0*M_PI;
		}
	}

	void ParticleFilter::resample(){
		double accumW[particleCount];
		accumW[0] = {particles[0].w};
		for(int i = 1; i < particleCount; i++){
			accumW[i] = accumW[i-1] + particles[i].w;
		}

		// Copy
		std::vector<Particle> tmp;
		for(int i = 0; i < particleCount; i++){
			tmp.push_back(particles[i]);
		}

		// Selection
		std::uniform_real_distribution<> dist(0.0, 1.0);
		for(int i = 0; i < particleCount; i++){
			double d = dist(rnd);
			for(int j = 0; j < particleCount; j++){
				if(d > accumW[j])continue;

				particles[i].x = tmp[j].x;
				particles[i].y = tmp[j].y;
				particles[i].d = tmp[j].d;
				particles[i].w = 0.0;
				break;
			}
		}
	}

	double dirDiff(double dir1, double dir2){
		double tmp = dir2 - dir1;

		while(tmp < -M_PI){
			tmp += 2.0 * M_PI;
		}
		while(M_PI < tmp){
			tmp -= 2.0 * M_PI;
		}
		return tmp;
	}
};
