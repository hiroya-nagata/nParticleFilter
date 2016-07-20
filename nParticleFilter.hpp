#pragma once

#include <vector>
#include <random>
#include <cmath>

namespace nP{
	double dirDiff(double dir1, double dir2);
	
	class Particle{
	public:
		double w; // Weight
		double x, y, d; // X, Y, Direction(rad)
	};

	class ParticleFilter{
	private:
		static const int particleCount;
		static const double xMin, xMax;
		static const double yMin, yMax;
		static const double dMin, dMax;

		std::vector<Particle> particles;
		std::mt19937 rnd; // Randomizer
		void randomizeParticles();
	public:
		ParticleFilter();
		std::vector<Particle>& getParticles();

		void filter(double camX, double camY, double camD,
			double* resX, double* resY, double* resD);

		void measure(double* x, double* y, double* d);
		void weight(double x, double y, double d);
		void predict();
		void resample();
	};
};