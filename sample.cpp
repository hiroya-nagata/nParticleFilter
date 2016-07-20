#include <iostream>
#include <cmath>
#include <opencv2/opencv.hpp>

#include "nParticleFilter.hpp"

void draw(cv::Mat img, std::vector<nP::Particle>& particles, double tgtX, double tgtY, double tgtD, double estX, double estY, double estD){
	img = cv::Scalar::all(255);

	// Course
	cv::circle(img, cv::Point2d(200, 200), 150.0, cv::Scalar(255, 0, 0));

	// Measured
	cv::circle(img, cv::Point2d(tgtX, tgtY), 10.0, cv::Scalar(0, 255, 0), -1);
	cv::line(img, cv::Point2d(tgtX, tgtY),
		cv::Point2d(tgtX, tgtY) + 50.0 * cv::Point2d(cos(tgtD), sin(tgtD)), cv::Scalar(0, 255, 0));

	// Estimated
	cv::circle(img, cv::Point2d(estX, estY), 10.0, cv::Scalar(0, 0, 255), -1);
	cv::line(img, cv::Point2d(estX, estY),
		cv::Point2d(estX, estY) + 50.0 * cv::Point2d(cos(estD), sin(estD)), cv::Scalar(0, 0, 255));

	// Particles
	for(nP::Particle p : particles){
		cv::circle(img, cv::Point2d(p.x, p.y), 0.5, cv::Scalar(0, 0, 255), -1);
	}
}

int main(){
	std::random_device rd;
	std::mt19937 rnd = std::mt19937(rd()); // Seed
	std::uniform_int_distribution<> distP(0, 100);
	std::normal_distribution<> dist(0.0, 50.0);

	nP::ParticleFilter pf;
	std::vector<nP::Particle>& p = pf.getParticles();

	int frame = 0;
	double tgtX = 0.0;
	double tgtY = 0.0;
	double tgtD = -M_PI/2.0;
	double estX, estY, estD;

	cv::Mat img = cv::Mat(400, 400, CV_8UC3);

	std::cout << "Press [ESC] to exit..." << std::endl;
	while(true){
		frame++;

		tgtX = 150.0 * cos(frame / 50.0) + 200.0;
		tgtY = 150.0 * sin(frame / 50.0) + 200.0;
		tgtD = frame / 50.0 + M_PI/2;
		if(distP(rnd) < 20){ // 5% chance
			tgtX += dist(rnd);
			tgtY += dist(rnd);
			tgtD += dist(rnd);
		}

		pf.filter(tgtX, tgtY, tgtD, &estX, &estY, &estD);

		draw(img, p, tgtX, tgtY, tgtD, estX, estY, estD);

		cv::imshow("img", img);
		char k = cv::waitKey(10);
		if(k == 27)break; // Esc key
	}
}
