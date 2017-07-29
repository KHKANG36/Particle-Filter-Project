/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// Set the number of particles. Initialize all particles to first position (based on estimates of 
	// x, y, theta and their uncertainties from GPS) and all weights to 1. Add random Gaussian noise to each particle.
	
	num_particles = 1000;

	default_random_engine gen; 

	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);

	for (int i=0; i<num_particles; ++i){

		Particle p;

		p.id = i;
		p.x = dist_x(gen);
		p.y = dist_y(gen);
		p.theta = dist_theta(gen);
		p.weight = 1.0;

		particles.push_back(p);

	}

	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// Add measurements to each particle and add random Gaussian noise.

	default_random_engine gen; 

	normal_distribution<double> dist_x(0, std_pos[0]);
	normal_distribution<double> dist_y(0, std_pos[1]);
	normal_distribution<double> dist_theta(0, std_pos[2]);

	for (int i = 0; i<num_particles; ++i) {

		//Predict when yaw_rate is not zero,
		if (fabs(yaw_rate) > 0.001){

			particles[i].x += velocity/yaw_rate * (sin(particles[i].theta+yaw_rate*delta_t) - sin(particles[i].theta)) + dist_x(gen);
			particles[i].y += velocity/yaw_rate * (cos(particles[i].theta) - cos(particles[i].theta+yaw_rate*delta_t)) + dist_y(gen);
			particles[i].theta += yaw_rate*delta_t + dist_theta(gen);
		}
		//Predict when yaw_rate is zero
		else{

			particles[i].x += velocity*delta_t*cos(particles[i].theta) + dist_x(gen);
			particles[i].y += velocity*delta_t*sin(particles[i].theta) + dist_y(gen);
			particles[i].theta += dist_theta(gen);
		}
	}
}


void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// Find the predicted measurement that is closest to each observed measurement and assign the 
	// observed measurement to this particular landmark.

	for (int i=0; i<observations.size(); ++i){

		LandmarkObs obsm = observations[i];
		//Initialize previous distance with large value
		double prev_distance = 10000000;
		int plm_id = 0;

		for (int j=0; j<predicted.size();++j){
			LandmarkObs predm = predicted[j];
			double distance = dist(obsm.x, obsm.y, predm.x, predm.y);
			if (distance < prev_distance){
				prev_distance = distance;
				plm_id = predm.id;
			}
		}
		observations[i].id = plm_id;
	}

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {
	// Update the weights of each particle using a mult-variate Gaussian distribution. 

	for (int i=0; i<num_particles; ++i){

		std::vector<LandmarkObs> predicted_p;

		//particle location in MAP coordinate system
		double p_x = particles[i].x;
		double p_y = particles[i].y;
		double p_theta = particles[i].theta; 

		for (int j=0; j < map_landmarks.landmark_list.size(); ++j){

			int lm_id = map_landmarks.landmark_list[j].id_i;
			float lm_x = map_landmarks.landmark_list[j].x_f;
			float lm_y = map_landmarks.landmark_list[j].y_f;

			//Push back only the landmarks within sensor range 
			if (fabs(lm_x-p_x) <= sensor_range && fabs(lm_y-p_y) <= sensor_range){
				predicted_p.push_back(LandmarkObs{lm_id, lm_x, lm_y});
			}
		}

		// Transform the observation from Local to Map coordinate
		std::vector<LandmarkObs> obs_transform;

		for (int k=0; k<observations.size();++k){
			double obs_tx = cos(p_theta)*observations[k].x - sin(p_theta)*observations[k].y + p_x;
			double obs_ty = sin(p_theta)*observations[k].x + cos(p_theta)*observations[k].y + p_y;

			obs_transform.push_back(LandmarkObs{observations[k].id, obs_tx, obs_ty});
		}

		//Data association 
		dataAssociation(predicted_p,obs_transform);

		//Update weight
		double t_weight = 1.0;

		for (int l=0;l<obs_transform.size();++l){
			double x_meas, y_meas, x_mu, y_mu; 
			for (int m=0; m<predicted_p.size();++m){
				if (predicted_p[m].id == obs_transform[l].id){
					x_mu = predicted_p[m].x;
					y_mu = predicted_p[m].y;
				}
			}
			x_meas = obs_transform[l].x;
			y_meas = obs_transform[l].y;

			//Update the weights with multivaritive distribution 
			double cov_x = std_landmark[0]*std_landmark[0];
			double cov_y = std_landmark[1]*std_landmark[1];
			double norm = 2.0*M_PI*std_landmark[0]*std_landmark[1];
			double d_x = (x_meas - x_mu);
			double d_y = (y_meas - y_mu); 

			double wt = exp(-(d_x*d_x/(2*cov_x)+d_y*d_y/(2*cov_y)))/norm;
			t_weight *= wt;
		}

		particles[i].weight = t_weight;
		weights.push_back(t_weight);
	}
}


void ParticleFilter::resample() {
	// Resample particles with replacement with probability proportional to their weight. 

	std::vector<Particle> new_particles; 
	default_random_engine gen;

	//Use discrete_distribution function
	std::discrete_distribution<int>w_dist(weights.begin(), weights.end());

	for (int i=0; i<num_particles; ++i){
		new_particles.push_back(particles[w_dist(gen)]);
	}
	//Update the particles and clear the weights
	particles.clear();
	particles = new_particles;
	weights.clear();

}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations= associations;
 	particle.sense_x = sense_x;
 	particle.sense_y = sense_y;

 	return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
