/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1.
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method
   *   (and others in this file).
   */
  num_particles = 30;  // TODO: Set the number of particles
  std::default_random_engine gen;
  double std_x = std[0];
  double std_y = std[1];
  double std_theta = std[2];

  std::normal_distribution<double> dist_x(x, std_x);
  std::normal_distribution<double> dist_y(y, std_y);
  std::normal_distribution<double> dist_theta(theta, std_theta);

  Particle particle;

  for (int i = 0; i < num_particles; i++) {
    particle.id = i;
    particle.x = dist_x(gen);
    particle.y = dist_y(gen);
    particle.theta = dist_theta(gen);
    particle.weight = 1;
    particles.push_back(particle);
    weights.push_back(particle.weight);
  }
  is_initialized = true;
  std::cout << "Initial particles with weights: \n";
  for (int i = 0; i < particles.size(); i++) {
    std::cout << "Particle " << i << " " << particles[i].x << " " << particles[i].y << " " << particles[i].theta << " weight = " << particles[i].weight << std::endl;
  }
}

void ParticleFilter::prediction(double delta_t, double std_pos[],
                                double velocity, double yaw_rate) {
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */
   std::default_random_engine gen;
   double std_x = std_pos[0];
   double std_y = std_pos[1];
   double std_theta = std_pos[2];
   for (int i = 0; i < particles.size(); i++) {
     if (fabs(yaw_rate) > 0.001) {
       particles[i].x = particles[i].x + (velocity / yaw_rate * (sin(particles[i].theta + (yaw_rate * delta_t)) - sin(particles[i].theta)));
       particles[i].y = particles[i].y + (velocity / yaw_rate * (-cos(particles[i].theta + (yaw_rate * delta_t)) + cos(particles[i].theta)));
       particles[i].theta = particles[i].theta + (yaw_rate * delta_t);
     } else {
       particles[i].x = particles[i].x + (velocity * cos(particles[i].theta) * delta_t);
       particles[i].y = particles[i].y + (velocity * sin(particles[i].theta) * delta_t);
       particles[i].theta = particles[i].theta;
     }

     std::normal_distribution<double> dist_x(particles[i].x, std_x);
     std::normal_distribution<double> dist_y(particles[i].y, std_y);
     std::normal_distribution<double> dist_theta(particles[i].theta, std_theta);

     particles[i].x = dist_x(gen);
     particles[i].y = dist_y(gen);
     particles[i].theta = dist_theta(gen);
   }
   std::cout << "Predicted particles with weights: \n";
   for (int i = 0; i < particles.size(); i++) {
     std::cout << "Particle " << i << " " << particles[i].x << " " << particles[i].y << " " << particles[i].theta << " weight = " << particles[i].weight << std::endl;
   }
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted,
                                     vector<LandmarkObs>& observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each
   *   observed measurement and assign the observed measurement to this
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will
   *   probably find it useful to implement this method and use it as a helper
   *   during the updateWeights phase.
   */
   for (int i = 0; i < observations.size(); i++) {
     double minDist = std::numeric_limits<double>::max();
     int id = -1;
     double obs_x = observations[i].x;
     double obs_y = observations[i].y;
     for (int j = 0; j < predicted.size(); j++) {
       double pred_x = predicted[j].x;
       double pred_y = predicted[j].y;
       int pred_id = predicted[j].id;

       double dist = std::sqrt(((pred_x - obs_x) * (pred_x - obs_x)) + ((pred_y - obs_y) * (pred_y - obs_y)));
       if (dist < minDist) {
         minDist = dist;
         id = pred_id;
       }
     }
     observations[i].id = id;
   }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   const vector<LandmarkObs> &observations,
                                   const Map &map_landmarks) {
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian
   *   distribution. You can read more about this distribution here:
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system.
   *   Your particles are located according to the MAP'S coordinate system.
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */
   double std_x = std_landmark[0];
   double std_y = std_landmark[1];
   double obs_x, obs_y = 0;
   double normalize = 0;
   double particle_x, particle_y, particle_theta;
   LandmarkObs obs_map;
   LandmarkObs in_Range;

   for (int i = 0; i < particles.size(); i++) {
     particle_x = particles[i].x;
     particle_y = particles[i].y;
     particle_theta = particles[i].theta;
     std::vector<LandmarkObs> transformed_observations;
     std::vector<LandmarkObs> inRangeLandmarks;

     for (int j = 0; j < map_landmarks.landmark_list.size(); j++) {
       in_Range.x = (double) map_landmarks.landmark_list[j].x_f;
       in_Range.y = (double) map_landmarks.landmark_list[j].y_f;
       in_Range.id = map_landmarks.landmark_list[j].id_i;
       if ((fabs((particle_x - in_Range.x)) <= sensor_range) && (fabs((particle_y - in_Range.y)) <= sensor_range)) {
         inRangeLandmarks.push_back(in_Range);
       }
     }

     for (int j = 0; j < observations.size(); j++) {
       obs_x = observations[j].x;
       obs_y = observations[j].y;

       obs_map.id = j;
       obs_map.x = (cos(particles[i].theta) * obs_x) - (sin(particles[i].theta) * obs_y) + particles[i].x;
       obs_map.y = (sin(particles[i].theta) * obs_x) + (cos(particles[i].theta) * obs_y) + particles[i].y;
       transformed_observations.push_back(obs_map);
     }
     dataAssociation(inRangeLandmarks, transformed_observations);

     for (int j = 0; j < transformed_observations.size(); j++) {
       double obs_x = transformed_observations[j].x;
       double obs_y = transformed_observations[j].y;
       int obs_id = transformed_observations[j].id;
       double weight = 1;
       for (int k = 0; k < inRangeLandmarks.size(); k++) {
         double landmark_x = inRangeLandmarks[k].x;
         double landmark_y = inRangeLandmarks[k].y;
         int landmark_id = inRangeLandmarks[k].id;
         if (obs_id == landmark_id) {
           double x_diff = obs_x - landmark_x;
           double y_diff = obs_y - landmark_y;
           double exponent = (pow(x_diff, 2) / (2 * pow(std_x, 2))) + (pow(y_diff, 2) / (2 * pow(std_y, 2)));
           weight = (1 / (2 * M_PI * std_x * std_y)) * exp(-1 * exponent);
           particles[i].weight = particles[i].weight * weight;
         }
       }
     }
     normalize = normalize + particles[i].weight;
   }
   for (int i = 0; i < particles.size(); i++) {
     particles[i].weight = particles[i].weight / normalize;
     weights[i] = particles[i].weight;
   }
   std::cout << "updated particles with weights: \n";
   for (int i = 0; i < particles.size(); i++) {
     std::cout << "Particle " << i << " " << particles[i].x << " " << particles[i].y << " " << particles[i].theta << " weight = " << particles[i].weight << std::endl;
   }
}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional
   *   to their weight.
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
   std::vector<Particle> sampled_particles;
   double beta = 0;
   std::default_random_engine gen;
   std::uniform_int_distribution<int> dist(0, num_particles - 1);
   int index = dist(gen);
   double max_weight = 2 * *std::max_element(weights.begin(), weights.end());

   for (int i = 0; i < particles.size(); i++) {
     std::uniform_real_distribution<double> dist_weight(0, max_weight);
     beta = beta + dist_weight(gen);
     while (weights[index] < beta) {
       beta = beta - weights[index];
       index = (index + 1) % num_particles;
     }
     sampled_particles.push_back(particles[index]);
   }
   particles = sampled_particles;

}

void ParticleFilter::SetAssociations(Particle& particle,
                                     const vector<int>& associations,
                                     const vector<double>& sense_x,
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association,
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}
