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
#include <limits>
#include <list>
#include <numeric>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;
using std::list;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1.
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method
   *   (and others in this file).
   */
  std::default_random_engine gen;
  // This line creates a normal (Gaussian) distribution for x, y, theta
  std::normal_distribution<double> dist_x(x, std[0]);
  std::normal_distribution<double> dist_y(y, std[1]);
  std::normal_distribution<double> dist_theta(theta, std[2]);

  // TODO: Set the number of particles
  num_particles_ = 100;
  const double double_pi = 2.0 * M_PI;
  for (int i = 0; i < num_particles_; ++i) {
    Particle particle;
    particle.id = i;
    particle.x = dist_x(gen);
    particle.y = dist_y(gen);
    particle.theta = fmod(dist_theta(gen), double_pi);
    particle.weight = 1.0;
    particles_.push_back(particle);
  }

  is_initialized_ = true;
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
  // This line creates a normal (Gaussian) distribution for x, y, theta
  std::normal_distribution<double> dist_x(0, std_pos[0]);
  std::normal_distribution<double> dist_y(0, std_pos[1]);
  std::normal_distribution<double> dist_theta(0, std_pos[2]);
  const double val_yaw = velocity / yaw_rate;
  const double delta_theta = yaw_rate * delta_t;
  const double double_pi = 2.0 * M_PI;
  for (int i = 0; i < num_particles_; ++i) {
    const double& theta = particles_[i].theta;
    const double new_theta =
        fmod(theta + delta_theta + dist_theta(gen), double_pi);
    particles_[i].x += val_yaw * (sin(new_theta) - sin(theta)) + dist_x(gen);
    particles_[i].y += val_yaw * (cos(theta) - cos(new_theta)) + dist_y(gen);
    particles_[i].theta = new_theta;
    // particles_[i].theta = fmod(new_theta + dist_theta(gen), double_pi);
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
  list<int> pool;
  for (size_t i = 0; i < predicted.size(); ++i) {
    pool.push_back(i);
  }
  for (size_t i = 0; i < observations.size(); ++i) {
    LandmarkObs& obs = observations[i];
    int nearest_idx = -1;
    auto nearest_iter = pool.end();
    double min_dist_2 = std::numeric_limits<const double>::infinity();
    for (auto iter = pool.begin(); iter != pool.end(); ++iter) {
      LandmarkObs& landmark = predicted[*iter];
      double dist_2 = pow(obs.x - landmark.x, 2) + pow(obs.y - landmark.y, 2);
      if (dist_2 < min_dist_2) {
        nearest_idx = *iter;
        nearest_iter = iter;
        min_dist_2 = dist_2;
      }
    }
    if (nearest_idx >= 0) {
      obs.id = nearest_idx;
      pool.erase(nearest_iter);
    }
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   const vector<LandmarkObs>& observations,
                                   const Map& map_landmarks) {
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian
   *   distribution. You can read more about this distribution here:
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system.
   *   Your particles are located according to the MAP'S coordinate system.
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no
   * scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */
  // Compute some constant values to save computation time.
  const double const_val_1 = 1 / (2 * M_PI * std_landmark[0] * std_landmark[1]);
  const double const_val_2 = 1 / (2 * pow(std_landmark[0], 2));
  const double const_val_3 = 1 / (2 * pow(std_landmark[1], 2));

  double sum_of_weights = 0.0;
  for (int i = 0; i < num_particles_; ++i) {
    Particle& particle = particles_[i];
    vector<LandmarkObs> t_observations(observations.size());
    // Observation transform
    const double cos_theta = cos(particle.theta);
    const double sin_theta = sin(particle.theta);
    for (size_t j = 0; j < observations.size(); ++j) {
      const LandmarkObs& landmark = observations[j];
      LandmarkObs t_landmark;
      t_landmark.id = -1;
      t_landmark.x =
          particle.x + (cos_theta * landmark.x) - (sin_theta * landmark.y);
      t_landmark.y =
          particle.y + (sin_theta * landmark.x) + (cos_theta * landmark.y);
      t_observations.push_back(t_landmark);
    }
    // Particle predicts its observable landmarks accroading to its location and
    // sensor range.
    vector<LandmarkObs> predicted;
    for (size_t i = 0; i < map_landmarks.landmark_list.size(); ++i) {
      LandmarkObs landmark;
      landmark.id = map_landmarks.landmark_list[i].id_i;
      landmark.x = map_landmarks.landmark_list[i].x_f;
      landmark.y = map_landmarks.landmark_list[i].y_f;
      // if (dist(particle.x, particle.y, landmark.x, landmark.y) <=
      //     sensor_range) {
      //   predicted.push_back(landmark);
      // }
      predicted.push_back(landmark);
    }
    // Observation association
    dataAssociation(predicted, t_observations);
    // Update weights
    double new_weight = 1.0;
    for (size_t i = 0; i < t_observations.size(); ++i) {
      LandmarkObs& predicted_obs = t_observations[i];
      if (predicted_obs.id >= 0) {
        LandmarkObs& associated_obs = predicted[predicted_obs.id];
        const double x_term =
            pow(predicted_obs.x - associated_obs.x, 2) / const_val_2;
        const double y_term =
            pow(predicted_obs.y - associated_obs.y, 2) / const_val_3;
        new_weight *= const_val_1 * exp(-(x_term + y_term));
      } else {
        std::cout << "Error, no landmark matches predicted observation"
                  << std::endl;
      }
    }
    particle.weight = new_weight;
    sum_of_weights += new_weight;
  }
  // Normalize particle's weight
  for (auto particle : particles_) {
    particle.weight /= sum_of_weights;
  }
}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional
   *   to their weight.
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
  vector<float> cumulative_weight(particles_.size(), 0.0);
  for (size_t i = 0; i < particles_.size(); ++i) {
    cumulative_weight[i] += particles_[i].weight;
  }

  // Systematic resample
  std::default_random_engine gen;
  std::uniform_real_distribution<double> dist(0.0, 1.0);
  const double weight_offset = 1.0 / num_particles_;
  double curr_weight = dist(gen) / num_particles_;
  vector<int> resample_idx(num_particles_);
  int idx = 0;
  while (resample_idx.size() < num_particles_) {
    if (curr_weight < cumulative_weight[idx]) {
      resample_idx.push_back(idx);
      curr_weight += weight_offset;
    } else {
      idx++;
    }
  }
  std::vector<Particle> new_particles(num_particles_);
  const double normalized_weight = 1 / num_particles_;
  for (int idx : resample_idx) {
    particles_[idx].weight = normalized_weight;
    new_particles.push_back(particles_[idx]);
  }
  particles_ = new_particles;
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
  particle.associations = associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length() - 1);  // get rid of the trailing space
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
  s = s.substr(0, s.length() - 1);  // get rid of the trailing space
  return s;
}