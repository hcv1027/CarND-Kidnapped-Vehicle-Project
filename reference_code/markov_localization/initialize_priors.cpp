#include <iostream>
#include <vector>
#include "helpers.h"

using std::vector;

// initialize priors assuming vehicle at landmark +/- 1.0 meters position stdev
vector<float> initialize_priors(int map_size, vector<float> landmark_positions,
                                float position_stdev);

float motion_model(float pseudo_position, float movement, vector<float> priors,
                   int map_size, int control_stdev);

int main() {
  // set standard deviation of control:
  float control_stdev = 1.0f;

  // set standard deviation of position:
  float position_stdev = 1.0f;

  // meters vehicle moves per time step
  float movement_per_timestep = 1.0f;

  // number of x positions on map
  int map_size = 25;

  // initialize landmarks
  vector<float> landmark_positions{5, 10, 20};

  // initialize priors
  vector<float> priors =
      initialize_priors(map_size, landmark_positions, position_stdev);

  // step through each pseudo position x (i)
  for (float i = 0; i < map_size; ++i) {
    float pseudo_position = i;

    // get the motion model probability for each x position
    float motion_prob = motion_model(pseudo_position, movement_per_timestep,
                                     priors, map_size, control_stdev);

    // print to stdout
    std::cout << pseudo_position << "\t" << motion_prob << std::endl;
  }

  return 0;
}

// TODO: implement the motion model: calculates prob of being at
// an estimated position at time t
float motion_model(float pseudo_position, float movement, vector<float> priors,
                   int map_size, int control_stdev) {
  // initialize probability
  float position_prob = 0.0f;

  // loop over state space for all possible positions x (convolution):
  for (int i = 0; i < map_size; ++i) {
    float delta_position = pseudo_position - i;
    // transition probabilities:
    float transition_prob =
        Helpers::normpdf(delta_position, movement, control_stdev);
    // estimate probability for the motion model, this is our prior
    position_prob += transition_prob * priors[i];
  }

  return position_prob;
}

// TODO: Complete the initialize_priors function
vector<float> initialize_priors(int map_size, vector<float> landmark_positions,
                                float position_stdev) {
  // initialize priors assuming vehicle at landmark +/- 1.0 meters position
  // stdev

  // set all priors to 0.0
  vector<float> priors(map_size, 0.0);

  int normalizer = 0;
  for (int i = 0; i < landmark_positions.size(); ++i) {
    for (int j = -position_stdev; j <= position_stdev; ++j) {
      int idx = landmark_positions[i] + j;
      if (idx >= 0 && idx < map_size) {
        normalizer++;
        priors[idx] += 1.0;
      }
    }
  }
  for (int i = 0; i < map_size; ++i) {
    priors[i] /= normalizer;
  }

  return priors;
}