#include <algorithm>
#include <iostream>
#include <vector>

#include "helpers.h"

using std::vector;
using std::cout;
using std::endl;

vector<float> initialize_priors(int map_size, vector<float> landmark_positions,
                                float position_stdev);

float motion_model(float pseudo_position, float movement, vector<float> priors,
                   int map_size, int control_stdev);

// function to get pseudo ranges
vector<float> pseudo_range_estimator(vector<float> landmark_positions,
                                     float pseudo_position);

// observation model: calculate likelihood prob term based on landmark proximity
float observation_model(vector<float> landmark_positions,
                        vector<float> observations, vector<float> pseudo_ranges,
                        float distance_max, float observation_stdev);

/*
 * extract sensor observations
 *   for each pseudo-position:
 *       1. get the motion model probability
 *       2. determine pseudo ranges
 *       3. get the observation model probability
 *       4. use the motion and observation model probabilities to calculate the
 *          posterior probability
 *   normalize posteriors (see helpers.h for a normalization function)
 *   update priors (priors --> posteriors)
 */
int main() {
  // set standard deviation of control
  float control_stdev = 1.0f;

  // set standard deviation of position
  float position_stdev = 1.0f;

  // meters vehicle moves per time step
  float movement_per_timestep = 1.0f;

  // set observation standard deviation
  float observation_stdev = 1.0f;

  // number of x positions on map
  int map_size = 25;

  // set distance max
  float distance_max = map_size;

  // define landmarks
  vector<float> landmark_positions{3, 9, 14, 23};

  // define observations vector, each inner vector represents a set
  // of observations for a time step
  vector<vector<float> > sensor_obs{{1, 7, 12, 21},
                                    {0, 6, 11, 20},
                                    {5, 10, 19},
                                    {4, 9, 18},
                                    {3, 8, 17},
                                    {2, 7, 16},
                                    {1, 6, 15},
                                    {0, 5, 14},
                                    {4, 13},
                                    {3, 12},
                                    {2, 11},
                                    {1, 10},
                                    {0, 9},
                                    {8},
                                    {7},
                                    {6},
                                    {5},
                                    {4},
                                    {3},
                                    {2},
                                    {1},
                                    {0},
                                    {},
                                    {},
                                    {}};

  /**
   * TODO: initialize priors
   */
  vector<float> priors =
      initialize_priors(map_size, landmark_positions, position_stdev);

  // UNCOMMENT TO SEE THIS STEP OF THE FILTER
  //   cout << "-----------PRIORS INIT--------------" << endl;
  //   for (int p = 0; p < priors.size(); ++p){
  //    cout << priors[p] << endl;
  //   }

  // initialize posteriors
  vector<float> posteriors(map_size, 0.0);

  // specify time steps
  int time_steps = sensor_obs.size();

  // declare observations vector
  vector<float> observations;

  // cycle through time steps
  for (int t = 0; t < time_steps; ++t) {
    // UNCOMMENT TO SEE THIS STEP OF THE FILTER
    // cout << "---------------TIME STEP---------------" << endl;
    // cout << "t = " << t << endl;
    // cout << "-----Motion----------OBS----------------PRODUCT--" << endl;

    // initialize posteriors
    // vector<float> posteriors(map_size, 0.0);

    if (!sensor_obs[t].empty()) {
      observations = sensor_obs[t];
    } else {
      observations = {float(distance_max)};
    }

    // step through each pseudo position x (i)
    for (unsigned int i = 0; i < map_size; ++i) {
      float pseudo_position = float(i);

      /**
       * TODO: get the motion model probability for each x position
       */
      float motion_prob = motion_model(pseudo_position, movement_per_timestep,
                                       priors, map_size, control_stdev);

      /**
       * TODO: get pseudo ranges
       */
      vector<float> pseudo_ranges =
          pseudo_range_estimator(landmark_positions, pseudo_position);

      /**
       * TODO: get observation probability
       */
      float observation_prob =
          observation_model(landmark_positions, observations, pseudo_ranges,
                            distance_max, observation_stdev);

      /**
       * TODO: calculate the ith posterior and pass to posteriors vector
       */
      posteriors[i] = observation_prob * motion_prob;

      // UNCOMMENT TO SEE THIS STEP OF THE FILTER
      // cout << motion_prob << "\t" << observation_prob << "\t"
      //     << "\t"  << motion_prob * observation_prob << endl;
    }

    // UNCOMMENT TO SEE THIS STEP OF THE FILTER
    // cout << "----------RAW---------------" << endl;
    // for (int p = 0; p < posteriors.size(); ++p) {
    //  cout << posteriors[p] << endl;
    //}

    /**
     * TODO: normalize posteriors (see helpers.h for a helper function)
     */
    posteriors = Helpers::normalize_vector(posteriors);

    // print to stdout
    // cout << posteriors[t] <<  "\t" << priors[t] << endl;

    // UNCOMMENT TO SEE THIS STEP OF THE FILTER
    // cout << "----------NORMALIZED---------------" << endl;

    /**
     * TODO: update priors
     */
    priors = posteriors;

    // UNCOMMENT TO SEE THIS STEP OF THE FILTER
    // for (int p = 0; p < posteriors.size(); ++p) {
    //  cout << posteriors[p] << endl;
    //}

    // print posteriors vectors to stdout
    for (int p = 0; p < posteriors.size(); ++p) {
      cout << posteriors[p] << endl;
    }
  }

  return 0;
}

// observation model: calculate likelihood prob term based on landmark proximity
float observation_model(vector<float> landmark_positions,
                        vector<float> observations, vector<float> pseudo_ranges,
                        float distance_max, float observation_stdev) {
  // initialize observation probability
  float distance_prob = 1.0f;

  // run over current observation vector
  for (int z = 0; z < observations.size(); ++z) {
    // define min distance
    float pseudo_range_min;

    // check, if distance vector exists
    if (pseudo_ranges.size() > 0) {
      // set min distance
      pseudo_range_min = pseudo_ranges[0];
      // remove this entry from pseudo_ranges-vector
      pseudo_ranges.erase(pseudo_ranges.begin());
    } else {  // no or negative distances: set min distance to a large number
      pseudo_range_min = std::numeric_limits<const float>::infinity();
    }

    // estimate the probability for observation model, this is our likelihood
    distance_prob *=
        Helpers::normpdf(observations[z], pseudo_range_min, observation_stdev);
  }

  return distance_prob;
}

vector<float> pseudo_range_estimator(vector<float> landmark_positions,
                                     float pseudo_position) {
  // define pseudo observation vector
  vector<float> pseudo_ranges;

  // loop over number of landmarks and estimate pseudo ranges
  for (int i = 0; i < landmark_positions.size(); ++i) {
    float range = landmark_positions[i] - pseudo_position;
    if (range > 0) {
      pseudo_ranges.push_back(range);
    }
  }

  // sort pseudo range vector
  std::sort(pseudo_ranges.begin(), pseudo_ranges.end());

  return pseudo_ranges;
}

// motion model: calculates prob of being at an estimated position at time t
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

// initialize priors assuming vehicle at landmark +/- 1.0 meters position stdev
vector<float> initialize_priors(int map_size, vector<float> landmark_positions,
                                float position_stdev) {
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