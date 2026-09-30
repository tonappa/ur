#include "ur_automata_scan_tsp/tsp_planning.hpp"

#include <algorithm>
#include <cmath>

double ptp_time(const std::vector<double> & a, const std::vector<double> & b,
                const std::vector<double> & max_velocity,
                const std::vector<double> & max_acceleration)
{
  double slowest = 0.0;
  for (size_t k = 0; k < a.size() && k < b.size(); ++k) {
    const double d = std::abs(b[k] - a[k]);
    const double v = max_velocity[k];
    const double acc = max_acceleration[k];
    double t = 0.0;
    if (d * acc >= v * v) {
      t = d / v + v / acc;          // accelerate, cruise at v, decelerate
    } else {
      t = 2.0 * std::sqrt(d / acc); // never reaches v
    }
    slowest = std::max(slowest, t);
  }
  return slowest;
}

SeqResult choose_sequence_lazy(const std::vector<double> & start_joints,
                               const std::vector<std::vector<SeqCandidate>> & layers,
                               const SeqCost & cost,
                               const EdgeFilter & known_free,
                               const EdgeFilter & check_segment,
                               int max_iterations,
                               int * iterations)
{
  SeqResult seq;
  std::vector<int> previous_chain;
  for (int iter = 0; iter < max_iterations; ++iter) {
    seq = choose_sequence(start_joints, layers, known_free, cost);
    if (iterations) *iterations = iter + 1;
    if (seq.chosen == previous_chain) break;   // nothing new learned since the last run
    previous_chain = seq.chosen;

    int new_blocked = 0;
    int prev_layer = -1;
    int prev_c = 0;
    for (size_t k = 0; k < layers.size(); ++k) {
      if (seq.chosen[k] < 0) continue;
      const int layer = static_cast<int>(k);
      const int c = seq.chosen[k];
      if (!check_segment(prev_layer, prev_c, layer, c)) {
        ++new_blocked;
        const size_t prev_count = (prev_layer < 0) ? 1 : layers[prev_layer].size();
        for (size_t p2 = 0; p2 < prev_count; ++p2) {
          for (size_t c2 = 0; c2 < layers[k].size(); ++c2) {
            check_segment(prev_layer, static_cast<int>(p2), layer, static_cast<int>(c2));
          }
        }
      }
      prev_layer = layer;
      prev_c = c;
    }
    if (new_blocked == 0) break;
  }
  return seq;
}
