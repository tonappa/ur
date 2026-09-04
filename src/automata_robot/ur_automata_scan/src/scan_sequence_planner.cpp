#include "ur_automata_scan/scan_sequence_planner.hpp"

#include <cmath>
#include <limits>

double joint_l2(const std::vector<double> & a, const std::vector<double> & b)
{
  double sum = 0.0;
  for (size_t k = 0; k < a.size() && k < b.size(); ++k) {
    double d = a[k] - b[k];
    sum += d * d;
  }
  return std::sqrt(sum);
}

SeqResult choose_sequence(const std::vector<double> & start_joints,
                          const std::vector<std::vector<SeqCandidate>> & layers,
                          const EdgeFilter & edge_ok,
                          double blocked_penalty)
{
  SeqResult result;
  result.chosen.assign(layers.size(), -1);

  // Stato "precedente": all'inizio e' il solo punto di partenza, costo zero.
  std::vector<double> prev_cost = {0.0};
  std::vector<std::vector<double>> prev_joints = {start_joints};

  // back[i][c] = indice del candidato scelto nel layer non vuoto precedente
  // per arrivare al candidato c del layer i con costo minimo.
  std::vector<std::vector<int>> back(layers.size());
  int last_layer = -1;   // ultimo layer non vuoto elaborato (-1 = punto di partenza)

  for (size_t i = 0; i < layers.size(); ++i) {
    const auto & layer = layers[i];
    if (layer.empty()) continue;

    std::vector<double> cost(layer.size(), std::numeric_limits<double>::infinity());
    back[i].assign(layer.size(), -1);

    for (size_t c = 0; c < layer.size(); ++c) {
      for (size_t p = 0; p < prev_joints.size(); ++p) {
        double total = prev_cost[p] + joint_l2(prev_joints[p], layer[c].joints) + layer[c].extra_cost;
        if (edge_ok && !edge_ok(last_layer, static_cast<int>(p), static_cast<int>(i), static_cast<int>(c))) {
          total += blocked_penalty;
        }
        if (total < cost[c]) {
          cost[c] = total;
          back[i][c] = static_cast<int>(p);
        }
      }
    }

    prev_cost = cost;
    prev_joints.clear();
    for (const auto & cand : layer) prev_joints.push_back(cand.joints);
    last_layer = static_cast<int>(i);
  }

  if (last_layer < 0) return result;   // tutti i layer vuoti

  // Miglior candidato dell'ultimo layer, poi si risale la catena all'indietro.
  int best = 0;
  for (size_t c = 1; c < prev_cost.size(); ++c) {
    if (prev_cost[c] < prev_cost[best]) best = static_cast<int>(c);
  }
  result.total_cost = prev_cost[best];

  int c = best;
  for (int i = last_layer; i >= 0; --i) {
    if (layers[i].empty()) continue;
    result.chosen[i] = c;
    c = back[i][c];
  }
  return result;
}
