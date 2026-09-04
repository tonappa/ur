#pragma once

#include <functional>
#include <vector>

// Un candidato = una configurazione dei giunti che raggiunge un waypoint,
// piu' un costo extra opzionale (es. penalita' sul pitch scelto).
struct SeqCandidate {
  std::vector<double> joints;
  double extra_cost = 0.0;
};

struct SeqResult {
  std::vector<int> chosen;   // per ogni layer: indice del candidato scelto, -1 se il layer e' vuoto
  double total_cost = 0.0;   // somma delle distanze joint-space lungo la catena + extra_cost
};

// Distanza euclidea nei giunti, SENZA wrap a +-pi: i giunti UR hanno limiti
// +-2pi e non possono "passare dall'altra parte", quindi andare da +179 a
// -179 gradi e' davvero un giro quasi completo, non 2 gradi.
double joint_l2(const std::vector<double> & a, const std::vector<double> & b);

// Callback opzionale sugli archi: ritorna false se il tratto dal candidato
// prev_c del layer prev_layer al candidato c del layer `layer` e' bloccato
// (es. la retta in joint space attraversa un ostacolo). prev_layer = -1 indica
// il punto di partenza. Un arco bloccato non e' vietato: costa blocked_penalty
// in piu', cosi' la DP lo evita quando puo' ma non scarta il waypoint.
using EdgeFilter = std::function<bool(int prev_layer, int prev_c, int layer, int c)>;

// Programmazione dinamica su grafo a strati: layer i = candidati del waypoint
// i. Sceglie un candidato per layer minimizzando la somma delle distanze
// joint-space tra layer consecutivi (piu' gli extra_cost), partendo da
// start_joints. I layer vuoti (waypoint irraggiungibili) vengono scavalcati.
SeqResult choose_sequence(const std::vector<double> & start_joints,
                          const std::vector<std::vector<SeqCandidate>> & layers,
                          const EdgeFilter & edge_ok = EdgeFilter(),
                          double blocked_penalty = 1000.0);
