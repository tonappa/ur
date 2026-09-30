#pragma once

#include <vector>

#include "ur_automata_scan/scan_sequence_planner.hpp"

// Estimated duration (s) of a Pilz PTP motion from a to b: every joint follows
// a trapezoidal velocity profile (or a triangular one when the move is too
// short to reach full speed) and PTP synchronizes the joints on the slowest
// one. max_velocity / max_acceleration are per joint, already scaled.
// Known limit: Pilz samples the trajectory every 0.1 s, the real duration can
// be a bit longer; the node compares this estimate with the planned times.
double ptp_time(const std::vector<double> & a, const std::vector<double> & b,
                const std::vector<double> & max_velocity,
                const std::vector<double> & max_acceleration);

// Layered DP of ur_automata_scan (choose_sequence) with lazy checks of the
// segments, same loop as scan_sequence_node: the DP picks a chain, the segments
// of the chain are checked with `check_segment` (true = straight joint line is
// free); when one is blocked every pair between those two layers is checked, so
// the next DP run knows all the free ways between them. It stops when the chain
// has no blocked segment or does not change any more.
// `known_free` answers from what was already checked (unknown = free), and
// `cost.blocked_penalty` is added to known blocked segments. The caller caches
// the checks. `iterations` (optional) receives the number of DP runs.
SeqResult choose_sequence_lazy(const std::vector<double> & start_joints,
                               const std::vector<std::vector<SeqCandidate>> & layers,
                               const SeqCost & cost,
                               const EdgeFilter & known_free,
                               const EdgeFilter & check_segment,
                               int max_iterations,
                               int * iterations = nullptr);
