#pragma once

#include <cstdint>
#include <string>
#include <vector>

// Visit order found by OR-Tools (routing solver, one vehicle).
struct RouteResult {
  bool ok = false;
  std::vector<int> nodes;     // visited nodes, in order, start and end excluded
  int64_t objective = 0;      // total cost of the route, same unit as the matrix
  std::string status;         // OR-Tools search status, e.g. ROUTING_SUCCESS
};

// Open route from node `start` to node `end` through the other nodes of an
// n x n cost matrix (row-major, cost[a * n + b] = cost from a to b, integers).
//   clusters empty -> every node is visited once (TSP);
//   clusters given -> exactly one node of each cluster is visited (generalized
//                     TSP); nodes outside any cluster are not visited.
// Search: PATH_CHEAPEST_ARC first solution, then guided local search until
// time_limit_s. The result is a heuristic solution: OR-Tools does not prove it
// optimal (status ROUTING_OPTIMAL would say so).
// `initial` (optional): a route to start from (nodes in order, start and end
// excluded); the local search then never returns a worse route.
RouteResult solve_route(const std::vector<int64_t> & cost, int n, int start, int end,
                        const std::vector<std::vector<int>> & clusters,
                        double time_limit_s,
                        const std::vector<int> & initial = {});
