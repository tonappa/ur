#include "ur_automata_scan_tsp/route_solver.hpp"

#include <algorithm>
#include <cmath>

#include "ortools/constraint_solver/routing.h"
#include "ortools/constraint_solver/routing_parameters.h"

using operations_research::Assignment;
using operations_research::DefaultRoutingSearchParameters;
using operations_research::FirstSolutionStrategy;
using operations_research::LocalSearchMetaheuristic;
using operations_research::RoutingIndexManager;
using operations_research::RoutingModel;
using operations_research::RoutingSearchParameters;
using operations_research::RoutingSearchStatus;

RouteResult solve_route(const std::vector<int64_t> & cost, int n, int start, int end,
                        const std::vector<std::vector<int>> & clusters,
                        double time_limit_s,
                        const std::vector<int> & initial)
{
  RouteResult result;
  if (n < 2 || static_cast<int>(cost.size()) != n * n) return result;

  RoutingIndexManager manager(n, 1, {RoutingIndexManager::NodeIndex(start)},
                              {RoutingIndexManager::NodeIndex(end)});
  RoutingModel routing(manager);

  const int transit = routing.RegisterTransitCallback([&](int64_t from, int64_t to) -> int64_t {
    const int a = manager.IndexToNode(from).value();
    const int b = manager.IndexToNode(to).value();
    return cost[static_cast<size_t>(a) * n + b];
  });
  routing.SetArcCostEvaluatorOfAllVehicles(transit);

  // Generalized TSP: one node per cluster. The penalty for skipping a whole
  // cluster is larger than any route, so a cluster is skipped only if it is
  // impossible to visit. Nodes outside the clusters must not be visited at
  // all: they get their own disjunction with zero penalty.
  if (!clusters.empty()) {
    int64_t max_cost = 0;
    for (int64_t c : cost) max_cost = std::max(max_cost, c);
    const int64_t skip_penalty = max_cost * static_cast<int64_t>(n) + 1;
    std::vector<bool> in_cluster(n, false);
    for (const std::vector<int> & cluster : clusters) {
      std::vector<int64_t> indices;
      for (int node : cluster) {
        indices.push_back(manager.NodeToIndex(RoutingIndexManager::NodeIndex(node)));
        in_cluster[node] = true;
      }
      routing.AddDisjunction(indices, skip_penalty, 1);
    }
    for (int node = 0; node < n; ++node) {
      if (in_cluster[node] || node == start || node == end) continue;
      routing.AddDisjunction({manager.NodeToIndex(RoutingIndexManager::NodeIndex(node))}, 0);
    }
  }

  RoutingSearchParameters params = DefaultRoutingSearchParameters();
  params.set_first_solution_strategy(FirstSolutionStrategy::PATH_CHEAPEST_ARC);
  params.set_local_search_metaheuristic(LocalSearchMetaheuristic::GUIDED_LOCAL_SEARCH);
  const int64_t ms = static_cast<int64_t>(std::llround(time_limit_s * 1000.0));
  params.mutable_time_limit()->set_seconds(ms / 1000);
  params.mutable_time_limit()->set_nanos(static_cast<int32_t>((ms % 1000) * 1000000));

  const Assignment * solution = nullptr;
  if (initial.empty()) {
    solution = routing.SolveWithParameters(params);
  } else {
    routing.CloseModelWithParameters(params);
    std::vector<std::vector<int64_t>> routes(1);
    for (int node : initial) routes[0].push_back(manager.NodeToIndex(RoutingIndexManager::NodeIndex(node)));
    const Assignment * start_solution = routing.ReadAssignmentFromRoutes(routes, true);
    if (start_solution != nullptr) solution = routing.SolveFromAssignmentWithParameters(start_solution, params);
  }
  result.status = RoutingSearchStatus::Value_Name(routing.status());
  if (solution == nullptr) return result;

  for (int64_t index = solution->Value(routing.NextVar(routing.Start(0)));
       !routing.IsEnd(index);
       index = solution->Value(routing.NextVar(index))) {
    result.nodes.push_back(manager.IndexToNode(index).value());
  }
  result.objective = solution->ObjectiveValue();
  result.ok = true;
  return result;
}
