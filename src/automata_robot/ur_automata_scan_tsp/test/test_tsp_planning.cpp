#include <gtest/gtest.h>

#include <cmath>
#include <set>

#include "ur_automata_scan_tsp/route_solver.hpp"
#include "ur_automata_scan_tsp/tsp_planning.hpp"

// Trapezoid: 2 rad at v 1, a 1 -> 1 s up, 1 s cruise (1 rad), 1 s down = 3 s.
// Triangle: 0.25 rad at a 1 never reaches v 1 -> 2 * sqrt(0.25) = 1 s.
// PTP waits for the slowest joint.
TEST(PtpTime, TrapezoidTriangleAndSlowestJoint)
{
  EXPECT_NEAR(ptp_time({0.0}, {2.0}, {1.0}, {1.0}), 3.0, 1e-9);
  EXPECT_NEAR(ptp_time({0.0}, {-0.25}, {1.0}, {1.0}), 1.0, 1e-9);
  EXPECT_NEAR(ptp_time({0.0, 0.0}, {2.0, 0.25}, {1.0, 1.0}, {1.0, 1.0}), 3.0, 1e-9);
  EXPECT_NEAR(ptp_time({1.0}, {1.0}, {1.0}, {1.0}), 0.0, 1e-12);
}

// Points on a line visited in scrambled index order: the TSP from the left end
// to the right end must sweep them left to right.
TEST(RouteSolver, TspOnALine)
{
  const std::vector<double> x = {0.0, 3.0, 1.0, 4.0, 2.0, 5.0};   // node 0 start, node 5 end
  const int n = static_cast<int>(x.size());
  std::vector<int64_t> cost(n * n);
  for (int a = 0; a < n; ++a) {
    for (int b = 0; b < n; ++b) cost[a * n + b] = static_cast<int64_t>(std::llround(1000 * std::abs(x[a] - x[b])));
  }
  RouteResult r = solve_route(cost, n, 0, 5, {}, 1.0);
  ASSERT_TRUE(r.ok) << r.status;
  EXPECT_EQ(r.nodes, (std::vector<int>{2, 4, 1, 3}));
  EXPECT_EQ(r.objective, 5000);
}

// Generalized TSP: two clusters, one node each must be picked; the cheap chain
// is 0 -> 2 -> 3 -> 5.
TEST(RouteSolver, GtspPicksOneNodePerCluster)
{
  const int n = 6;
  std::vector<int64_t> cost(n * n, 100);
  cost[0 * n + 2] = 1;
  cost[2 * n + 3] = 1;
  cost[3 * n + 5] = 1;
  RouteResult r = solve_route(cost, n, 0, 5, {{1, 2}, {3, 4}}, 1.0);
  ASSERT_TRUE(r.ok) << r.status;
  EXPECT_EQ(r.nodes, (std::vector<int>{2, 3}));
  EXPECT_EQ(r.objective, 3);
}

// Starting from a given (poor, 11 m) route: the result is never worse than it.
TEST(RouteSolver, InitialRouteIsNotWorsened)
{
  const std::vector<double> x = {0.0, 3.0, 1.0, 4.0, 2.0, 5.0};
  const int n = static_cast<int>(x.size());
  std::vector<int64_t> cost(n * n);
  for (int a = 0; a < n; ++a) {
    for (int b = 0; b < n; ++b) cost[a * n + b] = static_cast<int64_t>(std::llround(1000 * std::abs(x[a] - x[b])));
  }
  RouteResult r = solve_route(cost, n, 0, 5, {}, 1.0, {3, 4, 1, 2});
  ASSERT_TRUE(r.ok) << r.status;
  EXPECT_EQ(r.nodes.size(), 4u);
  EXPECT_LE(r.objective, 11000);
}

// Lazy DP: the cheap candidate (0.1) is reachable only through a blocked
// segment; once it is checked the DP moves to the free one (0.5).
TEST(LazyDp, AvoidsSegmentsFoundBlocked)
{
  std::vector<std::vector<SeqCandidate>> layers = {{{{0.1}}, {{0.5}}}};
  std::set<int> checked;
  SeqCost cost;
  cost.blocked_penalty = 100.0;
  EdgeFilter check = [&](int, int, int, int c) {
    checked.insert(c);
    return c != 0;
  };
  EdgeFilter known_free = [&](int, int, int, int c) { return !(checked.count(c) && c == 0); };
  int iterations = 0;
  SeqResult r = choose_sequence_lazy({0.0}, layers, cost, known_free, check, 10, &iterations);
  EXPECT_EQ(r.chosen[0], 1);
  EXPECT_EQ(checked.size(), 2u);   // the blocked pair made it check every pair of the layer
  EXPECT_GE(iterations, 2);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
