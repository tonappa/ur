#include <gtest/gtest.h>

#include "ur_automata_scan/scan_sequence_planner.hpp"

// La DP deve preferire il percorso globalmente piu' corto, non quello
// localmente piu' vicino (greedy). Start (0,0); layer 0: A=(0.1,0) vicino,
// B=(0,1) lontano; layer 1: solo (0,1.1). Greedy sceglie A e poi paga 1.10;
// il totale via A e' 1.20, via B e' 1.10 -> la DP deve scegliere B.
TEST(SequencePlanner, PrefersGlobalPathOverGreedy)
{
  std::vector<std::vector<SeqCandidate>> layers = {
    { {{0.1, 0.0}}, {{0.0, 1.0}} },
    { {{0.0, 1.1}} },
  };
  SeqResult r = choose_sequence({0.0, 0.0}, layers);
  ASSERT_EQ(r.chosen.size(), 2u);
  EXPECT_EQ(r.chosen[0], 1);
  EXPECT_EQ(r.chosen[1], 0);
  EXPECT_NEAR(r.total_cost, 1.1, 1e-9);
}

// Un layer vuoto (waypoint irraggiungibile) viene scavalcato: -1 per lui,
// la catena continua tra i layer vicini.
TEST(SequencePlanner, SkipsEmptyLayers)
{
  std::vector<std::vector<SeqCandidate>> layers = {
    { {{1.0}} },
    { },
    { {{5.0}}, {{1.5}} },
  };
  SeqResult r = choose_sequence({0.0}, layers);
  EXPECT_EQ(r.chosen[0], 0);
  EXPECT_EQ(r.chosen[1], -1);
  EXPECT_EQ(r.chosen[2], 1);
  EXPECT_NEAR(r.total_cost, 1.5, 1e-9);
}

// extra_cost entra nella scelta: due candidati identici nei giunti, vince
// quello con penalita' minore.
TEST(SequencePlanner, UsesExtraCost)
{
  std::vector<std::vector<SeqCandidate>> layers = {
    { {{1.0}, 0.5}, {{1.0}, 0.0} },
  };
  SeqResult r = choose_sequence({0.0}, layers);
  EXPECT_EQ(r.chosen[0], 1);
}

// Un arco bloccato viene evitato se esiste un'alternativa: il candidato
// piu' vicino (0.1) e' raggiungibile solo con un tratto bloccato, quindi
// vince l'altro (0.5).
TEST(SequencePlanner, AvoidsBlockedEdges)
{
  std::vector<std::vector<SeqCandidate>> layers = {
    { {{0.1}}, {{0.5}} },
  };
  auto edge_ok = [](int prev_layer, int /*prev_c*/, int layer, int c) {
    return !(prev_layer == -1 && layer == 0 && c == 0);
  };
  SeqResult r = choose_sequence({0.0}, layers, edge_ok);
  EXPECT_EQ(r.chosen[0], 1);
}

// Se TUTTI gli archi verso un layer sono bloccati, il waypoint non viene
// scartato: si paga la penalita' e si va comunque.
TEST(SequencePlanner, BlockedEdgesAreNotForbidden)
{
  std::vector<std::vector<SeqCandidate>> layers = { { {{1.0}} } };
  auto edge_ok = [](int, int, int, int) { return false; };
  SeqResult r = choose_sequence({0.0}, layers, edge_ok, 1000.0);
  EXPECT_EQ(r.chosen[0], 0);
  EXPECT_NEAR(r.total_cost, 1001.0, 1e-9);
}

// Nessun wrap: da +179 a -179 gradi sono ~358 gradi di rotazione.
TEST(JointL2, NoWrapAround)
{
  double d = joint_l2({3.1}, {-3.1});
  EXPECT_NEAR(d, 6.2, 1e-9);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
