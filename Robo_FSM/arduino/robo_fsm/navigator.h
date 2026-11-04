// Grid mission of pi/navigators.py (class Mission), ported to the Arduino.
// Same logic, line by line: graph.track.grid() + graph.path.dijkstra() + Mission.
// The only differences: distances are integers in mm (the Pi sends s and d in mm),
// and the cases where the Python code would crash return STOP instead.
#ifndef NAVIGATOR_H
#define NAVIGATOR_H

#include <stdint.h>

// orders; the first four in the order of ORDER = ["STRAIGHT", "LEFT", "U_TURN", "RIGHT"]
enum Order { STRAIGHT = 0, LEFT = 1, U_TURN = 2, RIGHT = 3, STOP = 4, GO_ON = 5 };
const char* orderName(Order o);

// exits seen at a node, as bits (the Python set of "S", "L", "R")
const int EXIT_S = 1, EXIT_L = 2, EXIT_R = 4;
const int EXITS_NONE = -1;              // exits = None: not seen, use the map

// on_fault(kind)
const int FAULT_NODE_MISSED = 0, FAULT_TURN_FAILED = 1;

const int N_MAX = 7;                    // largest grid: N_MAX x N_MAX nodes

struct Node {
  int8_t i, j;                          // as in navigators.py: i to the east, j to the north
};

class Mission {                         // goal on the grid, path recomputed at every node
 public:
  bool begin(Node start, int heading, Node goal, int n, long g);   // Mission(start, heading, goal, n, g); heading 0..3 = E N W S
  Order onNode(int exits, long sNode);  // on_node(exits, s_node); exits EXITS_NONE: from the map
  Order onBlocked(long s, long d);      // on_blocked(s, d): drop the edge of the obstacle, back to the node behind
  Order onFault(int kind);              // on_fault(kind): node missed is counted as passed
  Order onExitBlocked(int exits);       // on_exit_blocked(exits): sonar at the node, drop the exit just chosen, choose again
  bool prior(long& sNext, int& exits);  // prior(): false = None
  const char* why() const { return reason_; }

 private:
  int n_ = 0;
  long g_ = 0;                          // mm between nodes
  Node to_ = {0, 0}, goal_ = {0, 0}, node_ = {0, 0};
  bool hasNode_ = false;                // self.node is not None
  int h_ = 0, hIn_ = 0;
  long sAt_ = 0, sNext_ = 0;
  uint8_t blocked_[N_MAX * N_MAX];      // per node: bit d set = edge towards direction d blocked
  char reason_[96];

  bool inGrid(Node a) const;
  int index(Node a) const;
  Node nodeAt(int k) const;
  Node step(Node a, int d) const;       // _step()
  int exits(Node a, int h) const;       // _exits(): exits the map has at a, arriving with heading h
  bool firstStep(Node from, int& d) const;   // dijkstra(map, from, goal): direction of path[1]
  bool blockEdge(Node a, Node b);       // map.block_edge(); false = no such edge
};

#endif
