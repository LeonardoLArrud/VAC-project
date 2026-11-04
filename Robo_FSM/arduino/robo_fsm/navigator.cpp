// Grid mission of pi/navigators.py (class Mission), ported to the Arduino. See navigator.h.
#include "navigator.h"

#include <limits.h>
#include <stdio.h>
#include <string.h>

static const int8_t STEP_I[4] = {1, 0, -1, 0};   // STEP = [(1, 0), (0, 1), (-1, 0), (0, -1)]: E, N, W, S
static const int8_t STEP_J[4] = {0, 1, 0, -1};
static const int EXIT_BIT[4] = {EXIT_S, EXIT_L, 0, EXIT_R};   // EXIT = {0: "S", 1: "L", 3: "R"}, by (d - h) % 4
// Neighbour order of graph.track.grid() (dict insertion order): up, left, right, down = N, W, E, S.
// It decides which path dijkstra() returns when several have the same length.
static const int NEIGHBOUR_DIRS[4] = {1, 2, 0, 3};

static const char* const ORDER_NAMES[] = {"STRAIGHT", "LEFT", "U_TURN", "RIGHT", "STOP", "GO_ON"};
const char* orderName(Order o) { return ORDER_NAMES[o]; }

static bool same(Node a, Node b) { return a.i == b.i && a.j == b.j; }
static int mod4(int x) { return ((x % 4) + 4) % 4; }

// "(2, 3)", as Python prints a tuple
static void fmtNode(char* buf, int size, Node a) { snprintf(buf, size, "(%d, %d)", a.i, a.j); }

// "['L', 'S']", as Python prints sorted(exits)
static void fmtExits(char* buf, int size, int mask) {
  static const int BITS[3] = {EXIT_L, EXIT_R, EXIT_S};
  static const char* const NAMES[3] = {"'L'", "'R'", "'S'"};
  int k = snprintf(buf, size, "[");
  bool first = true;
  for (int t = 0; t < 3; t++)
    if (mask & BITS[t]) {
      k += snprintf(buf + k, size - k, "%s%s", first ? "" : ", ", NAMES[t]);
      first = false;
    }
  snprintf(buf + k, size - k, "]");
}

bool Mission::begin(Node start, int heading, Node goal, int n, long g) {
  if (n < 1 || n > N_MAX || heading < 0 || heading > 3) {
    snprintf(reason_, sizeof reason_, "bad mission: n must be 1..%d and heading 0..3", N_MAX);
    return false;
  }
  n_ = n;
  g_ = g;
  to_ = start;
  h_ = hIn_ = heading;
  goal_ = goal;
  node_ = start;
  hasNode_ = false;
  sAt_ = sNext_ = 0;
  memset(blocked_, 0, sizeof blocked_);
  snprintf(reason_, sizeof reason_, "mission set");
  return true;
}

bool Mission::inGrid(Node a) const { return a.i >= 0 && a.i < n_ && a.j >= 0 && a.j < n_; }
int Mission::index(Node a) const { return a.j * n_ + a.i; }
Node Mission::nodeAt(int k) const { return Node{(int8_t)(k % n_), (int8_t)(k / n_)}; }
Node Mission::step(Node a, int d) const { return Node{(int8_t)(a.i + STEP_I[d]), (int8_t)(a.j + STEP_J[d])}; }

int Mission::exits(Node a, int h) const {
  if (!inGrid(a)) return 0;             // map.adj.get(...) = {}
  int mask = 0;
  for (int d = 0; d < 4; d++)
    if (inGrid(step(a, d))) mask |= EXIT_BIT[mod4(d - h)];   // map.adj: blocked edges count too
  return mask;
}

bool Mission::blockEdge(Node a, Node b) {
  if (!inGrid(a) || !inGrid(b)) return false;   // no such edge: block_edge() raises KeyError
  for (int d = 0; d < 4; d++)
    if (same(step(a, d), b)) {
      blocked_[index(a)] |= 1 << d;
      blocked_[index(b)] |= 1 << mod4(d + 2);
      return true;
    }
  return false;
}

// dijkstra(): same order as the Python heap of (distance, push counter, node), so it
// returns the same path when several are equally short.
bool Mission::firstStep(Node from, int& dir) const {
  const long INF = LONG_MAX;
  const int count = n_ * n_;
  long dist[N_MAX * N_MAX];
  int8_t prev[N_MAX * N_MAX];
  uint16_t pushed[N_MAX * N_MAX];       // push counter of the current heap entry of each node
  bool done[N_MAX * N_MAX];
  for (int k = 0; k < count; k++) {
    dist[k] = INF;
    prev[k] = -1;
    pushed[k] = 0;
    done[k] = false;
  }
  const int start = index(from), goal = index(goal_);
  dist[start] = 0;
  uint16_t tie = 1;
  while (true) {
    int u = -1;                         // heappop(): smallest (distance, push counter)
    for (int k = 0; k < count; k++)
      if (!done[k] && dist[k] != INF &&
          (u < 0 || dist[k] < dist[u] || (dist[k] == dist[u] && pushed[k] < pushed[u])))
        u = k;
    if (u < 0) return false;            // goal unreachable: ([], inf)
    if (u == goal) break;
    done[u] = true;
    const Node a = nodeAt(u);
    for (int t = 0; t < 4; t++) {       // graph.neighbors(u): not blocked
      const int d = NEIGHBOUR_DIRS[t];
      const Node b = step(a, d);
      if (!inGrid(b) || (blocked_[u] & (1 << d))) continue;
      const int v = index(b);
      if (dist[u] + g_ < dist[v]) {
        dist[v] = dist[u] + g_;
        prev[v] = u;
        pushed[v] = tie++;
      }
    }
  }
  int v = goal;                         // _rebuild_path(): path[1] is the node after start
  while (prev[v] != start) v = prev[v];
  const Node b = nodeAt(v);
  for (int d = 0; d < 4; d++)
    if (same(step(from, d), b)) {
      dir = d;
      return true;
    }
  return false;
}

Order Mission::onNode(int ex, long sNode) {
  const Node node = to_;
  const int hIn = h_;
  char a[16], b[16], c[24], e[24];
  const int mapExits = exits(node, hIn);
  if (ex != EXITS_NONE && (ex & ~mapExits)) {
    fmtNode(a, sizeof a, node);
    fmtExits(e, sizeof e, ex);
    fmtExits(c, sizeof c, mapExits);
    snprintf(reason_, sizeof reason_, "pose lost at %s: saw %s, map has %s", a, e, c);
    return STOP;
  }
  node_ = node;
  hIn_ = hIn;
  sAt_ = sNode;
  sNext_ = sNode + g_;
  hasNode_ = true;
  fmtNode(a, sizeof a, node);
  if (same(node, goal_)) {
    snprintf(reason_, sizeof reason_, "arrived at %s", a);
    return STOP;
  }
  int d = 0, rel = 0;
  while (true) {
    if (!inGrid(node) || !inGrid(goal_) || !firstStep(node, d)) {   // (dijkstra() raises KeyError outside)
      snprintf(reason_, sizeof reason_, "no path from %s", a);
      return STOP;
    }
    rel = mod4(d - hIn);
    if (ex == EXITS_NONE || rel == 2 || (ex & EXIT_BIT[rel])) break;
    blockEdge(node, step(node, d));     // exit not there: drop the edge
  }
  h_ = d;
  to_ = step(node, d);
  fmtNode(b, sizeof b, to_);
  snprintf(reason_, sizeof reason_, "mission: at %s, next %s", a, b);
  return (Order)rel;
}

Order Mission::onBlocked(long s, long d) {
  if (!hasNode_) {                      // (navigators.py would crash here)
    snprintf(reason_, sizeof reason_, "blocked before the first node");
    return STOP;
  }
  const Node prev = step(node_, mod4(hIn_ + 2));
  const bool passed = s >= sAt_;
  Node e0 = node_, e1 = to_;
  if (!(s + d > sAt_)) {
    e0 = prev;
    e1 = node_;
  }
  char a[16], b[16], c[16];
  fmtNode(a, sizeof a, e0);
  fmtNode(b, sizeof b, e1);
  if (!blockEdge(e0, e1)) {             // (navigators.py would crash here: blocked twice on one edge, or off the grid)
    snprintf(reason_, sizeof reason_, "no edge %s-%s to block", a, b);
    return STOP;
  }
  const Node back = passed ? node_ : prev;
  const long sBack = passed ? sAt_ : sAt_ - g_;
  to_ = back;
  h_ = mod4((passed ? h_ : hIn_) + 2);
  sNext_ = 2 * s - sBack;               // the node behind is s - s_back away
  fmtNode(c, sizeof c, back);
  snprintf(reason_, sizeof reason_, "edge %s-%s blocked: back to %s", a, b, c);
  return U_TURN;
}

Order Mission::onExitBlocked(int ex) {
  if (!hasNode_) {                      // (navigators.py would crash here)
    snprintf(reason_, sizeof reason_, "exit blocked before the first node");
    return STOP;
  }
  blockEdge(node_, to_);
  to_ = node_;
  h_ = hIn_;
  return onNode(ex, sAt_);
}

Order Mission::onFault(int kind) {
  if (kind != FAULT_NODE_MISSED) {
    snprintf(reason_, sizeof reason_, "turn failed");
    return GO_ON;
  }
  node_ = to_;
  hIn_ = h_;
  sAt_ = sNext_;
  hasNode_ = true;
  to_ = step(to_, h_);
  sNext_ = sNext_ + g_;
  char a[16];
  fmtNode(a, sizeof a, node_);
  if (!inGrid(to_)) {
    snprintf(reason_, sizeof reason_, "pose lost: node %s missed at the border", a);
    return STOP;
  }
  snprintf(reason_, sizeof reason_, "node %s missed, counted", a);
  return GO_ON;
}

bool Mission::prior(long& sNext, int& ex) {
  if (!hasNode_) return false;
  sNext = sNext_;
  ex = exits(to_, h_);
  return true;
}
