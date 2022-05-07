#include "utils/FuncThread.hpp"
//
#include <cmath>
#include <random>
#include <utility>

#include "TSP/TSP.hpp"
#include "bresenham.hpp"
#include "devices.hpp"

extern "C" {
#include "SEEKFREE_IPS114_SPI.h"
}
using namespace std;
using namespace TSP;
using namespace imgProc;

constexpr int N = IPS114_H, M = IPS114_W;

static void drawPoint(int x, int y) { ips114_drawpoint(x, y, RED); }
static void erasePoint(int x, int y) { ips114_drawpoint(x, y, WHITE); }

static TSP_Solver tsp;
static int points[MAXN][2];

static inline void randPoints() {
    static default_random_engine rd;
    static uniform_int_distribution<int> dist(0, N * M - 1);
    for (int i = 0; i < MAXN; ++i) {
        int p = dist(rd);
        points[i][0] = p / M, points[i][1] = p % M;
    }
}

static inline void drawPoints(bool erase = false) {
    for (int i = 0; i < MAXN; ++i) drawCircle(points[i][0], points[i][1], 7, erase ? erasePoint : drawPoint);
}

static inline void drawPath(bool erase = false) {
    for (int i = 0; i < MAXN; ++i) {
        int u = tsp.hamilton_path[i > 0 ? i - 1 : MAXN - 1], v = tsp.hamilton_path[i];
        drawLine(points[u][0], points[u][1], points[v][0], points[v][1], erase ? erasePoint : drawPoint);
    }
}

static inline void calcDist() {
    for (int i = 0; i < MAXN; ++i) {
        tsp.dist[i][i] = 0;
        for (int j = i + 1; j < MAXN; ++j) {
            int dx = points[i][0] - points[j][0], dy = points[i][1] - points[j][1];
            tsp.dist[i][j] = tsp.dist[j][i] = sqrt(dx * dx + dy * dy);
        }
    }
}

static void testTSPSolverEntry() {
    ips114_clear(WHITE);
    tsp.N = MAXN;
    for (;;) {
        randPoints();
        calcDist();
        tsp.solve();
        drawPoints();
        drawPath();
        rt_thread_mdelay(1);
        drawPoints(true);
        drawPath(true);
    }
}

bool testTSPSolverNode() { return FuncThread(testTSPSolverEntry, "testTSPSolver"); }
