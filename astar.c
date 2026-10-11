// Octilinear multi-layer A* over a cost raster.
//
// Cells are flat indices l*H*W + y*W + x. cost < 0 is blocked; otherwise it multiplies
// the step length. A layer change at (y,x) is a through via and needs viaok[y*W+x].
// tvia (optional): placing a via there completes the route (a plane connection).
// Turns: 45 deg costs turn45, 90 deg costs turn90, sharper turns are not allowed.
// dirCost (optional, 4 floats per cell): extra cost of entering a cell along each axis
// (0 horizontal, 1 diagonal \, 2 vertical, 3 diagonal /), e.g. for crossing other wiring.
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

typedef struct { float f; int32_t n; } HeapItem;

typedef struct { HeapItem *a; int64_t len, cap; } Heap;

static void heapPush(Heap *h, float f, int32_t n) {
  if (h->len == h->cap) {
    h->cap = h->cap ? h->cap * 2 : 1 << 16;
    h->a = (HeapItem *)realloc(h->a, (size_t)h->cap * sizeof(HeapItem));
  }
  int64_t i = h->len++;
  while (i > 0) {
    int64_t p = (i - 1) >> 1;
    if (h->a[p].f <= f) break;
    h->a[i] = h->a[p];
    i = p;
  }
  h->a[i].f = f;
  h->a[i].n = n;
}

static HeapItem heapPop(Heap *h) {
  HeapItem top = h->a[0];
  HeapItem last = h->a[--h->len];
  int64_t i = 0;
  for (;;) {
    int64_t c = 2 * i + 1;
    if (c >= h->len) break;
    if (c + 1 < h->len && h->a[c + 1].f < h->a[c].f) c++;
    if (h->a[c].f >= last.f) break;
    h->a[i] = h->a[c];
    i = c;
  }
  if (h->len > 0) h->a[i] = last;
  return top;
}

static const int DX[8] = {1, 1, 0, -1, -1, -1, 0, 1};
static const int DY[8] = {0, 1, 1, 1, 0, -1, -1, -1};

static inline float octile(int x, int y, int x0, int y0, int x1, int y1) {
  int dx = x < x0 ? x0 - x : (x > x1 ? x - x1 : 0);
  int dy = y < y0 ? y0 - y : (y > y1 ? y - y1 : 0);
  int mn = dx < dy ? dx : dy, mx = dx < dy ? dy : dx;
  return (float)(mx - mn) + 1.41421356f * (float)mn;
}

// Returns path length written to outPath (source first); 0 = no path; -1 = outPath too small.
// If the route ends in a terminal via, the last entry is -1.
int astar(int L, int H, int W, const float *cost, const uint8_t *viaok, float viaCost,
          const uint8_t *target, const uint8_t *tvia, const int32_t *src, int nsrc,
          float turn45, float turn90, float hweight, int tx0, int ty0, int tx1, int ty1,
          int32_t *outPath, int maxPath, int64_t maxExpand, int64_t *expandedOut,
          const float *dirCost, float viaCostFar) {
  const int64_t plane = (int64_t)H * W;
  const int64_t N = plane * L;
  const int32_t TERM = (int32_t)N;  // pseudo node: terminal via
  float *g = (float *)malloc((size_t)(N + 1) * sizeof(float));
  int32_t *par = (int32_t *)malloc((size_t)(N + 1) * sizeof(int32_t));
  int8_t *dir = (int8_t *)malloc((size_t)(N + 1));
  uint8_t *closed = (uint8_t *)calloc((size_t)(N + 1), 1);
  for (int64_t i = 0; i <= N; i++) g[i] = INFINITY;
  Heap h = {0};
  const int useH = tvia == NULL;
  for (int i = 0; i < nsrc; i++) {
    int32_t n = src[i];
    if (n < 0 || n >= N || cost[n] < 0) continue;
    g[n] = 0;
    par[n] = -1;
    dir[n] = 8;
    int yx = (int)(n % plane), y = yx / W, x = yx % W;
    heapPush(&h, useH ? hweight * octile(x, y, tx0, ty0, tx1, ty1) : 0, n);
  }
  int32_t end = -1;
  int64_t expanded = 0;
  while (h.len > 0) {
    HeapItem it = heapPop(&h);
    int32_t n = it.n;
    if (closed[n]) continue;
    closed[n] = 1;
    if (n == TERM || target[n]) { end = n; break; }
    if (++expanded > maxExpand) break;
    int l = (int)(n / plane);
    int yx = (int)(n % plane), y = yx / W, x = yx % W;
    float gn = g[n];
    int dn = dir[n];
    for (int d = 0; d < 8; d++) {
      float tc = 0;
      if (dn < 8) {
        int dd = abs(d - dn);
        if (dd > 4) dd = 8 - dd;
        if (dd >= 3) continue;
        tc = dd == 1 ? turn45 : (dd == 2 ? turn90 : 0);
      }
      int nx = x + DX[d], ny = y + DY[d];
      if (nx < 0 || ny < 0 || nx >= W || ny >= H) continue;
      int32_t m = (int32_t)(l * plane + (int64_t)ny * W + nx);
      float c = cost[m];
      if (c < 0 || closed[m]) continue;
      float step = (d & 1) ? 1.41421356f : 1.0f;
      float ng = gn + step * 0.5f * (c + cost[n]) + tc;
      if (dirCost) ng += dirCost[(int64_t)m * 4 + (d & 3)];
      if (ng < g[m]) {
        g[m] = ng;
        par[m] = n;
        dir[m] = (int8_t)d;
        heapPush(&h, ng + (useH ? hweight * octile(nx, ny, tx0, ty0, tx1, ty1) : 0), m);
      }
    }
    if (viaok[yx]) {
      const float vc = viaok[yx] == 2 ? viaCostFar : viaCost;
      for (int k = 0; k < L; k++) {
        if (k == l) continue;
        int32_t m = (int32_t)(k * plane + yx);
        if (cost[m] < 0 || closed[m]) continue;
        float ng = gn + vc;
        if (ng < g[m]) {
          g[m] = ng;
          par[m] = n;
          dir[m] = 8;
          heapPush(&h, ng + (useH ? hweight * octile(x, y, tx0, ty0, tx1, ty1) : 0), m);
        }
      }
      if (tvia && tvia[yx]) {
        float ng = gn + vc;
        if (ng < g[TERM]) {
          g[TERM] = ng;
          par[TERM] = n;
          heapPush(&h, ng, TERM);
        }
      }
    }
  }
  if (expandedOut) *expandedOut = expanded;
  int len = 0;
  if (end >= 0) {
    // count, then fill reversed
    int32_t c = end;
    while (c >= 0) { len++; c = par[c]; }
    if (len > maxPath) {
      len = -1;
    } else {
      c = end;
      for (int i = len - 1; i >= 0; i--) {
        outPath[i] = c == TERM ? -1 : c;
        c = par[c];
      }
    }
  }
  free(g); free(par); free(dir); free(closed); free(h.a);
  return len;
}

// Exact Euclidean distance transform (Felzenszwalb & Huttenlocher), squared, in cells.
// feat[i] != 0 marks a feature cell; out[i] = squared distance to the nearest one.
static void dt1d(const double *f, int n, double *d, int *v, double *z) {
  int k = 0;
  v[0] = 0;
  z[0] = -1e300;
  z[1] = 1e300;
  for (int q = 1; q < n; q++) {
    double s = ((f[q] + (double)q * q) - (f[v[k]] + (double)v[k] * v[k])) / (2.0 * (q - v[k]));
    while (s <= z[k]) {
      k--;
      s = ((f[q] + (double)q * q) - (f[v[k]] + (double)v[k] * v[k])) / (2.0 * (q - v[k]));
    }
    k++;
    v[k] = q;
    z[k] = s;
    z[k + 1] = 1e300;
  }
  k = 0;
  for (int q = 0; q < n; q++) {
    while (z[k + 1] < (double)q) k++;
    d[q] = (double)(q - v[k]) * (q - v[k]) + f[v[k]];
  }
}

void edt2d(const uint8_t *feat, int H, int W, float *out) {
  const double INF = 1e20;
  int n = H > W ? H : W;
  double *f = (double *)malloc(sizeof(double) * n);
  double *d = (double *)malloc(sizeof(double) * n);
  int *v = (int *)malloc(sizeof(int) * n);
  double *z = (double *)malloc(sizeof(double) * (n + 1));
  for (int x = 0; x < W; x++) {
    for (int y = 0; y < H; y++) f[y] = feat[(int64_t)y * W + x] ? 0 : INF;
    dt1d(f, H, d, v, z);
    for (int y = 0; y < H; y++) out[(int64_t)y * W + x] = (float)d[y];
  }
  for (int y = 0; y < H; y++) {
    float *row = out + (int64_t)y * W;
    for (int x = 0; x < W; x++) f[x] = row[x];
    dt1d(f, W, d, v, z);
    for (int x = 0; x < W; x++) row[x] = d[x];
  }
  free(f); free(d); free(v); free(z);
}
