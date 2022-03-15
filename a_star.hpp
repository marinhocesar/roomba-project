#ifndef A_STAR
#define A_STAR

#include "environment.hpp"
#include <cmath>
#include <vector>
#include <limits>
#include <algorithm>
#include <map>

int heuristic(int, int, int);
int smallest_fScore(std::vector<int>, int *);
void reconstruct_path(int *, std::map<int, int>, int, int, int);
std::vector<int> get_neighbors(Environment *, int, int);

/*==================Auxiliary Functions=========================== */
int get_index(int, int, int);
int get_x(int, int);
int get_y(int, int);

#endif