#include "a_star.hpp"

int get_index(int x, int y, int n_cols)
{
    return (x + y*n_cols);
}

int get_x(int index, int n_cols)
{
    return (index % n_cols);
}

int get_y(int index, int n_cols)
{
    return floor(index / n_cols);
}

int heuristic(int start_index, int end_index, int n_cols)
{
    int start_x = get_x(start_index, n_cols);
    int start_y = get_y(start_index, n_cols);
    int end_x = get_x(end_index, n_cols);
    int end_y = get_y(end_index, n_cols);
    int d = abs(end_x - start_x) + abs(end_y - start_y);
    return d;
}

int smallest_fScore(std::vector<int> open_set, int* f_score)
{
    int min_fScore = std::numeric_limits<int>::max();
    int min_index = 0;

    for (std::vector<int>::iterator it = begin(open_set); it != end(open_set); ++it)
    {
        int current_index = *it;

        if (f_score[current_index] < min_fScore)
        {
            min_fScore = f_score[current_index];
            min_index = current_index;
        }
    }
    return min_index;
}

// std::vector<int> get_neighbors(Environment* envo, int cell_index, int cols)
// {
//     std::vector<int> neighbors;
//     for (int i = -1; i<2; ++i)
//     {
//         for (int j = -1; j<2; ++j)
//         {
//             if (i==0 && j==0) { continue; }
//             int cell_x = get_x(cell_index, cols);
//             int cell_y = get_y(cell_index, cols);
//             int neighbor_x = cell_x + i;
//             int neighbor_y = cell_y + j;

//             bool cond1 = (neighbor_x< 0 || neighbor_x>envo->get_width()-1);
//             bool cond2 = (neighbor_y<0 || neighbor_y>envo->get_height()-1);
//             if (cond1 || cond2) 
//             {
//                 continue;
//             }

//             if (envo->get_grid()[neighbor_y][neighbor_x] != 1)
//             {
//                 int index = get_index(neighbor_x, neighbor_y, cols);
//                 neighbors.push_back(index);
//             }
//         }
//     }

//     return neighbors;
// }

