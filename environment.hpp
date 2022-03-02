#ifndef ENVIRONMENT
#define ENVIRONMENT

#include <iostream>
#include <fstream>
#include <string.h>

/* ==============================Class======================================= */

class Environment
{
public:
    int width = 8, height = 8;
    int charging_station_x = 1, charging_station_y = 1;
    int **grid;
    Environment();
    Environment(int, int);
    Environment(int, int, int, int);
    Environment(std::string);
    void add_obstacle(int, int);
    void set_charging_station(int, int);
    void add_obstacle(int, int, int, int);
    void add_obstacle(std::string);
    void save_to_file(std::string);
    friend std::ostream &operator<<(std::ostream &, const Environment &);
};

#endif

/*==================Environment Test Functions=========================== */


void customInitialization(Environment*);
void fileInitialization(Environment*);
void saveMenu(Environment*);
void obstaclesMenu(Environment*);
