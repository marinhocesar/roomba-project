#ifndef ENVIRONMENT
#define ENVIRONMENT

#include <iostream>
#include <fstream>
#include <string.h>

/* ==============================Class======================================= */

class Environment
{
    int width = 8, height = 8;
    int charging_station_x = 1, charging_station_y = 1;
    int **grid;
public:
    Environment();
    Environment(int, int);
    Environment(int, int, int, int);
    Environment(std::string);
    void add_obstacle(int, int);
    void delete_obstacle(int, int);
    void set_charging_station(int, int);
    void add_obstacle(int, int, int, int);
    void add_obstacle(std::string);
    void save_to_file(std::string);
    void set_element(int, int, int);
    int get_width();
    int get_height();
    int get_charging_x();
    int get_charging_y();
    int** get_grid();
    friend std::ostream &operator<<(std::ostream &, const Environment &);
};

/*==================Environment Test Functions=========================== */

void standardInitialization(Environment*);
void customInitialization(Environment*);
void fileInitialization(Environment*);
void saveMenu(Environment*);
void obstaclesMenu(Environment*);
void moveChargingStation(Environment*);
void addSingularObstacle(Environment*);
void removeObstacles(Environment*);
void addRectOfObstacles(Environment*);
void addObstaclesFromFile(Environment*);

#endif