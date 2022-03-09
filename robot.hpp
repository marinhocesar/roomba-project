#ifndef ROBOT
#define ROBOT

#include <iostream>
#include <fstream>
#include <string.h>
#include "environment.hpp"

/* ==============================Class Prototypes============================ */

class Bumper
{
    Environment *current_envo;

public:
    Bumper();
    Bumper(Environment *);
    bool calc_collision(int, int);
};

class Battery
{
    int max_battery;
    int current_battery;

public:
    Battery();
    Battery(int);
    void discharge();
    void charge();
    int get_battery_level();
    void show_battery();
};

class Robot
{
protected:
    Environment *current_envo;
    std::string name = "robot";
    int x_pos = 0, y_pos = 0;
    Battery battery;

public:
    Robot();
    Robot(std::string, int, int, int, Environment *);
    Robot(std::string, Environment *);
    virtual void clean(){
        std::cout << "cleaning" << std::endl;
    }
    
    bool stop_robot();
    void show_battery();
    bool has_charge();
    Environment* get_environment();
};

class Model1 : public Robot
{

    Bumper bumper;
public:
    Model1(std::string, int, int, int, Environment *);
    Model1(std::string, Environment *);
    void return_to_charger();
    void clean();
    void update_cell();
    
};

#endif

/*=======================Roomba Test Functions=========================== */
void customRobot(Environment*, Robot*&);
void fileRobot(Environment*, Robot*&); //ainda falta
void modelRobot(std::string, int, int, int, Environment*, Robot*&);
