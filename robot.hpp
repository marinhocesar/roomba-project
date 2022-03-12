#ifndef ROBOT
#define ROBOT

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

class Laser
{
    Environment *current_envo;

public:
    Laser();
    Laser(Environment *);
    bool* calc_collision(int, int);
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
    void update_cell();
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
    
    
};

class Model2 : public Robot
{
    bool* neighbors;
    int angle; // 0=NW; 1=N; 2=NE; 3=W; 4=E; 5=SW; 6=S; 7=SE;
    Laser laser;
public:
    Model2(std::string, int, int, int, Environment *);
    Model2(std::string, Environment *);
    bool get_neighbor(int);
    void return_to_charger();
    void rotate();
    void advance();
    void clean();
    
    
};

/*=======================Roomba Test Functions=========================== */
void customRobot(Environment*, Robot*&);
void fileRobot(Environment*, Robot*&); //ainda falta
void modelRobot(std::string, int, int, int, Environment*, Robot*&);
void modelRobot(std::string, Environment*, Robot*&);

#endif