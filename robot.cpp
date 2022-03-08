#include "environment.hpp"

/* =============================Bumper====================================== */

class Bumper
{
public:
    Bumper();
    Bumper(Environment*);
    Environment* current_envo;
    bool calc_collision(int, int);
};

Bumper::Bumper()
{

}

Bumper::Bumper(Environment* p_a)
{
    current_envo = p_a;
}

bool Bumper::calc_collision(int x, int y)
{
    if (x < 0 || x >= current_envo->width || y < 0 || y >= current_envo->height)
    {
        // Environment borders
        return true;
    }

    if ((*current_envo).grid[y][x] == 1)
    {
        // Obstacle
        return true;
    }
    else
    {
        // Free path
        return false;
    }

}

/* =====================Battery Class======================================= */

class Battery
{

    int max_battery = 100;
    int current_battery = 100;

public:
    Battery();
    Battery(int);
    void discharge();
    void charge();
    int battery_level();
    void show_battery();
};

Battery::Battery()
{
}

Battery::Battery(int max_capacity)
{
    max_battery = max_capacity;
    current_battery = max_capacity;
}

void Battery::discharge()
{
    --current_battery;
}

void Battery::charge()
{
    ++current_battery;
}

int Battery::battery_level()
{
    return current_battery;
}

void Battery::show_battery()
{
    std::cout << current_battery << std::endl;
}

/* ======================Robot Class======================================= */

class Robot
{
public:
    Environment* current_envo;
    std::string name = "robot";
    int x_pos = 0, y_pos = 0;
    Battery battery;
    bool has_charge = true;
    Robot();
    Robot(std::string, int, int, int, Environment*);
    Robot(std::string, Environment*);
    void stop_robot();
};

// Robot::Robot()
// {
//     // Constructor without user input
//     x_pos = 0;
//     y_pos = 0;
//     battery = Battery();
//     Environment envo = Environment();
//     current_envo = &envo;
// }

Robot::Robot(std::string name, int x, int y, int capacity, Environment* p_a)
{
    // Constructor with user input
    x_pos = x;
    y_pos = y;
    current_envo = p_a;
    battery = Battery(capacity);
    p_a->grid[y][x] = 3;

}

Robot::Robot(std::string filename, Environment* p_a)
{
    // Constructor with file info
    std::string file_info;
    std::ifstream f(filename); // Opens the file in input mode.
    int capacity = 100;
    if (f.is_open())
    {
        while (getline(f, file_info))
        {
            int start = file_info.find("=") + 1;
            int end = file_info.length();
            std::string file_info_att = file_info.substr(start, end - start);
            if (file_info.find("$") != std::string::npos)
            {
                break;
            }
            if (file_info.find("name") != std::string::npos)
            {
                name = file_info_att;

                continue;
            }
            if (file_info.find("robot_x") != std::string::npos)
            {
                x_pos = std::stoi(file_info_att);
                
                continue;
            }
            if (file_info.find("robot_y") != std::string::npos)
            {
                y_pos = std::stoi(file_info_att);

                continue;
            }
            if (file_info.find("battery") != std::string::npos)
            {
                capacity = std::stoi(file_info_att);

                continue;
            }
        }

        f.close();
    }
    battery = Battery(capacity);
    current_envo = p_a;
    p_a->grid[y_pos][x_pos] = 3;

}

/* ===========================Class Methods================================ */

void Robot::stop_robot()
{
    // Stops the robot
    has_charge = false;
}

/* =============================Model 1===================================== */

class Model1 : public Robot
{
    public:
    Bumper bumper;
    Model1(std::string, int, int, int, Environment*);
    Model1(std::string, Environment*);
    void clean();
    void return_to_charger();
    void update_pos();
};

Model1::Model1(std::string name, int x, int y, int capacity, Environment* p_a) : Robot(name, x, y, capacity, p_a)
{
    bumper = Bumper(p_a);
}

Model1::Model1(std::string filename, Environment* p_a) : Robot(filename, p_a)
{
    bumper = Bumper(p_a);
}

void Model1::update_pos()
{
    

    int charger_x = current_envo->charging_station_x - 1;
    int charger_y = current_envo->charging_station_y - 1;
    if (x_pos == charger_x && y_pos == charger_y){
        current_envo->grid[y_pos][x_pos] = 4;
    }
    else 
    {
        current_envo->grid[y_pos][x_pos] = 0;
    }

    int dir = (rand() % 4); // Pick a direction

    if (dir == 0 && !(bumper.calc_collision(x_pos - 1, y_pos)))
    {
        // Left
        --x_pos;
        battery.discharge();
        current_envo->grid[y_pos][x_pos] = 3;
        return;
    }
    if (dir == 1 && !(bumper.calc_collision(x_pos, y_pos - 1)))
    {
        // Up
        --y_pos;
        battery.discharge();
        current_envo->grid[y_pos][x_pos] = 3;
        return;
    }
    if (dir == 2 && !(bumper.calc_collision(x_pos + 1, y_pos)))
    {
        // Right
        ++x_pos;
        battery.discharge();
        current_envo->grid[y_pos][x_pos] = 3;
        return;
    }
    if (dir == 3 && !(bumper.calc_collision(x_pos, y_pos + 1)))
    {
        // Down
        ++y_pos;
        battery.discharge();
        current_envo->grid[y_pos][x_pos] = 3;
        return;
    }

        current_envo->grid[y_pos][x_pos] = 3;
        update_pos();
}


/* ======================================================================== */

int main()
{
    Environment room = Environment("environment_info.txt");
    Environment* p_a = &room;
    Model1 bob = Model1("robo1", 0, 0, 100, p_a);
    // bob.battery.show_battery();
    // bob.battery.discharge();
    // bob.battery.show_battery();
    // bob.battery.discharge();
    // bob.battery.show_battery();
    // std::cout << room << std::endl;
    // bob.update_pos();
    // std::cout << room << std::endl;
    // bob.update_pos();
    // std::cout << room << std::endl;
    // bob.update_pos();
    // std::cout << room << std::endl;
    // bob.update_pos();
    // std::cout << room << std::endl;
    while (bob.battery.battery_level() > 0)
    {
        bob.update_pos();
        bob.battery.show_battery();
        std::cout << room << std::endl;
    }
    


    return 0;
}