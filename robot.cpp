#include <iostream>
#include <fstream>
#include <string.h>

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

void Battery::show_battery()
{
    std::cout << current_battery << std::endl;
}

/* ======================Robot Class======================================= */

class Robot
{
public:
    std::string name = "robot";
    int x_pos = 0, y_pos = 0;
    Battery battery;
    bool has_charge = true;
    Robot();
    Robot(std::string, int, int, int);
    Robot(std::string);
    void stop_robot();
};

Robot::Robot()
{
    // Constructor without user input
    x_pos = 0;
    y_pos = 0;
    battery = Battery();
}

Robot::Robot(std::string name, int x, int y, int capacity)
{
    // Constructor with user input
    x_pos = x;
    y_pos = y;
    battery = Battery(capacity);

}

Robot::Robot(std::string filename)
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

}

/* ===========================Class Methods================================ */

void Robot::stop_robot()
{
    // Stops the robot
    has_charge = false;
}


/* ======================================================================== */

int main()
{
    Robot bob = Robot();
    bob.battery.show_battery();
    bob.battery.discharge();
    bob.battery.show_battery();
    bob.battery.discharge();
    bob.battery.show_battery();
    return 0;
}