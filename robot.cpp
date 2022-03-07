#include <iostream>
#include <fstream>
#include <string.h>

/* ==============================Class======================================= */

class Robot
{
public:
    std::string name = "robot";
    int x_pos = 0, y_pos = 0;
    int max_battery = 100;
    bool has_charge = true;
    Robot(std::string, int, int, int);
    Robot(std::string);
    void stop_robot();
};

Robot::Robot(std::string name, int x, int y, int battery)
{
    // Constructor with user input
    x_pos = x;
    y_pos = y;
    max_battery = battery;

}

Robot::Robot(std::string filename)
{
    // Constructor with file info
    std::string file_info;
    std::ifstream f(filename); // Opens the file in input mode.
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
                max_battery = std::stoi(file_info_att);

                continue;
            }
        }

        f.close();
    }

}

/* ===========================Class Methods================================== */

void Robot::stop_robot()
{
    // Stops the robot
    has_charge = false;
}