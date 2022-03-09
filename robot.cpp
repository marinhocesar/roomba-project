#include "robot.hpp"

/* =============================Bumper====================================== */


Bumper::Bumper()
{

}

Bumper::Bumper(Environment* p_a)
{
    current_envo = p_a;
}

bool Bumper::calc_collision(int x, int y)
{

    if (x < 0 || x >= current_envo->get_width() || y < 0 || y >= current_envo->get_height())
    {
        // Environment borders
        std::cout << "Wall detected ";
        return true;
    }

    if (current_envo->get_grid()[y][x] == 1)
    {
        // Obstacle
        std::cout << "Obstacle detected ";
        return true;
    }
    else
    {
        // Free path
        return false;
    }

}

/* =====================Battery Class======================================= */

Battery::Battery()
{
    max_battery = 100;
    current_battery = 100;
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

int Battery::get_battery_level()
{
    return current_battery;
}

/* ======================Robot Class======================================= */

Robot::Robot()
{

}

Robot::Robot(std::string robot_name, int x, int y, int capacity, Environment* p_a)
{
    // Constructor with user input
    x_pos = x;
    y_pos = y;
    current_envo = p_a;
    battery = Battery(capacity);
    p_a->set_element(x, y, 3);
    name = robot_name;

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
                x_pos = std::stoi(file_info_att) - 1;
                
                continue;
            }
            if (file_info.find("robot_y") != std::string::npos)
            {
                y_pos = std::stoi(file_info_att) - 1;

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
    bool cond1 = (x_pos < 1 || x_pos > p_a->get_width());
    bool cond2 = (y_pos < 1 || y_pos > p_a->get_height());
    if (cond1 || cond2)
    {
        std::cout << "File information for robot starting position is invalid";
        std::cout << ".\n" << "Robot will start at charging station.";
        x_pos = p_a->get_charging_x();
        y_pos = p_a->get_charging_y();
    }
    bool cond3 = p_a->get_grid()[y_pos-1][x_pos-1] == 1;
    if (cond1 || cond2)
    {
        std::cout << "The robot's starting position has to be free of obstacles.";
        std::cout << ".\n" << "Robot will start at charging station.";
        x_pos = p_a->get_charging_x();
        y_pos = p_a->get_charging_y();
    }

    p_a->set_element(x_pos, y_pos, 3);
}

/* ===========================Class Methods================================ */

bool Robot::stop_robot()
{
    // Stops the robot
    if (has_charge()){
        return false;
    }
    return true;
}


void Robot::show_battery()
{
    std::cout << name << "'s current battery level: " << battery.get_battery_level() << std::endl;
}

bool Robot::has_charge()
{
    if (battery.get_battery_level() < 1)
    {
        return false;
    }
    return true;
}

Environment* Robot::get_environment()
{
    return current_envo;
}

/* =============================Model 1===================================== */


Model1::Model1(std::string name, int x, int y, int capacity, Environment* p_a) : Robot(name, x, y, capacity, p_a)
{
    bumper = Bumper(p_a);
}

Model1::Model1(std::string filename, Environment* p_a) : Robot(filename, p_a)
{
    bumper = Bumper(p_a);
}

void Model1::clean()
{
    if (!has_charge())
    {
        return;
    }

    std::string direction[4] = {"left", "up", "right", "down"};
    int charger_x = current_envo->get_charging_x() - 1;
    int charger_y = current_envo->get_charging_y() - 1;
    if (x_pos == charger_x && y_pos == charger_y){
        current_envo->set_element(x_pos, y_pos, 4);
    }
    else 
    {
        current_envo->set_element(x_pos, y_pos, 0);
    }

    int dir = (rand() % 4); // Pick a direction

    if (dir == 0 && !(bumper.calc_collision(x_pos - 1, y_pos)))
    {
        // Left
        --x_pos;
        std::cout << "Moving " << direction[dir] << std::endl;
        update_cell();
        return;
    }
    if (dir == 1 && !(bumper.calc_collision(x_pos, y_pos - 1)))
    {
        // Up
        --y_pos;
        std::cout << "Moving " << direction[dir] << std::endl;
        update_cell();
        return;
    }
    if (dir == 2 && !(bumper.calc_collision(x_pos + 1, y_pos)))
    {
        // Right
        ++x_pos;
        std::cout << "Moving " << direction[dir] << std::endl;
        update_cell();
        return;
    }
    if (dir == 3 && !(bumper.calc_collision(x_pos, y_pos + 1)))
    {
        // Down
        ++y_pos;
        std::cout << "Moving " << direction[dir] << std::endl;
        update_cell();
        return;
    }
    std::cout << "while trying to move " << direction[dir];
    std::cout << "!"<< std::endl;
    update_cell();
    std::cout << "\n";
    clean();
}

void Model1::update_cell(){
    battery.discharge();
    show_battery();
    current_envo->set_element(x_pos, y_pos, 3);
}


/* ======================================================================== */

void customRobot(Environment* p_r, Robot*& p_rob)
{
    int x = 0, y = 0, battery_capacity = 0;
    std::cout << "\nCustom Robot" << std::endl;
    std::string robot_name = "";
    std::cout << "Enter robot name: ";
    std::cin.ignore();
    std::getline(std::cin, robot_name);
    if (robot_name == "")
    {
        robot_name = "generic_robot";
    }
    std::cout << "The coordinates must be integers." << std::endl;
    std::cout << "Enter the X coordinate for the starting position: ";
    std::cin >> x;
    std::cout << "Enter the Y coordinate for the starting position: ";
    std::cin >> y;
    std::cout << "Enter battery capactiy: ";
    std::cin >> battery_capacity;

    bool cond1 = (x < 1 || y < 1 || x > p_r->get_width() || y > p_r->get_height());
    bool cond2 = (p_r->get_grid())[y-1][x-1] == 1;
    if (cond1)
    {
        std::cout << "Invalid arguments." << std::endl;
        return customRobot(p_r, p_rob);
    }
    if (cond2)
    {
        std::cout << "The robot's starting position has to be free of obstacles.";
        std::cout << "\n";
        return customRobot(p_r, p_rob);
    }

    return modelRobot(robot_name, x-1, y-1, battery_capacity, p_r, p_rob);
}

void fileRobot(Environment* p_r, Robot*& p_rob)
{
    std::string filename = "";
    std::cout << "\nFile Initialization" << std::endl;
    std::cout << "Enter filename: ";
    std::cin.ignore();
    std::getline(std::cin, filename);
    if (filename == "")
    {
        filename = "robot_info.txt";
    }
    
    return modelRobot(filename, p_r, p_rob);
}

void modelRobot(std::string name, int x, int y, int btr_cp, Environment* p_r, Robot*& p_rob)
{
    std::cout << "\nChose the model of the robot" << std::endl;
    std::cout << "1- Model1." << std::endl;
    std::cout << "2- Model2." << std::endl;
    int answer = -1;
    std::cin >> answer;
    if (answer != 1 && answer != 2)
    {
        std::cout << "Sorry, this is not an option." << std::endl;
        return modelRobot(name, x, y, btr_cp, p_r, p_rob);
    }

    if (answer == 1)
    {

        p_rob = new Model1(name, x, y, btr_cp, p_r);
        return;
    }

    std::cout << "This option is not yet available!" << std::endl;
    
    return modelRobot(name, x, y, btr_cp, p_r, p_rob);
}

void modelRobot(std::string filename, Environment* p_r, Robot*& p_rob)
{
    // Overloading for robot initialization from file
    std::cout << "\nChose the model of the robot" << std::endl;
    std::cout << "1- Model1." << std::endl;
    std::cout << "2- Model2." << std::endl;
    int answer = -1;
    std::cin >> answer;
    if (answer != 1 && answer != 2)
    {
        std::cout << "Sorry, this is not an option." << std::endl;
        return modelRobot(filename, p_r, p_rob);
    }

    if (answer == 1)
    {
        
        std::cout << "aqui 355!" << std::endl;
        p_rob = new Model1(filename, p_r);
        return;
    }

    std::cout << "This option is not yet available!" << std::endl;
    
    return modelRobot(filename, p_r, p_rob);
}