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
/* =============================Laser====================================== */


Laser::Laser()
{

}

Laser::Laser(Environment* p_a)
{
    current_envo = p_a;
}

bool* Laser::calc_collision(int x, int y)
{
    /*
    Returns an array of booleans where each element is true or false depending
    on the existence of an obstacle at the corresponding position on the grid 

    0 1 2       0 0 0
    3 X 4   ->  0 3 1
    5 6 7       0 1 1
    ^ indexes of elements surrounding X
    
    For the case above, a function call in X
    would return {false, false, false, false, true, false, true, true}
    */
    bool* neighbors = new bool[8];
    int count = 0;

    for (int j=-1; j<2; ++j)
    {
        for (int i=-1; i<2; ++i)
        {
            if (i == 0 && j == 0)
            {
                continue;
            }
            int neigh_x = x + i;
            int neigh_y = y + j;
            bool cond1 = (neigh_x < 0 || neigh_x >= current_envo->get_width());
            bool cond2 = (neigh_y < 0 || neigh_y >= current_envo->get_height());
            if (cond1 || cond2)
            {
                // Environment borders
                neighbors[count] = true;
                ++count;
                continue;
            }

            if (current_envo->get_grid()[neigh_y][neigh_x] == 1)
            {
                // Obstacle
                neighbors[count] = true;
                ++count;
                continue;
            }
            
            // Free path
            neighbors[count] = false;
            ++count;
        }
    }
    return neighbors;
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

int Battery::get_max_battery()
{
    return max_battery;
}

/* ======================Robot Class======================================= */

Robot::Robot()
{
    // 
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
    bool cond1 = (x_pos < 0 || x_pos > p_a->get_width() - 1);
    bool cond2 = (y_pos < 0 || y_pos > p_a->get_height() - 1);
    if (cond1 || cond2)
    {
        std::cout << "File information for robot starting position is invalid";
        std::cout << ".\n" << "Robot will start at charging station.";
        x_pos = p_a->get_charging_x() - 1;
        y_pos = p_a->get_charging_y() - 1;
    }

    bool cond3 = p_a->get_grid()[y_pos][x_pos] == 1;
    if (cond3)
    {
        std::cout << "The robot's starting position has to be free of obstacles.";
        std::cout << ".\n" << "Robot will start at charging station.";
        x_pos = p_a->get_charging_x() - 1;
        y_pos = p_a->get_charging_y() - 1;
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

void Robot::update_cell(){
    battery.discharge();
    show_battery();
    current_envo->set_element(x_pos, y_pos, 3);
}

Environment* Robot::get_environment()
{
    return current_envo;
}

int Robot::get_x_pos()
{
    return x_pos;
}

int Robot::get_y_pos()
{
    return y_pos;
}

void Robot::a_star(Environment* envo, int x_start, int y_start)
{
    returning = true;
    std::cout << "Returning to charging station." << std::endl;
    int cols = envo->get_width();
    int rows = envo->get_height();

    int* arr = new int[cols*rows];
    int* f_score = new int[cols*rows];
    int* g_score = new int[cols*rows];
    int* h_score = new int[cols*rows];
    std::map<int, int> came_from;

    int start_index = get_index(x_start, y_start, cols);
    
    int end_x  = envo->get_charging_x() - 1;
    int end_y = envo->get_charging_y() - 1;
    int end_index = get_index(end_x, end_y, cols); 
    

    for(int j = 0; j<rows; ++j)
    {
        for(int i = 0; i<cols; ++i)
        {
            int index = get_index(i, j, cols);
            arr[index] = envo->get_grid()[j][i];
            f_score[index] = std::numeric_limits<int>::max();
            g_score[index] = std::numeric_limits<int>::max();
            h_score[index] = heuristic(index, end_index, cols);
        }
    }

    std::vector<int> open_set;
    open_set.push_back(start_index);
    g_score[start_index] = 0;
    f_score[start_index] = h_score[start_index];


    while (!open_set.empty())
    {
        int current_index = smallest_fScore(open_set, f_score);

        if (current_index == end_index)
        {
            return reconstruct_path(arr, came_from, current_index, rows, cols);
        }


        // Find current in open_set and remove it
        
        
        // Finds the first occurrence of current_index in open_set
        std::vector<int>::iterator current_in_open;
        current_in_open	= std::find(open_set.begin(), open_set.end(), current_index);

        if (current_in_open != open_set.end())
        {
            open_set.erase(current_in_open); // Deletes the current_index from the open set
        }

        std::vector<int> neighbors = get_neighbors(envo, current_index, cols);


        for (std::vector<int>::iterator it = neighbors.begin(); it != neighbors.end(); ++it)
        {
            int neighbor = *it;
            int tentative_score = g_score[current_index] + heuristic(current_index, neighbor, cols);
            if (tentative_score < g_score[neighbor])
            {
                came_from[neighbor] = current_index;
                g_score[neighbor] = tentative_score;
                f_score[neighbor] = tentative_score + h_score[neighbor];

                std::vector<int>::iterator neighbor_in_open;
                neighbor_in_open = std::find(open_set.begin(), open_set.end(), neighbor);
                
                if (neighbor_in_open == open_set.end())
                {
                    // If the neighbor is not in open_set
                    open_set.push_back(neighbor);
                }
            }
        }
    }

    return;
}

void Robot::reconstruct_path(int* arr, std::map<int,int> came_from, int current_index, int rows, int cols)
{
    std::vector<int> total_path;
    total_path.push_back(current_index);

    int size = rows*cols;

    while (came_from.count(current_index))
    {
        // While current exists as a key in came_from
        current_index = came_from[current_index];
        total_path.insert(total_path.begin(), current_index);
    }

    
    for (std::vector<int>::iterator it = total_path.begin(); it!= total_path.end(); ++it)
    {
        int index = *it;
        int x = get_x(index, current_envo->get_width());
        int y = get_y(index, current_envo->get_width());
        if (has_charge())
        {
            go_to(x, y);
        }
        else
        {
            stop_robot();
            break;
        }
    }

    return;
}

std::vector<int> Model2::get_neighbors(Environment* envo, int cell_index, int cols)
{
    std::vector<int> neighbors;
    for (int i = -1; i<2; ++i)
    {
        for (int j = -1; j<2; ++j)
        {
            if (i==0 && j==0) { continue; }
            int cell_x = get_x(cell_index, cols);
            int cell_y = get_y(cell_index, cols);
            int neighbor_x = cell_x + i;
            int neighbor_y = cell_y + j;

            bool cond1 = (neighbor_x< 0 || neighbor_x>envo->get_width()-1);
            bool cond2 = (neighbor_y<0 || neighbor_y>envo->get_height()-1);
            if (cond1 || cond2) 
            {
                continue;
            }

            if (envo->get_grid()[neighbor_y][neighbor_x] != 1)
            {
                int index = get_index(neighbor_x, neighbor_y, cols);
                neighbors.push_back(index);
            }
        }
    }

    return neighbors;
}

void Robot::go_to(int x, int y)
{
    reset_cell();
    x_pos = x;
    y_pos = y;
    update_cell();
    std::cout << *current_envo << std::endl;
}

void Robot::reset_cell()
{
    int charger_x = current_envo->get_charging_x() - 1;
    int charger_y = current_envo->get_charging_y() - 1;
    if (x_pos == charger_x && y_pos == charger_y){
        current_envo->set_element(x_pos, y_pos, 4);
    }
    else 
    {
        current_envo->set_element(x_pos, y_pos, 0);
    }
}

void Robot::cleaning_routine()
{
    while (!stop_robot())
    {   
        clean();
        std::cout << *current_envo << std::endl;
        int charger_x = current_envo->get_charging_x() - 1;
        int charger_y = current_envo->get_charging_y() - 1;
        if (charger_x == x_pos && charger_y == y_pos && returning)
        {
            return charging();
        } 
    }
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
    reset_cell();

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


/* =============================Model 2===================================== */


Model2::Model2(std::string name, int x, int y, int capacity, Environment* p_a) : Robot(name, x, y, capacity, p_a)
{
    angle = rand() % 8;
    laser = Laser(p_a);
    neighbors = laser.calc_collision(x_pos, y_pos);
}

Model2::Model2(std::string filename, Environment* p_a) : Robot(filename, p_a)
{
    angle = rand() % 8;
    laser = Laser(p_a);
    neighbors = laser.calc_collision(x_pos, y_pos);
}

void Model2::clean()
{
    if (!has_charge())
    {
        // Won't clean if has no charge
        return;
    }

    if (returning)
    {
        return;
    }

    if (battery.get_battery_level() < 0.5*battery.get_max_battery())
    {
        // Won't clean if has no charge
        return a_star(current_envo, x_pos, y_pos);
    }

    if (!neighbors[angle])
    {
        // If the direction that it faces is clear, the robot advances
        advance();
    }
    else
    {
        /*
        If the direction the robot faces has an obstacle,
        it picks other direction.
        */
        rotate();
        advance();
    }
}

bool Model2::get_neighbor(int index)
{
    if (index<0 || index>7)
    {
        std::cout << "Neighboring cells must be refered with index values";
        std::cout << " between 0 and 7" << std::endl;
        return true;
    }
    return neighbors[index];
}

void Model2::rotate()
{
    angle = rand() % 8;
    if (neighbors[angle])
    {   
        return rotate();
    }
    std::cout << "Rotating..." << std::endl;
    battery.discharge();
    show_battery();
}

void Model2::advance()
{
    int charger_x = current_envo->get_charging_x() - 1;
    int charger_y = current_envo->get_charging_y() - 1;
    if (x_pos == charger_x && y_pos == charger_y){
        current_envo->set_element(x_pos, y_pos, 4);
    }
    else 
    {
        current_envo->set_element(x_pos, y_pos, 0);
    }

    if (angle == 0)
    {
        --x_pos;
        --y_pos;
    }
    else if (angle == 1)
    {
        --y_pos;
    }
    else if (angle == 2)
    {
        ++x_pos;
        --y_pos;
    }
    else if (angle == 3)
    {
        --x_pos;
    }
    else if (angle == 4)
    {
        ++x_pos;
    }
    else if (angle == 5)
    {
        --x_pos;
        ++y_pos;
    }
    else if (angle == 6)
    {
        ++y_pos;
    }
    else if (angle == 7)
    {
        ++x_pos;
        ++y_pos;
    }
    neighbors = laser.calc_collision(x_pos, y_pos);
    update_cell();
}

void Model2::charging()
{
    returning = false;
    while(battery.get_battery_level() < battery.get_max_battery())
    {
        battery.charge();
        std::cout << "Charging..." << std::endl;
        show_battery();
    }
    neighbors = laser.calc_collision(x_pos, y_pos);
    return cleaning_routine();
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

    // std::cout << "This option is not yet available!" << std::endl;
    // return modelRobot(name, x, y, btr_cp, p_r, p_rob);

    p_rob = new Model2(name, x, y, btr_cp, p_r);
    return;
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
        
        p_rob = new Model1(filename, p_r);
        return;
    }

    // std::cout << "This option is not yet available!" << std::endl;
    // return modelRobot(filename, p_r, p_rob);
    
    p_rob = new Model2(filename, p_r);
    return;
}