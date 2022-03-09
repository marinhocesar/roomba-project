#include "environment.hpp"


/* ============================Auxiliary Functions=========================== */

int *getCoordinates(std::string s, std::string separator = ",")
{
    // Function that recieves a string and subdivides into 2 integers using a
    // separator.
    int *coord = new int[2];     // Pointer as a way to return multiple values.
    int start = 0;               // Index of 's' string start.
    int end = s.find(separator); // Index of the first separator.

    std::string coord_str;
    coord_str = s.substr(start, end - start);
    coord[0] = std::stoi(coord_str); // First coordinate.
    // Updates 'start' and 'end' to correspond to the second number.
    start = end + separator.size();
    end = s.length();
    coord_str = s.substr(start, end - start);
    coord[1] = std::stoi(coord_str); // Second coordinate.

    return coord;
}

int **createMatrix(int rows, int cols)
{
    // Allocates memory for a matrix.
    int **matrix = new int *[cols];
    for (int i = 0; i < cols; i++)
    {
        matrix[i] = new int[rows];
    }
    return matrix;
}

void fillMatrix(int **matrix, int rows, int cols)
{
    // Fills the matrix with zeros.
    for (int i = 0; i < cols; i++)
    {
        for (int j = 0; j < rows; j++)
        {
            matrix[i][j] = 0;
        }
    }
}

/* ===========================Class Constructors============================ */

Environment::Environment()
{
    // Construtor for when the class is instantiated without arguments.
    grid = createMatrix(width, height);
    fillMatrix(grid, width, height);
    set_charging_station(charging_station_x, charging_station_y);
}

Environment::Environment(int x, int y)
{
    // Construtor for when the class is instantiated with user arguments.
    width = x;
    height = y;
    grid = createMatrix(width, height);
    fillMatrix(grid, width, height);
    set_charging_station(charging_station_x, charging_station_y);
}

Environment::Environment(int x, int y, int m, int n)
{
    // Construtor for when the class is instantiated with user arguments.
    width = x;
    height = y;
    grid = createMatrix(width, height);
    fillMatrix(grid, width, height);
    set_charging_station(m, n);
}

Environment::Environment(std::string filename)
{
    // Construtor for when the class is instantiated from a file.
    std::string file_info;
    std::ifstream f(filename); // Opens the file in input mode.
    if (f.is_open())
    {
        while (getline(f, file_info))
        {
            int start = file_info.find("=") + 1;
            int end = file_info.length();
            std::string file_info_number = file_info.substr(start, end - start);
            if (file_info.find("$") != std::string::npos)
            {
                break;
            }
            if (file_info.find("width") != std::string::npos)
            {
                width = std::stoi(file_info_number);

                continue;
            }
            if (file_info.find("height") != std::string::npos)
            {
                height = std::stoi(file_info_number);

                continue;
            }
            if (file_info.find("charging_station_x") != std::string::npos)
            {
                charging_station_x = std::stoi(file_info_number);

                continue;
            }
            if (file_info.find("charging_station_y") != std::string::npos)
            {
                charging_station_y = std::stoi(file_info_number);

                continue;
            }
        }

        f.close();
    }

    if (width < 1 || height < 1)
    {
        std::cout << "Invalid arguments found in the file." << std::endl;
        std::cout << "Initialization will occour with default values." << std::endl;

        width = 8;
        height = 8;
    }

    grid = createMatrix(width, height);
    fillMatrix(grid, width, height);
    set_charging_station(charging_station_x, charging_station_y);
    add_obstacle(filename);
}

/* ===========================Class Methods================================== */

void Environment::add_obstacle(int i, int j)
{
    // Adds an obstacle to the matrix representation of the Environment instance.
    if (i == charging_station_x && j == charging_station_y)
    {
        return;
    }
    if (i < 1 || i > width || j < 1 || j > height)
    {
        std::cout << "One or more arguments provided are invalid." << std::endl;
        return;
    }
    grid[j - 1][i - 1] = 1;
}

void Environment::add_obstacle(int x_start, int y_start, int x_finish, int y_finish)
{
    // Adds a rectangle to the matrix representation of the Environment instance.
    for (int j = y_start; j <= y_finish; j++)
    {
        for (int i = x_start; i <= x_finish; i++)
        {
            add_obstacle(i, j);
        }
    }
}

void Environment::add_obstacle(std::string filename)
{
    // Adds obstacles based on information that comes from a file.
    std::string file_info;
    std::ifstream f(filename); // Opens the file in input mode.
    if (f.is_open())
    {
        while (getline(f, file_info))
        {
            int start = file_info.find("=") + 1;
            int end = file_info.length();
            std::string file_info_number = file_info.substr(start, end - start);
            if (file_info.find("$") != std::string::npos)
            {
                break;
            }
            if (file_info.find("obstacle") != std::string::npos)
            {
                int *coord = getCoordinates(file_info_number);
                
                add_obstacle(coord[0], coord[1]);
            }
        }

        f.close();
    }
}

void Environment::set_charging_station(int i, int j)
{
    // Adds an charging station to the matrix representation of the Environment instance.
    if (i < 1 || i > width || j < 1 || j > height)
    {
        std::cout << "One or more arguments provided are invalid." << std::endl;
        return;
    }
    grid[charging_station_y - 1][charging_station_x - 1] = 0;
    charging_station_x = i;
    charging_station_y = j;
    grid[j - 1][i - 1] = 4;
}

void Environment::save_to_file(std::string filename)
{
    // Method to save the matrix that represents the environment.
    if (filename.find(".txt") == std::string::npos)
    {
        filename += ".txt"; // Adds the .txt file extension.
    }

    std::ofstream f(filename); // Opens the file in output mode.
    if (f.is_open())
    {
        f << "width= " << width << "\n";
        f << "height= " << height << "\n";
        f << "charging_station_x= " << charging_station_x << "\n";
        f << "charging_station_y= " << charging_station_y << "\n";

        for (int i = 0; i < width; i++)
        {
            for (int j = 0; j < height; j++)
            {

                // The if statement skips a pass of the loop if the current cell
                // isn't an obstacle.
                if (grid[j][i] != 1)
                {
                    continue;
                }
                else
                {
                    f << "obstacle= " << i + 1 << ", " << j + 1 << "\n";
                }
            }
        }

        f << "$\n";

        // ASK THE PROFESSOR IF THERE IS A WAY TO USE THE OVERLOADED << OPERATOR

        // f << grid << std::endl;

        // Writes the matrix that represents the environment into the file.
        for (int i = 0; i < height; i++)
        {
            f << "| ";
            for (int j = 0; j < width; j++)
            {
                f << grid[i][j];
                if (j != width - 1)
                {
                    f << " ";
                }
            }
            f << " |" << std::endl;
        }

        f.close();
    }
}

void Environment::set_element(int x, int y, int value)
{
    grid[y][x] = value;
}

int Environment::get_width()
{
    return width;
}
int Environment::get_height()
{
   return height; 
}
int Environment::get_charging_x()
{
    return charging_station_x;
}
int Environment::get_charging_y()
{
    return charging_station_y;
}
int** Environment::get_grid()
{
    return grid;
}

/* ===========================Operator Overload============================== */

std::ostream &operator<<(std::ostream &os, const Environment &env)
{
    // Overloading of the << operator
    os << "\n";
    for (int i = 0; i < env.height; i++)
    {
        os << "| ";
        for (int j = 0; j < env.width; j++)
        {
            os << env.grid[i][j];
            if (j != env.width - 1)
            {
                os << " ";
            }
        }
        os << " |" << std::endl;
    }
    return os;
}

//
void standardInitialization(Environment *p_room)
{
    *p_room = Environment();
}

void customInitialization(Environment *p_room)
{
    int width = 0, height = 0;
    std::cout << "\nCustom Initialization" << std::endl;
    std::cout << "The dimensions must be integers." << std::endl;
    std::cout << "Enter the width: ";
    std::cin >> width;
    std::cout << "Enter the height: ";
    std::cin >> height;

    if (width < 1 || height < 1)
    {
        std::cout << "Invalid arguments." << std::endl;
        std::cout << "Initialization will occour with default values." << std::endl;
        *p_room = Environment();
        return;
    }

    std::cout << "Do you want to set the location of the charging station?" << std::endl;
    std::cout << "1- Yes" << std::endl;
    std::cout << "2- No" << std::endl;

    int answer = 0;
    std::cin >> answer;

    if (answer == 1)
    {
        int x_charger = 0, y_charger = 0;
        std::cout << "Value must be between 1 and " << width << std::endl;
        std::cout << "X coordinate of the charging station: ";
        std::cin >> x_charger;
        std::cout << "Value must be between 1 and " << height << std::endl;
        std::cout << "Y coordinate of the charging station: ";
        std::cin >> y_charger;
        *p_room = Environment(width, height, x_charger, y_charger);
        return;    
    }


    std::cout << "Initialization will occour with default values." << std::endl;
    *p_room = Environment(width, height);
}

void fileInitialization(Environment *p_room)
{
    std::string filename = "";
    std::cout << "\nFile Initialization" << std::endl;
    std::cout << "Enter filename: ";
    std::cin.ignore();
    std::getline(std::cin, filename);
    if (filename == "")
    {
        filename = "environment_info.txt";
    }
    *p_room = Environment(filename);
}

void saveMenu(Environment *room)
{
    std::string filename = "";
    std::cout << "Choose a filename: ";
    std::cin.ignore();
    std::getline(std::cin, filename);
    if (filename == "")
    {
        filename = "environment_info";
    }
    (*room).save_to_file(filename);
    std::cout << "\nFile saved as " << filename << ".txt" << std::endl;
}
void obstaclesMenu(Environment *room)
{
    while (true)
    {
        std::cout << "\nAdd obstacles (0 to return)" << std::endl;
        std::cout << "1- Individual obstacles." << std::endl;
        std::cout << "2- Rectangle of obstacles." << std::endl;
        std::cout << "3- Obstacles from a file." << std::endl;
        int answer = -1;
        std::cin >> answer;
        if (answer == 0)
        {
            break;
        }
        if (answer != 1 && answer != 2 && answer != 3)
        {
            std::cout << "Sorry, this is not an option." << std::endl;
            continue;
        }

        if (answer == 1)
        {
            int x = -1, y = -1;
            std::cout << "Value must be between 1 and " << room->get_width() << std::endl;
            std::cout << "Enter the X coordinate: ";
            std::cin >> x;
            std::cout << "Value must be between 1 and " << room->get_height() << std::endl;
            std::cout << "Enter the Y coordinate: ";
            std::cin >> y;

            if (x < 1 || y < 1)
            {
                std::cout << "Invalid arguments." << std::endl;
                return;
            }

            (*room).add_obstacle(x, y);
            std::cout << (*room) << std::endl;
        }
        else if (answer == 2)
        {
            int x1 = -1, y1 = -1, x2 = -1, y2 = -1;
            std::cout << "Value must be between 1 and " << room->get_width() << std::endl;
            std::cout << "Enter the X coordinate of the top-left corner: ";
            std::cin >> x1;
            std::cout << "Value must be between 1 and " << room->get_height() << std::endl;
            std::cout << "Enter the Y coordinate of the top-left corner: ";
            std::cin >> y1;
            std::cout << "Value must be between 1 and " << room->get_width() << std::endl;
            std::cout << "Enter the X coordinate of the bottom-right corner: ";
            std::cin >> x2;
            std::cout << "Value must be between 1 and " << room->get_height() << std::endl;
            std::cout << "Enter the Y coordinate of the bottom-right corner: ";
            std::cin >> y2;

            if (x1 < 0 || y1 < 0 || x2 < 0 || y2 < 0 || x2 < x1 || y2 < y1)
            {
                std::cout << "Invalid arguments." << std::endl;
                return;
            }
            (*room).add_obstacle(x1, y1, x2, y2);
            std::cout << (*room) << std::endl;
        }
        else
        {
            std::string filename = "";
            std::cout << "Enter the filename: ";
            std::cin.ignore();
            std::getline(std::cin, filename);
            std::cout << filename << std::endl;
            if (filename == "")
            {
                filename = "environment_info.txt";
            }
            (*room).add_obstacle(filename);
            std::cout << (*room) << std::endl;
        }
    }
}