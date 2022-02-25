#include <iostream>
#include <fstream>
#include <string.h>

/* ==============================Class======================================= */

class Environment
{
public:
    int width, height;
    int** grid;
    Environment();
    Environment(int, int);
    Environment(std::string);
    add_obstacle(int, int);
    add_obstacle(int, int, int, int);
    add_obstacle(std::string);
    save_to_file(std::string);
    friend std::ostream& operator<<(std::ostream&, const Environment&);
};

/* ===========================Class Constructors============================ */

Environment::Environment()
{
    // Construtor for when the class is instantiated without arguments.
    width = 8;
    height = 8;
    grid = createMatrix(width, height);
    fillMatrix(grid, width, height);
}

Environment::Environment(int x, int y)
{
    // Construtor for when the class is instantiated with user arguments.
    width = x;
    height = y;
    grid = createMatrix(width, height);
    fillMatrix(grid, width, height);
}

Environment::Environment(std::string filename)
{
    // Construtor for when the class is instantiated from a file.
    std::string file_info;
    std::ifstream f(filename); // Opens the file in input mode.
    if(f.is_open())
    {
        int temp[2];
        for(int i = 0; i < 2; i++)
        {
            getline(f, file_info, ' '); // Gets strings separated by whitespace.
            temp[i] = std::stoi(file_info); // Transforms strings into integers.
        }
        width = temp[0];
        height = temp[1];
        f.close();
    }
    
    grid = createMatrix(width, height);
    fillMatrix(grid, width, height);
}

/* ===========================Class Methods================================== */

Environment::add_obstacle(int i, int j)
{
    // Adds an obstacle to the matrix representation of the Environment instance.
    grid[i][j] = 1;
}

Environment::add_obstacle(int x_start, int y_start, int x_finish, int y_finish)
{
    // Adds a rectangle to the matrix representation of the Environment instance.
    for(int i = x_start; i <= x_finish; i++)
    {
        for(int j = y_start; j <= y_finish; j++){
            grid[i][j] = 1;
        }
    }
}

Environment::add_obstacle(std::string filename)
{
    // Adds obstacles based on information that comes from a file.
    //
    // Looks for information inside of a .txt file in the following format:
    // x_1 y_1
    // x_2 x_2
    // .
    // .
    // .
    // x_n y_n
    // $
    // Where x_i and y_i are the coordinates of where the object is going to be
    // added. The coordinates go from 0 to the width/height
    std::string file_info;
    std::ifstream f(filename); // Opens the file in input mode.
    if(f.is_open())
    {
        while (getline(f, file_info, '\n'))
        {
            // The '$' is used to stop looking for coordinates, allowing for
            // extra information to be added to the same file without interfering
            // with the way the method looks for coordinates.

            if(file_info.find("$") != std::string::npos){ break; }
            int* coord = getCoordinates(file_info); // Call of auxiliary function.
            add_obstacle(coord[0], coord[1]);
        }
        
        f.close();
    
    }
}

Environment::save_to_file(std::string filename)
{
    // Method to save the matrix that represents the environment.
    filename += ".txt"; // Adds the .txt file extension.
    std::ofstream f(filename); // Opens the file in output mode.
    if(f.is_open())
    {

        for (int i = 0; i < width; i++){
            for (int j = 0; j < height; j++){

                // The if statement skips a pass of the loop if the current cell
                // isn't an obstacle.
                if(grid[i][j] != 1){ continue; }
                f << i << " " << j << "\n";
            }
        }

        f << "$\n";

        // ASK THE PROFESSOR IF THERE IS A WAY TO USE THE OVERLOADED << OPERATOR
        
        // Writes the matrix that represents the environment into the file.
        for (int i = 0; i < width; i++){
        f << "| ";
            for (int j = 0; j < height; j++){
                f << grid[i][j];
                if(j != height-1){ f << " "; }
            }
            f << " |" << std::endl;
        }
        f.close();
    }
}

/* ===========================Operator Overload============================== */

std::ostream& operator << (std::ostream& os, const Environment& env)
{
    // Overloading of the << operator
    for (int i = 0; i < env.width; i++){
        os << "| ";
        for (int j = 0; j < env.height; j++){
            os << env.grid[i][j];
            if(j != env.height-1){ os << " "; }
        }
        os << " |" << std::endl;
    }
    return os;
}

Environment customInitialization()
{
    int width = 0, height = 0;
    std::cout << "\nCustom Initialization" << std::endl;
    std::cout << "The dimensions must be integers." << std::endl;
    std::cout << "Enter the width: ";
    std::cin >> width;
    std::cout << "Enter the height: ";
    std::cin >> height;

    if(width < 1 || height < 1){
        std::cout << "Invalid arguments." << std::endl;
        std::cout << "Initialization will occour with default values." << std::endl;
        Environment room = Environment();
        return room;
    }

    Environment room(width, height);
    return room;
}

Environment fileInitialization()
{
    std::string filename = "" ;
    std::cout << "\nFile Initialization" << std::endl;
    std::cout << "Enter filename: ";
    std::cin.ignore();
    std::getline(std::cin, filename);
    if(filename == ""){ filename = "grid_dimensions.txt"; }
    Environment room = Environment(filename);

    return room;
}

/* ============================Auxiliary Functions=========================== */

int* getCoordinates(std::string s, std::string separator = " ")
{
    // Function that recieves a string and subdivides into 2 integers using a
    // separator.
    int * coord = new int [2]; // Pointer as a way to return multiple values.
    int start = 0; // Index of 's' string start.
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

int** createMatrix(int rows, int cols)
{
    // Allocates memory for a matrix.
    int** matrix = new int*[rows];
    for(int i = 0; i < rows; i++){
        matrix[i] = new int[cols];
    }
    return matrix;
}

void fillMatrix(int** matrix, int rows, int cols)
{
    // Fills the matrix with zeros.
    int counter = 1;
    for (int i = 0; i < rows; i++){
        for (int j = 0; j < cols; j++){
            matrix[i][j] = 0;
            counter++;
        }
    }
}

void saveMenu(Environment* room)
{
    std::string filename = "" ;
    std::cout << "Choose a filename: ";
    std::cin.ignore();
    std::getline(std::cin, filename);
    if(filename == ""){ filename = "saved_state"; }
    (*room).save_to_file(filename);
    std::cout << "File saved as " << filename << ".txt" << std::endl;
}
void obstaclesMenu(Environment* room)
{
    while(true){
        std::cout << "\nAdd obstacles (0 to return)" << std::endl;
        std::cout << "1- Individual obstacles." << std::endl;
        std::cout << "2- Rectangle of obstacles." << std::endl;
        std::cout << "3- Obstacles from a file." << std::endl;
        int answer = -1;
        std::cin >> answer;
        if(answer == 0){
            break; }
        if(answer != 1 && answer != 2 && answer != 3){ 
            std::cout << "Sorry, this is not an option." << std::endl;
            continue;
        }

        if(answer == 1)
        {
            int x = -1, y = -1;
            std::cout << "Enter the X coordinate: ";
            std::cin >> x;
            std::cout << "Enter the Y coordinate: ";
            std::cin >> y;

            if(x < 0 || y < 0){
                std::cout << "Invalid arguments." << std::endl;
                return;
            }

            (*room).add_obstacle(x, y);
            std::cout << (*room) << std::endl;

        }else if(answer == 2)
        {
            int x1 = -1, y1 = -1, x2 = -1, y2 = -1;
            std::cout << "Enter the X coordinate of the top-left corner: ";
            std::cin >> x1;
            std::cout << "Enter the Y coordinate of the top-left corner: ";
            std::cin >> y1;
            std::cout << "Enter the X coordinate of the bottom-right corner: ";
            std::cin >> x2;
            std::cout << "Enter the Y coordinate f the bottom-right corner: ";
            std::cin >> y2;

            if(x1 < 0 || y1 < 0 || x2 < 0 || y2 < 0 || x2 < x1 || y2 < y1){
                std::cout << "Invalid arguments." << std::endl;
                return;
            }
            (*room).add_obstacle(x1, y1, x2, y2);
            std::cout << (*room) << std::endl;
        }else
        {
            std::string filename = "" ;
            std::cout << "Enter the filename: ";
            std::cin.ignore();
            std::getline(std::cin, filename);
            if(filename == ""){ filename = "obstacles.txt"; }
            (*room).add_obstacle(filename);
            std::cout << (*room) << std::endl;
        }
    }
}

int main()
{
    while(true){
        std::cout << "\nEnvironment creation (0 to quit)" << std::endl; 
        std::cout << "1- Initialize basic environment (8x8 grid)." << std::endl; 
        std::cout << "2- Custom environment size." << std::endl; 
        std::cout << "3- Import environment from file." << std::endl;
        int answer = -1;
        std::cin >> answer;
        if(answer == 0){ break; }
        if(answer != 1 && answer != 2 && answer != 3){ 
            std::cout << "Sorry, this is not an option." << std::endl;
            continue;
        }
        
        Environment room;

        if(answer == 1)
        {
            Environment room = Environment();
        }else if(answer == 2)
        {
            room = customInitialization();
        }else
        {
            room = fileInitialization();
        }
        
        std::cout << room << std::endl;
        
        while(true)
        {
            std::cout << "\nOptions (0 to exit options menu)" << std::endl;
            std::cout << "1- Add obstacles." << std::endl;
            std::cout << "2- Save environment in its current state." << std::endl;
            std::cout << "3- Show environment." << std::endl;
            
            int menu = -1;
            std::cin >> menu;
            if(menu == 0){ break; }
            if(menu != 1 && menu != 2 && menu != 3){ 
                std::cout << "Sorry, this is not an option." << std::endl;
                continue;
            }

            Environment* p_room = &room;

            if(menu == 1)
            {
                obstaclesMenu(p_room);
            }else if(menu == 2)
            {
                saveMenu(p_room);
            }else
            {
                std::cout << room << std::endl;
            }
        }


    }

    return 0;
}



