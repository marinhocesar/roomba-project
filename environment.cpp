#include <iostream>
#include <fstream>
#include <string.h>
#include <tuple>

//using namespace std;


int* getCoordinates(std::string s, std::string separator = " ")
{
    int * coord;
    coord = new int [2];
    int i = 0;
    int start = 0;
    int end = s.find(separator);
    while (i<2)
    {
        std::string coord_str;
        coord_str = s.substr(start, end - start);
        coord[i] = std::stoi(coord_str);
        start = end + separator.size();
        end = s.find(separator, start);
        i++;
    }
    
    return coord;
}

int** createMatrix(int rows, int cols)
{
    // Allocates memory for a matrix
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

Environment::Environment()
{
    width = 8;
    height = 8;
    grid = createMatrix(width, height);
    fillMatrix(grid, width, height);
}

Environment::Environment(int x, int y)
{
    width = x;
    height = y;
    grid = createMatrix(width, height);
    fillMatrix(grid, width, height);
}

Environment::Environment(std::string filename)
{
    std::string file_info;
    std::ifstream f(filename);
    if(f.is_open())
    {
        int temp[2];
        for(int i = 0; i < 2; i++)
        {
            getline(f, file_info, ' ');
            temp[i] = std::stoi(file_info);
        }
        width = temp[0];
        height = temp[1];
        f.close();
    }
    
    grid = createMatrix(width, height);
    fillMatrix(grid, width, height);
}

Environment::add_obstacle(int i, int j)
{
    grid[i][j] = 1;
}

Environment::add_obstacle(int x_start, int x_finish, int y_start, int y_finish)
{
    // Adds a rectangular obstacle to the environment matrix
    for(int i = x_start; i < x_finish; i++)
    {
        for(int j = y_start; j < y_finish; j++){
            grid[i][j] = 1;
        }
    }
}

Environment::add_obstacle(std::string filename)
{
    std::string file_info;
    std::ifstream f(filename);
    if(f.is_open())
    {
        while (getline(f, file_info, '\n'))
        {
            int* coord = getCoordinates(file_info);
            std::cout << coord[0] << " " << coord[1] << std::endl;
            add_obstacle(coord[0], coord[1]);
        }
        
        f.close();
    
    }
}

Environment::save_to_file(std::string filename)
{
    filename += ".txt";
    std::ofstream f(filename);
    if(f.is_open())
    {
        // ASK THE TEACHER IF THERE IS A WAY TO USE THE
        // OVERCHARGED << OPERATOR
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

std::ostream& operator << (std::ostream& os, const Environment& env)
{
    // Overcharging of the << operator
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

int main()
{
    Environment grade;
    Environment grade_b (5,5);
    Environment grade_f ("./grid_dimensions.txt");

    std::cout << grade << std::endl;
    //grade.add_obstacle(0, 0);
    //std::cout << grade << std::endl;
    //grade.add_obstacle(0,2,2,4);
    //std::cout << grade << std::endl;
    //grade.save_to_file("teste");
    //grade.add_obstacle("obstacles.txt");
    std::cout << grade << std::endl;
}
