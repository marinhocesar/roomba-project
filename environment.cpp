#include <iostream>
#include <fstream>
#include <string.h>

//using namespace std;

int** createMatrix(int rows, int cols)
{
    int** matrix = new int*[rows];
    for(int i = 0; i < rows; i++){
        matrix[i] = new int[cols];
    }
    return matrix;
}

void fillMatrix(int** matrix, int rows, int cols){
    int counter = 1;
    for (int i = 0; i < rows; i++){
        for (int j = 0; j < cols; j++){
            matrix[i][j] = 0;
            counter++;
        }
    }
}

void printMatrix(int** matrix, int rows, int cols){
    for (int i = 0; i < rows; i++){
        std::cout << "| ";
        for (int j = 0; j < cols; j++){
            std::cout << matrix[i][j];
            if(j != cols-1){ std::cout << " "; }
        }
        std::cout << " |" << std::endl;
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
        int i = 0;
        for(int i = 0; i < 2; i++)
        {
            getline(f, file_info, ' ');
            temp[i] = stoi(file_info);
        }
        width = temp[0];
        height = temp[1];
        f.close();
    }
    
    grid = createMatrix(width, height);
    fillMatrix(grid, width, height);
}

int main()
{
    Environment grade;
    Environment grade_b (5,5);
    Environment grade_f ("./grid_dimensions.txt");

    printMatrix(grade.grid, grade.width, grade.height);
    std::cout << std::endl;
    printMatrix(grade_b.grid, grade_b.width, grade_b.height);
    std::cout << std::endl;
    printMatrix(grade_f.grid, grade_f.width, grade_f.height);
}
