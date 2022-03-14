#include "menus.hpp"

// g++ .\environment.cpp .\robot.cpp .\main.cpp .\menus.cpp -c
// g++ .\environment.o .\robot.o .\main.o .\menus.o -o main.exe

int main()
{
    // Environment room;
    Environment room = Environment();
    Environment* p_room = &room;
    Robot* p_roomba; 
    
    menu_A(p_room, p_roomba);
    std::cout << room << std::endl;
    
    std::cout << "Your Roomba will start cleaning." << std::endl;     
    
    p_roomba->cleaning_routine();

    // p_roomba = new Model2("robot_info.txt", p_room);

    // p_roomba->a_star(p_roomba->get_environment(), p_roomba->get_x_pos(), p_roomba->get_y_pos());

    std::cout << "End of testing. Robot out of battery." << std::endl;
    return 0;
}

