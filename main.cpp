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
    
    while (!p_roomba->stop_robot())
    {        
        p_roomba->clean();
        std::cout << room << std::endl;
        
    }

    std::cout << "End of testing. Robot out of battery." << std::endl;
    return 0;
}

