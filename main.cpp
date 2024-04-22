#include "menus.hpp"

int main()
{
    // Environment room;
    Environment room = Environment();
    Environment *p_room = &room;
    Robot *p_roomba;

    menu_A(p_room, p_roomba);
    std::cout << room << std::endl;

    std::cout << "Your Roomba will start cleaning." << std::endl;

    p_roomba->cleaning_routine();

    std::cout << "End of testing. Robot out of battery." << std::endl;
    return 0;
}
