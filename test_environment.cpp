#include "environment.hpp"

int main()
{
    while (true)
    {
        std::cout << "\nEnvironment creation (0 to quit)" << std::endl;
        std::cout << "1- Initialize basic environment (8x8 grid)." << std::endl;
        std::cout << "2- Custom environment size." << std::endl;
        std::cout << "3- Import environment from file." << std::endl;
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

        Environment room;
        Environment *p_room = &room;
        if (answer == 1)
        {
            Environment room = Environment();
        }
        else if (answer == 2)
        {
            customInitialization(p_room);
        }
        else
        {
            fileInitialization(p_room);
            p_room->set_charging_station(p_room->charging_station_x, p_room->charging_station_x);
            std::string filename = "environment_info.txt";
            p_room->add_obstacle(filename);
        }

        std::cout << room << std::endl;

        while (true)
        {
            std::cout << "\nOptions (0 to exit options menu)" << std::endl;
            std::cout << "1- Add obstacles." << std::endl;
            std::cout << "2- Save environment in its current state." << std::endl;
            std::cout << "3- Show environment." << std::endl;
            std::cout << "4- Move charging station." << std::endl;

            int menu = -1;
            std::cin >> menu;
            if (menu == 0)
            {
                break;
            }
            if (menu != 1 && menu != 2 && menu != 3 && menu != 4)
            {
                std::cout << "Sorry, this is not an option." << std::endl;
                continue;
            }

            Environment *p_room = &room;

            if (menu == 1)
            {
                obstaclesMenu(p_room);
            }
            else if (menu == 2)
            {
                saveMenu(p_room);
            }
            else if (menu == 3)
            {
                std::cout << room << std::endl;
            }
            else
            {
                int x_charger = 0, y_charger = 0;
                std::cout << "Value must be between 1 and " << p_room->width << std::endl;
                std::cout << "X coordinate for the charging station: ";
                std::cin >> x_charger;
                std::cout << "Value must be between 1 and " << p_room->height << std::endl;
                std::cout << "New Y coordinate for the charging station: ";
                std::cin >> y_charger;
                p_room->set_charging_station(x_charger, y_charger);
                std::cout << room << std::endl;
            }
        }
    }

    return 0;
}
