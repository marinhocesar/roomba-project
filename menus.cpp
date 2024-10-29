#include "menus.hpp"

void start_menu(Environment *p_r, Robot *&p_rob)
{
    std::cout << "\nEnvironment (0 to quit)" << std::endl;
    std::cout << "1- Create new environment." << std::endl;
    std::cout << "2- Load environment from file." << std::endl;
    int answer = -1;
    std::cin >> answer;
    if (answer == 0)
    {
        exit(0);
    }
    if (answer != 1 && answer != 2)
    {
        std::cout << "Sorry, this is not an option." << std::endl;
        return menu_A(p_r, p_rob);
    }

    if (answer == 1)
    {
        return environment_creation_menu(p_r, p_rob);
    }

    fileInitialization(p_r);
    return environment_setup_menu(p_r, p_rob);
}

void environment_creation_menu(Environment *p_r, Robot *&p_rob)
{
    std::cout << "\nEnvironment Creation (0 to go back)" << std::endl;
    std::cout << "1- Create standard 8x8 environment." << std::endl;
    std::cout << "2- Create environment with custom dimensions." << std::endl;
    int answer = -1;
    std::cin >> answer;
    if (answer == 0)
    {
        return start_menu(p_r, p_rob);
    }
    if (answer != 1 && answer != 2)
    {
        std::cout << "Sorry, this is not an option." << std::endl;
        return environment_creation_menu(p_r, p_rob);
    }

    if (answer == 1)
    {
        standardInitialization(p_r);
        return environment_setup_menu(p_r, p_rob);
    }

    customInitialization(p_r);
    return environment_setup_menu(p_r, p_rob);
}

void environment_setup_menu(Environment *p_r, Robot *&p_rob)
{
    std::cout << "Your Environment:" << std::endl;
    std::cout << *p_r << std::endl;

    std::cout << "\nEnvironment Management (0 to quit)" << std::endl;
    std::cout << "1- Add/Remove obstacles." << std::endl;
    std::cout << "2- Save Environment to a file." << std::endl;
    std::cout << "3- Show Environment." << std::endl;
    std::cout << "4- Set/Move charging station." << std::endl;
    std::cout << "5- Add robot" << std::endl;
    int answer = -1;
    std::cin >> answer;
    if (answer == 0)
    {
        exit(0);
    }
    if (answer != 1 && answer != 2 && answer != 3 && answer != 4 && answer != 5)
    {
        std::cout << "Sorry, this is not an option." << std::endl;
        return environment_setup_menu(p_r, p_rob);
    }

    if (answer == 1)
    {
        return obstacles_menu(p_r, p_rob);
    }
    if (answer == 2)
    {
        saveMenu(p_r);
        return environment_setup_menu(p_r, p_rob);
    }
    if (answer == 3)
    {
        return environment_setup_menu(p_r, p_rob);
    }
    if (answer == 4)
    {
        moveChargingStation(p_r);
        return environment_setup_menu(p_r, p_rob);
    }

    return robot_creation_menu(p_r, p_rob);
}

void obstacles_menu(Environment *p_r, Robot *&p_rob)
{
    std::cout << "\nObstacle Menu (0 to go back)" << std::endl;
    std::cout << "1- Individual obstacles." << std::endl;
    std::cout << "2- Rectangle of obstacles." << std::endl;
    std::cout << "3- Obstacles from a file." << std::endl;
    std::cout << "4- Remove obstacle." << std::endl;
    int answer = -1;
    std::cin >> answer;
    if (answer == 0)
    {
        return environment_setup_menu(p_r, p_rob);
    }
    if (answer != 1 && answer != 2 && answer != 3 && answer != 4)
    {
        std::cout << "Sorry, this is not an option." << std::endl;
        return obstacles_menu(p_r, p_rob);
    }

    if (answer == 1)
    {
        addSingularObstacle(p_r);
        return obstacles_menu(p_r, p_rob);
    }
    else if (answer == 2)
    {
        addRectOfObstacles(p_r);
        return obstacles_menu(p_r, p_rob);
    }
    else if (answer == 3)
    {
        addObstaclesFromFile(p_r);
        return obstacles_menu(p_r, p_rob);
        
    }

    removeObstacles(p_r);
    return obstacles_menu(p_r, p_rob);
}

void robot_creation_menu(Environment *p_r, Robot *&p_rob)
{
    std::cout << "\nRobot Creation (0 to go back)" << std::endl;
    std::cout << "1- Create robot with custom parameters." << std::endl;
    std::cout << "2- Create robot from a file." << std::endl;
    int answer = -1;
    std::cin >> answer;
    if (answer == 0)
    {
        return environment_setup_menu(p_r, p_rob);
    }
    if (answer != 1 && answer != 2)
    {
        std::cout << "Sorry, this is not an option." << std::endl;
        return robot_creation_menu(p_r, p_rob);
    }

    if (answer == 1)
    {
        customRobot(p_r, p_rob);
        return;
    }

    fileRobot(p_r, p_rob);
    return;
}

void menu_cleaning()
{
    std::cout << "\nThe roomba is fully charged!" << std::endl;
    std::cout << "Would you like to continue cleaning?" << std::endl;
    std::cout << "1- Yes." << std::endl;
    std::cout << "2- No." << std::endl;
    int answer = -1;
    std::cin >> answer;

    if (answer != 1 && answer != 2)
    {
        std::cout << "Sorry, this is not an option." << std::endl;
        return menu_cleaning();
    }

    if (answer == 1)
    {
        return;
    }

    exit(0);
}
