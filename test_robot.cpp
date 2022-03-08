#include "robot.hpp"

int main()
{
    Environment room = Environment("environment_info.txt");
    Environment* p_a = &room;
    // Model1 bob = Model1("robo1", 0, 0, 100, p_a);
    Model1 bob = Model1("robot_info.txt", p_a);
    
    while (!bob.stop_robot())
    {
        bob.clean();
        bob.show_battery();
        std::cout << room << std::endl;
        
    }

    return 0;
}

