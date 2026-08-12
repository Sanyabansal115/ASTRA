#include "nav2_client/navigation_client.hpp"
#include <iostream>
#include <chrono>
#include <thread>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto client = std::make_shared<NavigationClient>();

    std::vector<std::string> rooms = {
        "command_module",
        "sleeping_quarters", 
        "airlock",
        "cargo_bay",
        "medical_bay",
        "engine_room"
    };

    while (rclcpp::ok()) {
        std::cout << "\n=== SPACECRAFT NAVIGATION MENU ===" << std::endl;
        std::cout << "1. Command Module" << std::endl;
        std::cout << "2. Sleeping Quarters" << std::endl;
        std::cout << "3. Airlock" << std::endl;
        std::cout << "4. Cargo Bay" << std::endl;
        std::cout << "5. Medical Bay" << std::endl;
        std::cout << "6. Engine Room" << std::endl;
        std::cout << "q. Quit" << std::endl;
        std::cout << "Enter choice: ";

        char choice;
        std::cin >> choice;

        if (choice == 'q' || choice == 'Q') {
            break;
        }

        int index = choice - '1';
        if (index >= 0 && index < 6) {
            client->navigateToRoom(rooms[index]);
        } else {
            std::cout << "Invalid choice. Please try again." << std::endl;
        }
    }

    rclcpp::shutdown();
    return 0;
}
