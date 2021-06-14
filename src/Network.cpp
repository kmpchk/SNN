#include <iostream>
#include <chrono>

int main() {
    std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count() << std::endl;
    return 0;
}