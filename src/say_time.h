#ifndef SAY_TIME_H
#define SAY_TIME_H

#include <chrono>
#include <iostream>

/*
 * @brief       Get the current time in milliseconds since epoch.
 */
inline long epoch_time() {
    using namespace std::chrono;
    long ms = duration_cast<milliseconds> (
            system_clock::now().time_since_epoch()
    ).count();
    return ms;
}


/*
 * @brief       Print the current time without line termination.
 */
inline void say_time() {
    long ms = epoch_time();
    std::cout << ms << " | ";
}

#endif // SAY_TIME_H