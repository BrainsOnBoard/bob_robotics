// BoB robotics includes
#include "common/progress_bar.h"

int bobMain(int, char **)
{
    constexpr size_t Count = 100;
    BoBRobotics::ProgressBar progbar{ Count };
    for (size_t i = 0; i < Count; i++) {
        progbar.increment();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}
