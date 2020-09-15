#include "../../robots/rc_car_bot.h"

// third party includes
#include "../../third_party/units.h"

using namespace units::literals;

class PM_Control
{
    using degree_t = units::angle::degree_t;

    BoBRobotics::Robots::RCCarBot car;

public:
    void updateMotors(int angleOfBestRotation)
    {
        int normalisedAngle = normaliseAngle(angleOfBestRotation);
        //int normalisedAngle = angleOfBestRotation;
        //   int angleToTurn = 0;
        if (normalisedAngle <= -40) {
            normalisedAngle = -40;
        } else if (normalisedAngle >= 40) {
            normalisedAngle = 40;
        }

        car.move(0.18, degree_t((double) normalisedAngle));
        std::cout << angleOfBestRotation << " angle =" << normalisedAngle << std::endl;
    }

    int normaliseAngle(int angle)
    {
        angle = (angle + 180) % 360;
        if (angle < 0)
            angle += 360;
        return angle - 180;
    }

private:
};
