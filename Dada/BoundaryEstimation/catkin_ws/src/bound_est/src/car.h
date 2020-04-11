#ifndef CAR_H
#define CAR_H

#include <utility>

#include "cone.h"
#include "definitions.h"

constexpr double length = 24;   //JUNK NUMBER
constexpr double width = 12;    
constexpr carDims dims{ length, width };

class Car
{
public:
    Car();
    Car(const coord &newPos);
    ~Car() = default;
    void updatePos(const coord &newPos);
    const carDims getDimensions();
    const coord &getPos() const;
    const double &getVelocity() const;
    const double &getAngle() const;
    bool setVelocity(const double &vel);
    bool setDirection(const double &ang);

private:
    coord position = {0, 0};
    double velocity = 0;
    double angle = 0;
};

#endif // CAR_H
