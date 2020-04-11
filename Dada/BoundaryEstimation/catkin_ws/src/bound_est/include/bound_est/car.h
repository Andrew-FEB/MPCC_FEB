#ifndef CAR_H
#define CAR_H

#include <utility>

#include "cone.h"
#include "definitions.h"

constexpr double length = 24;
constexpr double width = 12;
constexpr carDims dims{ length, width };

class Car
{
public:
    Car(const double &x = 0, const double &y = 0);
    ~Car() = default;
    void updatePos(const double &x, const double &y);
    const carDims &getDims();
    double getX() const;
    double getY() const;

private:
    double x = 0;
    double y = 0;
};

#endif // CAR_H
