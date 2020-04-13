#ifndef CAR_H
#define CAR_H

#include <utility>
#include <cmath>

#include "cone.h"
#include "definitions.h"

constexpr double length = 24;   //JUNK NUMBER
constexpr double width = 12;

constexpr carDims dims{length, width};

using namespace std;

// Car should be a singleton TODO
class Car
{
public:
    Car();
    Car(const coord &newPos);
    ~Car() = default;
    void setPosition(const coord &newPos);
    const coord & getPosition() const;
    void setVelocity(const Vel & vel);
    const Vel & getVelocity() const;
    void setHeadingAngle(const double &ang);
    const double & getHeadingAngle() const;
    void setAngularVelocity(const double &av);
    const double getAngularVelocity() const;
    const carDims getDimensions();

    void updateCarKinematicModel(ControlInputs ci);
    void updateCarDynamicModel(ControlInputs ci, TireForces tf);

    Car operator*(double a);
    Car operator+(Car c);
    
private:
    TireForces tireModel(ControlInputs ci) const;

private:
    coord position = {0, 0};
    Vel velocity = {0, 0};
    double headingAngle = 0;
    double angularVelocity = 0;
};

#endif // CAR_H
