#ifndef CAR_H
#define CAR_H

#include <utility>
#include <cmath>

#include "cone.h"
#include "definitions.h"

using namespace std;

class Car
{
public:
    Car();
    Car(const coord & newPos);
    ~Car() = default;
    void setPosition(const Pos & newPos);
    const Pos & getPosition() const;
    void setVelocity(const Vel & vel);
    const Vel & getVelocity() const;

    void updateCarKinematicModel(ControlInputs ci);
    void updateCarDynamicModel(ControlInputs ci, TireForces tf);
    void discretizeModelRungeKutta(Car & car, ControlInputs control, TireForces forces,
                                    double dt, Car (*vehicleModel)(Car &, ControlInputs, TireForces));

    Car operator*(double a);
    Car operator+(Car c);
    
private:
    TireForces tireModel(ControlInputs ci) const;

private:
    Pos position = {0, 0, 0};
    Vel velocity = {0, 0, 0};
};

#endif // CAR_H
