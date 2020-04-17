#ifndef CAR_H
#define CAR_H

#include <utility>
#include <cmath>
#include <memory>

#include "cone.h"
#include "definitions.h"
#include "visualisation.h"

using namespace std;

class Car
{
public:
    Car(shared_ptr<Visualisation> vis);
    Car(const Pos & newPos, const Vel & newVel);
    ~Car() = default;
    void setPosition(const Pos & newPos);
    const Pos & getPosition() const;
    void setVelocity(const Vel & vel);
    const Vel & getVelocity() const;

    Car & updateCarKinematicModel(ControlInputs control);
    Car & updateCarDynamicModel(ControlInputs control);
    void updateCar(ControlInputs control, double dt, Car & (*vehicleModel)(Car &, ControlInputs));

    Car & operator*(double a);
    Car & operator+(Car c);
    
private:
    TireForces tireModel(ControlInputs ci) const;

private:
    Pos position = {5, 1, 0};
    Vel velocity = {1, 0, 0};
    shared_ptr<Visualisation> visualisation;
};

Car & kinematicModel(Car & car, ControlInputs control);
Car & dynamicModel(Car & car, ControlInputs control);

#endif // CAR_H
