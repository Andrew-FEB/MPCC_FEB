#ifndef CAR_H
#define CAR_H

#include <utility>
#include <cmath>
#include <memory>
#include <iostream>

#include "cone.h"
#include "definitions.h"
#include "visualisation.h"

using namespace std;

class Car
{
public:
    Car();
    Car(shared_ptr<Visualisation> vis);
    Car(const Pos & newPos, const Vel & newVel);
    Car(const Pos & newPos, const Vel & newVel, shared_ptr<Visualisation> vis);
    ~Car() = default;
    void setPosition(const Pos & newPos);
    const Pos & getPosition() const;
    void setVelocity(const Vel & vel);
    const Vel & getVelocity() const;

    Car updateCarKinematicModel(const ControlInputs & control) const;
    Car updateCarDynamicModel(const ControlInputs & control) const;
    void updateCar(ControlInputs control, double dt, Car (*vehicleModel)(const Car &, const ControlInputs &));

    Car operator*(double a);
    Car operator+(Car c);
    friend ostream & operator<<(ostream & os, const Car & car);

private:
    TireForces tireModel(const ControlInputs & control) const;

private:
    Pos position = {0, 0, 0};
    Vel velocity = {1.5, 0, 0};
    shared_ptr<Visualisation> visualisation = nullptr;
};

Car kinematicModel(const Car & car, const ControlInputs & control);
Car dynamicModel(const Car & car, const ControlInputs & control);

#endif // CAR_H
