#ifndef CAR_H
#define CAR_H

#include <utility>
#include <cmath>
#include <memory>
#include <iostream>

#include "cone.h"
#include "definitions.h"
#include "visualisation.h"

class Car
{
public:
    Car();
    Car(std::shared_ptr<Visualisation> vis);
    Car(const Pos & newPos, const Vel & newVel);
    Car(const Pos & newPos, const Vel & newVel, std::shared_ptr<Visualisation> vis);
    ~Car() = default;
    void setPosition(const Pos & newPos);
    const Pos & getPosition() const;
    void setVelocity(const Vel & vel);
    const Vel & getVelocity() const;

    void updateCar(ControlInputs control, double dt);

    Car operator*(double a);
    Car operator+(Car c);
    friend std::ostream & operator<<(std::ostream & os, const Car & car);

private:
    Car kinematicModel(const ControlInputs & control) const;
    Car dynamicModel(const ControlInputs & control) const;
    TireForces tireModel(const ControlInputs & control) const;

private:
    Pos position = {0, 0, 0};
    Vel velocity = {0, 0, 0};
    std::shared_ptr<Visualisation> visualisation = nullptr;
};

#endif // CAR_H
