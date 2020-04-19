#include "car.h"

Car::Car() {}
Car::Car(shared_ptr<Visualisation> vis) : visualisation(vis) {}
Car::Car(const Pos & newPos, const Vel & newVel) : position(newPos), velocity(newVel) {}
Car::Car(const Pos & newPos, const Vel & newVel, shared_ptr<Visualisation> vis) : position(newPos), velocity(newVel), visualisation(vis) {}

/**
 *  Bicycle Models
 */

Car Car::updateCarKinematicModel(const ControlInputs & control) const
{
    // State parameters
    auto x = position.p.x;           // Longitudinal position
    auto y = position.p.y;           // Lateral Position
    auto psi = velocity.omega;       // Yaw rate
    auto v = velocity.vx;            // Velocity
    // Control inputs
    auto a = control.D;                 // Acceleration
    auto delta = control.delta;         // Front steering angle

    // Compute slip angle
    auto beta = atan2(CarParams.lr * tan(delta), (CarParams.lf + CarParams.lr));

    // Compute next state
    auto xNext = v * cos(psi + beta);
    auto yNext = v * sin(psi + beta);
    auto psiNext = v / CarParams.lr * sin(beta);
    auto vNext = a;

    // Set next state
    return Car({xNext, yNext, position.phi}, {vNext, velocity.vy, psiNext});
}

Car Car::updateCarDynamicModel(const ControlInputs & control) const
{
    auto forces = tireModel(control);
    // State variables
    auto x = position.p.x;
    auto y = position.p.x;
    auto phi = position.phi;
    auto vx = velocity.vx;
    auto vy = velocity.vy;
    auto omega = velocity.omega;

    // Control variables
    auto delta = control.delta;

    // Tire forces
    auto Ffy = forces.Ffy;
    auto Frx = forces.Frx;
    auto Fry = forces.Fry;

    // Next state
    auto xNext = vx * cos(phi) - vy * sin(phi);
    auto yNext = vx * sin(phi) + vy * cos(phi);
    auto phiNext = omega;
    auto vxNext = 1 / CarParams.m * (Frx - Ffy * sin(delta) + CarParams.m * vy * omega);
    auto vyNext = 1 / CarParams.m * (Fry + Ffy * cos(delta) - CarParams.m * vx * omega);
    auto omegaNext = 1 / CarParams.Iz * (Ffy * CarParams.lf * cos(delta) - Fry * CarParams.lr);

    // Set next state
    return Car({xNext, yNext, phiNext}, {vxNext, vyNext, omegaNext});
}

/**
 * Pacejka Tire Model
 */
TireForces Car::tireModel(const ControlInputs & control) const
{
    // State variables
    auto vx = velocity.vx;
    auto vy = velocity.vy;
    auto omega = velocity.omega;

    // Control variables
    auto D = control.D;
    auto delta = control.delta;

    // Force calculations

    auto alphaF = - atan2((CarParams.lf * omega + vy), vx) + delta;
    auto alphaR = atan2((CarParams.lr * omega - vy), vx);

    auto Ffy = CarParams.Df * sin(CarParams.Cf * atan(CarParams.Bf * alphaF));
    auto Fry = CarParams.Dr * sin(CarParams.Cr * atan(CarParams.Br * alphaR));

    auto Frx = (CarParams.Cm1 - CarParams.Cm2 * vx) * D - CarParams.Crr - CarParams.Cd * pow(vx, 2);

    // TODO Tire constraints (i.e. the friction ellipse)
}

/**
 * Discretization of the model (4th order Runge-Kutta method)
 */
void Car::updateCar(ControlInputs control, double dt, Car (*vehicleModel)(const Car &, const ControlInputs &))
{
    Car & car{*this};
    cout << "BEFORE UPDATE:\n"  << car << endl;

    auto k1 = vehicleModel(car, control) * dt;
    // cout << "K1 = \n" << k1 << endl;
    // cout << "CAR:\n"  << car << endl;

    auto k2 = vehicleModel(car + k1 * 0.5, control) * dt;
    // cout << "K2 = \n" << k2 << endl;
    // cout << "CAR:\n"  << car << endl;

    auto k3 = vehicleModel(car + k2 * 0.5, control) * dt;
    // cout << "K3 = \n" << k3 << endl;
    // cout << "CAR:\n"  << car << endl;

    auto k4 = vehicleModel(car + k3, control) * dt;
    // cout << "K4 = \n" << k4 << endl;
    // cout << "CAR:\n"  << car << endl;

    car = car + (k1 + k2 * 2 + k3 * 2 + k4) * (1.0 / 6.0);
    // cout << "AFTER UPDATE:\n" << car << endl;

    #ifdef VISUALISE
    visualisation->showCar(position.p);
    #endif
}

void Car::setPosition(const Pos & newPos)
{
    this->position = newPos;
}

const Pos & Car::getPosition() const
{
    return position;
}

void Car::setVelocity(const Vel & vel)
{
    this->velocity = vel;
}

const Vel & Car::getVelocity() const
{
    return velocity;
}

Car Car::operator*(double a)
{
    // position.p.x = a * position.p.x;
    // position.p.y = a * position.p.y;
    // position.phi = a * position.phi;
    // velocity.vx = a * velocity.vx;
    // velocity.vy = a * velocity.vy;
    // velocity.omega = a * velocity.omega;

    // return *this;
    return Car({a * position.p.x, a * position.p.y, a * position.phi},
                {a * velocity.vx, a * velocity.vy, a * velocity.omega},
                visualisation);
}

Car Car::operator+(Car c)
{
    // position.p.x = position.p.x + c.getPosition().p.x;
    // position.p.y = position.p.y + c.getPosition().p.y;
    // position.phi = position.phi + c.getPosition().phi;
    // velocity.vx = velocity.vx + c.getVelocity().vx;
    // velocity.vy = velocity.vy + c.getVelocity().vy;
    // velocity.omega = velocity.omega + c.getVelocity().omega;

    // return *this;
    return Car({position.p.x + c.getPosition().p.x, position.p.y + c.getPosition().p.y, position.phi + c.getPosition().phi},
                {velocity.vx + c.getVelocity().vx, velocity.vy + c.getVelocity().vy, velocity.omega + c.getVelocity().omega},
                visualisation);
}

ostream & operator<<(ostream & os, const Car & car)
{
    os << "Position = {" << car.position.p.x << ", " << car.position.p.y << ", " << car.position.phi
            << "}\nVelocity = {" << car.velocity.vx << ", " << car.velocity.vy << ", " << car.velocity.omega << "}\n";
    return os;
}

/**
 * Non-member functions
 * These forward the member carModel functions so they can be used as the 
 * function handle parameter to the updateCar function.
 */

Car kinematicModel(const Car & car, const ControlInputs & control)
{
    return car.updateCarKinematicModel(control);
}

Car dynamicModel(const Car & car, const ControlInputs & control)
{
    return car.updateCarDynamicModel(control);
}