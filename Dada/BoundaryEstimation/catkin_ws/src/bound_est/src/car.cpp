#include "car.h"

Car::Car() : position{0,0} {}

Car::Car(const coord & newPos) : position({newPos, 0}) {}

/**
 *  Bicycle Models
 */
void Car::updateCarKinematicModel(ControlInputs control)
{
    // First simple (kinematic) model (source: https://github.com/MPC-Berkeley/barc/wiki/Car-Model):
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
    position = {xNext, yNext, atan2(yNext, xNext)};
    velocity = {vNext, velocity.vy, psiNext};
}

void Car::updateCarDynamicModel(ControlInputs control, TireForces forces)
{
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
    position = {xNext, yNext, phiNext};
    velocity = {vxNext, vyNext, omegaNext};
}

/**
 * Pacejka Tire Model
 */
TireForces Car::tireModel(ControlInputs control) const
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
 * Discretization of the model
 */
// void Car::discretizeModelRungeKutta(Car & car, ControlInputs control, TireForces forces,
//                                     double dt, Car & (*vehicleModel)(Car &, ControlInputs, TireForces))
// {
//     auto k1 = vehicleModel(car, control, forces) * dt;
//     auto k2 = vehicleModel(car + k1 * 0.5 * dt, control, forces) * dt;
//     auto k3 = vehicleModel(car + k2 * 0.5 * dt, control, forces) * dt;
//     auto k4 = vehicleModel(car + k3 * dt, control, forces) * dt;
//     car = car + (k1 + k2 * 2 + k3 * 2 + k4) * (1.0 / 6.0);
// }

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
    position.p.x = a * position.p.x;
    position.p.y = a * position.p.y;
    position.phi = a * position.phi;
    velocity.vx = a * velocity.vx;
    velocity.vy = a * velocity.vy;
    velocity.omega = a * velocity.omega;
}

Car Car::operator+(Car c)
{
    position.p.x = position.p.x + c.getPosition().p.x;
    position.p.y = position.p.y + c.getPosition().p.y;
    position.phi = position.phi + c.getPosition().phi;
    velocity.vx = velocity.vx + c.getVelocity().vx;
    velocity.vy = velocity.vy + c.getVelocity().vy;
    velocity.omega = velocity.omega + c.getVelocity().omega;
}