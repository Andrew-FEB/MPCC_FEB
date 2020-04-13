#include "car.h"

Car::Car() : position{0,0} {}

Car::Car(const coord &newPos) : position(newPos) {}

/**
 *  Bicycle Models
 */
void Car::updateCarKinematicModel(ControlInputs control)
{
    // First simple (kinematic) model (source: https://github.com/MPC-Berkeley/barc/wiki/Car-Model):
    // State parameters
    auto x = position.x;           // Longitudinal position
    auto y = position.y;           // Lateral Position
    auto psi = headingAngle;       // Yaw rate
    auto v = velocity.vx;          // Velocity
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
    position = {xNext, yNext};
    headingAngle = psiNext;
    velocity = {vNext, velocity.vy};
}

void Car::updateCarDynamicModel(ControlInputs control, TireForces forces)
{
    // State variables
    auto x = position.x;
    auto y = position.y;
    auto phi = headingAngle;
    auto vx = velocity.vx;
    auto vy = velocity.vy;
    auto omega = angularVelocity;

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
    position = {xNext, yNext};
    headingAngle = phiNext;
    velocity = {vxNext, vyNext};
    angularVelocity = omega;

}

/**
 * Pacejka Tire Model
 */
TireForces Car::tireModel(ControlInputs control) const
{
    // State variables
    auto vx = velocity.vx;
    auto vy = velocity.vy;
    auto omega = angularVelocity;

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

void Car::setPosition(const coord &newPos)
{
    this->position = newPos;
}

const coord &Car::getPosition() const
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

void Car::setHeadingAngle(const double &ang)
{
    this->headingAngle = ang;
}

const double &Car::getHeadingAngle() const
{
    return headingAngle;
}

void Car::setAngularVelocity(const double &av)
{
    this->angularVelocity = av;
}

const double Car::getAngularVelocity() const
{
    return angularVelocity;
}

const carDims Car::getDimensions()
{
    return {length, width};
}

Car Car::operator*(double a)
{
    position.x = a * position.x;
    position.y = a * position.y;
    headingAngle = a * headingAngle;
    velocity.vx = a * velocity.vx;
    velocity.vy = a * velocity.vy;
    angularVelocity = a * angularVelocity;
}

Car Car::operator+(Car c)
{
    position.x = position.x + c.getPosition().x;
    position.y = position.y + c.getPosition().y;
    headingAngle = headingAngle + c.getHeadingAngle();
    velocity.vx = velocity.vx + c.getVelocity().vx;
    velocity.vy = velocity.vy + c.getVelocity().vy;
    angularVelocity = angularVelocity + c.getAngularVelocity();
}