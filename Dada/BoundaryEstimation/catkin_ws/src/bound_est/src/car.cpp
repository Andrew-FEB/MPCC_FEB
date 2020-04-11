#include "car.h"

Car::Car() : position{0,0}
{
}

Car::Car(const coord &newPos) : position(newPos)
{

}

void Car::updatePos(const coord &newPos)
{
    position = newPos;
}

const carDims Car::getDimensions()
{
    return { length, width };
}

const coord &Car::getPos() const
{
    return position;
}

const double &Car::getVelocity() const
{
    return velocity;
}

bool Car::setVelocity(const double &vel)
{
    velocity = vel;
}

bool Car::setDirection(const double &ang)
{
    angle = ang;
}
