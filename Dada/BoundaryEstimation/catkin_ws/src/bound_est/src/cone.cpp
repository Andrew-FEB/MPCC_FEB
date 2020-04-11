#include "cone.h"

static int ID_VAL = 0;
static constexpr double radius = 0.5;

Cone::Cone(const double &newX, const double &newY, const BoundPos &newPos): x(newX), y(newY), pos(newPos), id(ID_VAL++)
{
}

const double &Cone::getX() 
{
    return x;
}

const double &Cone::getY() 
{
    return y;
}

const int &Cone::getID() 
{
	return id;
}

void Cone::setX(double value)
{
    x = value;
}

void Cone::setY(double value)
{
    y = value;
}

void Cone::setPos(const BoundPos& newPos)
{
	pos = newPos;
}

std::ostream &operator<<(std::ostream &os, const Cone &cone)
{
	//BoundStrTranslate defined in definitions.h. Used as map to translate enum class to string.
    os << "Cone id="<<cone.id<<", x=" << cone.x << ", y=" << cone.y << " and calculated boundary = "<<BoundStrTranslate.find(cone.pos)->second<<std::endl;
   return os;
}

const BoundPos &Cone::getPos()
{
	return pos;
}

const double &Cone::getConeRadius()
{
    return radius;
}

