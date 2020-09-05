#include "cone.h"

static int ID_VAL = 0;
static constexpr double radius = 0.3;

Cone::Cone(const double &newX, const double &newY, const BoundPos &newPos): coordinates({newX,newY}), pos(newPos), id(ID_VAL++)
{
}

const double &Cone::getX() const 
{
    return coordinates.x;
}

const double &Cone::getY() const
{
    return coordinates.y;
}

const int &Cone::getID() const
{
	return id;
}

void Cone::setX(double value)
{
    coordinates.x = value;
}

void Cone::setY(double value)
{
    coordinates.y = value;
}

void Cone::setCoordinates(const Coord &coord)
{
    coordinates = coord;
}


void Cone::setPos(const BoundPos& newPos)
{
	pos = newPos;
}

std::ostream &operator<<(std::ostream &os, const Cone &cone)
{
	//BoundStrTranslate defined in definitions.h. Used as map to translate enum class to string.
    os << "Cone id="<<cone.id<<", x=" << cone.coordinates.x << ", y=" << cone.coordinates.y << " and calculated boundary = "<<BoundStrTranslate.find(cone.pos)->second<<std::endl;
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

const Coord &Cone::getCoordinates() const
{
    return coordinates;
}

