#ifndef CONE_H
#define CONE_H

#include <iostream>

#include "definitions.h"

class Cone
{

public:

    Cone(const double &x, const double &y, const BoundPos& pos = BoundPos::undefined);
    ~Cone() = default;
    friend std::ostream& operator<<(std::ostream& os, const Cone &cone);

    const double &getX() const;
    const double &getY() const;
    const Coord &getCoordinates() const;
    void setX(double value);
    void setY(double value);
    void setCoordinates(const Coord &coord);

    const int& getID() const;

    const BoundPos &getPos();   //Indicates track boundary position (options defined in enum Boundpos in definitions.h)
    void setPos(const BoundPos& pos);

    const double &getConeRadius();  //Not currently used. In case cone size becomes relevant for avoiding collisions and they can no longer be treated as point masses

    bool operator==(const Cone *a) const
	{
		return (coordinates.x==a->getX() && coordinates.y == a->getY());
	}
   

private:
    Coord coordinates;
    const int id = 0;
    BoundPos pos = BoundPos::undefined;
};

#endif // CONE_H
