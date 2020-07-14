#ifndef CONE_H
#define CONE_H

#include <iostream>

enum class BoundPos
{
    undefined,
    left,
    right
};

class Cone
{

public:

    Cone(const double &x, const double &y, const BoundPos& pos = BoundPos::undefined);
    ~Cone() = default;
    friend std::ostream& operator<<(std::ostream& os, const Cone &cone);

    const double &getX();
    
    const double &getY();

    const int& getID();
    
    void setX(double value);
    
    void setY(double value);

	void setPos(const BoundPos& pos);
   

private:
    double x = 0;
    double y = 0;
    const int id = 0;
    BoundPos pos = BoundPos::undefined;
};

#endif // CONE_H
