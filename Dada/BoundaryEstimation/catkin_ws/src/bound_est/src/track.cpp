#include "track.h"

Track::Track(std::shared_ptr<Visualise> visualise_cont)
{

#ifdef VISUALISE
    if (visualise_cont!=nullptr)
    {
        visualise = visualise_cont;
    }
    else std::cerr << "ERROR: Visualise pre-processor statement defined but nullptr Visualise object passed to Track object";
#endif
    triangulate = std::make_unique<Triangulation>(visualise_cont);
    car = std::make_unique<Car>();
#ifdef VISUALISE
    visualise->showCar(car);
#endif
}

void Track::addCone(const double &x, const double &y, const BoundPos &pos)
{
    auto coneCheck = checkConePos(x, y);
    auto code = coneCheck.first;
    switch (code)
    {
        case ConeError::overwrite:
            if (coneCheck.second!=nullptr)
            {
              auto oldCone = coneCheck.second;
              oldCone->setX(x);
              oldCone->setY(y);
            }
        break;
        case ConeError::valid:
            coneList.push_back(std::make_unique<Cone>(x, y, pos));
        break;
        case ConeError::outlier:
        break;
    }
    if (coneList.size()>=7)
    {
        processTrackSect();
    }
}

Car &Track::getCarRef()
{
    return *car;
}

std::ostream& operator<<(std::ostream& os, Track& track)
{
    for (const Cone *cone : track.getConeList())
    {
        os << *cone;
    }
    return os;
}

std::pair<Track::ConeError, Cone *> Track::checkConePos(const double &x, const double &y)
{
    if (coneList.size()>0)  //Avoid undefined behaviour from operating on empty list
    {
        //Check if cone should overwrite existing data
        auto it = std::find_if(coneList.begin(), coneList.end(), [&x, &y] (auto const &cone)
        {
            return ((abs(x-cone->getX())*abs(x-cone->getX())+(abs(y-cone->getY())*abs(y-cone->getY())))<=minRadiusSquared);
        });
        if (it!=coneList.end()) return std::make_pair(ConeError::overwrite, it->get());

        //Cone is judged to be new cone. Check if cone is outlier
        auto &last_cone = coneList.back();
        if ((powf(x-last_cone->getX(), 2)+powf(y-last_cone->getY(), 2))>=maxRadiusSquared)
        {
            return std::make_pair(ConeError::outlier, nullptr);
        }

    }
    return std::make_pair(ConeError::valid, nullptr);
}

void Track::processTrackSect()
{
    auto carPos = car->getPosition();
    coord lastPoint = {carPos.x, carPos.y};
    /*if (centreCoords.size()<1)    //Better to start at car always, or at last coordinate?
    {
        lastPoint.x = car->getX();
        lastPoint.y = car->getY();
    }
    else
    {
        auto back = centreCoords.back();
        lastPoint.x = back.x;
        lastPoint.y = back.y;
    }*/
    auto result = triangulate->getCentreCoords(coneList, lastPoint);
    centreCoords.insert(centreCoords.end(), result.begin(), result.end());
        
}

std::vector<const Cone*> Track::getConeList()
{
    std::vector<const Cone*> conePtrs;
    for (const std::unique_ptr<Cone> &cone : coneList)
    {
        conePtrs.push_back(cone.get());
    }
    return conePtrs;
}

void Track::recalcSpline()
{
    std::vector<double> x, y;
    for (const coord &xy : centreCoords)
    {
        x.push_back(xy.x);
        y.push_back(xy.y);
    }
}

std::vector<tk::spline>Track::getCentreLine(const double &desired_dist)
{

}

bool Track::checkShapeOverlap()
{

}

bool Track::checkForCollision(const coord &carPos, const double &carDirection)
{
    
}

