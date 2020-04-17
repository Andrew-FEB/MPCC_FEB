#include "track.h"

Track::Track(std::shared_ptr<Visualisation> visualise_cont)
{

#ifdef VISUALISE
    if (visualise_cont!=nullptr)
    {
        visualise = visualise_cont;
    }
    else std::cerr << "ERROR: Visualise pre-processor statement defined but nullptr Visualise object passed to Track object";
#endif
    triangulate = std::make_unique<Triangulation>(visualise_cont);
    car = std::make_unique<Car>(visualise_cont);
#ifdef VISUALISE
    visualise->showCar(car->getPosition().p);
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

Car *Track::getCar()
{
    return car.get();
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
    auto carPos = car->getPosition().p;
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

MPC_targets Track::getCentreLine(const double &desired_dist)
{
    //DUMMY OPERATIONS FOR DADA
    std::cout<<"Car position = x: "<<car->getPosition().p.x<<" and y:"<<car->getPosition().p.y<<std::endl;
    std::cout<<"Desired distance to travel = "<<desired_dist<<std::endl;
    MPC_targets output;

    std::vector<double> x_points_dummy = {0, 12, 19, 38, 52, 65, 80, 100, 150, 300};
    std::vector<double> y_points_dummy = {0, 19, 5, -7, 22, 29, 16, 0, -40, -160};
    std::vector<coord> set_points;
    for (int i = 0; i<x_points_dummy.size(); i++)
    {
        set_points.push_back({x_points_dummy[i], y_points_dummy[i]});
    }
    #ifdef VISUALISE
    visualise->showNodeMids(set_points);
    #endif
    tk::spline centreline_spline;
    centreline_spline.set_points(x_points_dummy, y_points_dummy, true);
    std::vector<coord> vis_coords;
    for (double i = 0; i<*x_points_dummy.end(); i=i+2)
    {
        vis_coords.push_back({i, centreline_spline(i)});
    }
    #ifdef VISUALISE
    visualise->showCentreCoords(vis_coords);
    #endif

    //Find nearest point
    double best_distance = std::numeric_limits<double>::max();
    double best_spline_x = std::numeric_limits<double>::min();
    auto car_pos = car->getPosition().p;
    for (double i = 0; i<*x_points_dummy.end(); i=i+0.5)
    {
        coord spline_pos = {i, centreline_spline(i)};
        double spline_dis = pow(car_pos.x-spline_pos.x, 2) + pow(car_pos.y-spline_pos.y, 2);
        if (spline_dis < best_distance)
        {
            best_distance = spline_dis;
            best_spline_x = i;
        }
    }
    std::cout<<"Nearest point: x("<<best_spline_x<<"), y("<<centreline_spline(best_spline_x)<<")"<<std::endl;
    output.nearest_point  = {best_spline_x, centreline_spline(best_spline_x)};
    std::pair<coord, coord> path_to_nearest = std::make_pair(car_pos, output.nearest_point);
    #ifdef VISUALISE
    visualise->showNodeParentLinks({path_to_nearest});
    #endif

    //Find slope at point
    double delta = 0.05;
    coord a = {best_spline_x, centreline_spline(best_spline_x)};
    coord b = {best_spline_x+delta, centreline_spline(best_spline_x+delta)};
    double slope = (b.y-a.y)/(b.x-a.x);
    output.slope = slope;
    std::cout<<"Slope:"<<slope<<std::endl;

    //Find goal via arc length - messy approximation version
    double distance_so_far = 0;
    double index = best_spline_x;
    a = {best_spline_x, centreline_spline(best_spline_x)};
    while (distance_so_far<desired_dist)
    {
        index+=0.005;
        b = {index, centreline_spline(index)};
        distance_so_far += sqrt(pow(b.x-a.x, 2) + pow(b.y-a.y, 2));
        a = b;
    }
    std::cout<<"Distance so far: "<<distance_so_far<<std::endl;
    output.goal = a;
    std::cout<<"End goal: x("<<a.x<<"), y("<<a.y<<")"<<std::endl;
    #ifdef VISUALISE
    visualise->showEndPoint(output.goal);
    #endif
    
    return output;
    //END DUMMY
}

bool Track::checkShapeOverlap()
{

}

bool Track::checkForCollision(const coord &carPos, const double &carDirection)
{
    
}

