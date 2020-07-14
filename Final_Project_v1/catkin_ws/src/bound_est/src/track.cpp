#include "track.h"
#include "boundGlobals.h"

#ifndef DEBUG
    #undef DEBUG_CLASSIFICATION
#endif

Track::Track(std::shared_ptr<Visualisation> visualisation_cont)
{
    #ifdef VISUALISE
    if (visualisation_cont!=nullptr)
    {
        visualisation = visualisation_cont;
    }
    else std::cerr << "ERROR: Visualise pre-processor statement defined but nullptr Visualisation object passed to Track object";
    #endif

    triangulate = std::make_unique<Triangulation>(visualisation_cont);
    car = std::make_unique<Car>();
    path_analysis = std::make_unique<PathAnalysis>(*car);
}

void Track::addCone(const double &x, const double &y, const BoundPos &pos)
{
    auto cone_check = checkConePos(x, y);
    auto code = cone_check.first;
    switch (code)
    {
        case ConeError::overwrite:
            if (cone_check.second!=nullptr)
            {
                auto old_cone = cone_check.second;
                old_cone->setX(x);
                old_cone->setY(y);
            }
            break;
        case ConeError::valid:
            new_cones.push_back(std::make_unique<Cone>(x, y, pos));
            break;
        case ConeError::outlier:
            break;
    }
}

Car *Track::getCar()
{
    return car.get();
}

std::ostream& operator<<(std::ostream& os, Track& track)
{
    os<<"Cones to be processed:"<<std::endl;
    for (const Cone *cone : track.getNewCones())
    {
        os << *cone;
    }
    os<<"Processed cones:"<<std::endl;
    for (const Cone *cone : track.getConeList())
    {
        os << *cone;
    }
    return os;
}

std::pair<Track::ConeError, Cone *> Track::checkConePos(const double &x, const double &y)
{
    //If vectors empty, std::find_if() returns last so no undefned behaviour at first check.

    //Check if cone in already processed cones
    auto it = std::find_if(processed_cone_list.begin(), processed_cone_list.end(), [&x, &y] (auto const &cone)
    {
        return ((abs(x-cone->getX())*abs(x-cone->getX())+(abs(y-cone->getY())*abs(y-cone->getY())))<=minRadiusSquared);
    });
    if (it!=processed_cone_list.end()) return std::make_pair(ConeError::overwrite, it->get());

    //  Might not be necessary. Think it over
    // //Cone is judged to be new cone. Check if cone is outlier
    // auto &last_cone = cone_list.back();
    // if ((powf(x-last_cone->getX(), 2)+powf(y-last_cone->getY(), 2))>=maxRadiusSquared)
    // {
    //     return std::make_pair(ConeError::outlier, nullptr);
    // }
  
    //Check if cone in cones yet to be processed
    it = std::find_if(new_cones.begin(), new_cones.end(), [&x, &y] (auto const &cone)
    {
        return ((abs(x-cone->getX())*abs(x-cone->getX())+(abs(y-cone->getY())*abs(y-cone->getY())))<=minRadiusSquared);
    });
    if (it!=new_cones.end()) return std::make_pair(ConeError::overwrite, it->get());

    return std::make_pair(ConeError::valid, nullptr);
}

void Track::processNextSection()
{
    #ifdef VISUALISE
        visualisation->showCar(car);
        visualisation->showCarDirection(car);
    #endif
    if (new_cones.size()<=0) return;
    //Begin finding cones within valid ranges to find reference path
    std::vector<std::unique_ptr<Cone>> cones_within_range;
    extractNewConesInRange(new_cones, cones_within_range, car);
    if (cones_within_range.size()<=0) return;
    #ifdef VISUALISE
	    visualisation->showNewCones(new_cones);
        visualisation->showFramedCones(cones_within_range);
    #endif
    //Find entry point into next track section
    coord entry_point = (centre_coords.size()<1) ? car->getPosition().p : centre_coords.back();

    //Seperate cones into left and right
    auto seperated_cones = seperateConeList(cones_within_range);

    #ifdef VISUALISE
        visualisation->showLeftCones(seperated_cones.first);
        visualisation->showRightCones(seperated_cones.second);
    #endif
    auto paths = triangulate->getTraversingPaths(cones_within_range, entry_point, seperated_cones);
    #ifdef VISUALISE
        visualisation->showViablePaths(paths);
    #endif
    //auto best_path = path_analysis->findBestPath(paths, new_cones);
    //centre_coords.insert(centre_coords.end(), best_path.begin(), best_path.end()); 

    //Move new cones into processed cones
    std::move(cones_within_range.begin(), cones_within_range.end(), std::back_inserter(processed_cone_list));
    new_cones.clear();    
    //Move seperated classified cones into processed cones 
    std::move(seperated_cones.first.begin(), seperated_cones.first.end(), std::back_inserter(processed_cone_list_left));
    new_cones.clear();    
    std::move(seperated_cones.second.begin(), seperated_cones.second.end(), std::back_inserter(processed_cone_list_right));
    new_cones.clear();    
    #ifdef VISUALISE
	    visualisation->showOldCones(processed_cone_list);
    #endif
}

std::vector<const Cone*> Track::getConeList()
{
    std::vector<const Cone*> conePtrs;
    for (const std::unique_ptr<Cone> &cone : processed_cone_list)
    {
        conePtrs.push_back(cone.get());
    }
    return conePtrs;
}

std::vector<const Cone*> Track::getNewCones()
{
    std::vector<const Cone*> conePtrs;
    for (const std::unique_ptr<Cone> &cone : new_cones)
    {
        conePtrs.push_back(cone.get());
    }
    return conePtrs;
}

void Track::recalcSpline()
{
    std::vector<double> x, y;
    for (const coord &xy : centre_coords)
    {
        x.push_back(xy.x);
        y.push_back(xy.y);
    }
}

std::pair<std::vector<const Cone *>, std::vector<const Cone *>> Track::seperateConeList(std::vector<std::unique_ptr<Cone>> &cone_list)
{
    std::vector<const Cone *> right_list;
    std::vector<const Cone *> left_list;
    #if defined(DEBUG_CLASSIFICATION) && defined(DEBUG)
    std::unique_ptr<BoundaryLogger> log = std::make_unique<BoundaryLogger>("DEBUG_CLASSIFICATION", "Classification", reset_logs);
    std::stringstream ss;
    #endif

    for (std::unique_ptr<Cone> &cone : cone_list)
    {
        switch(cone->getPos())
        {
            case (BoundPos::end):   //Not part of track boundary. Ignore for now.
            break;

            case (BoundPos::offramp):   //Not part of track boundary. Ignore for now.
            break;

            case (BoundPos::left):
                left_list.push_back(cone.get());
            break;

            case (BoundPos::right):
                right_list.push_back(cone.get());
            break;

            case(BoundPos::undefined): 
                //min track width 3m. Max distance between cones 5m
                if ((right_list.empty() && processed_cone_list_right.empty()) || (left_list.empty() && processed_cone_list_left.empty()))   //No previous cone classifications to use as initial state
                {
                    coord a = car->getPosition().p;
                    auto angle = car->getPosition().phi;
                    coord b = {cos(angle*M_PI/180)+a.x, sin(angle*M_PI/180)+a.y};
                    auto c = cone->getCoordinates();

                    #ifdef DEBUG_CLASSIFICATION
                    log->write(ss<<"Entered determinant check");
                    log->write(ss<<"Cone position, x = "<<cone->getCoordinates().x<<", y = "<<cone->getCoordinates().y);
                    log->write(ss<<"Coord car, x = "<<a.x<<", y = "<<a.y);
                    log->write(ss<<"Coord car projection, x = "<<b.x<<", y = "<<b.y);
                    #endif
                    double determinant = (c.x-a.x)*(b.y-a.y)-(c.y-a.y)*(b.x-a.x); //d=(x−x1)(y2−y1)−(y−y1)(x2−x1)
                    if (determinant<0) 
                    {
                        #ifdef DEBUG_CLASSIFICATION
                        log->write(ss<<"Determinant: "<<determinant<<" so selected left");
                        #endif
                        cone->setPos(BoundPos::left);
                        left_list.push_back(cone.get());
                    }
                    else 
                    {
                        #ifdef DEBUG_CLASSIFICATION
                        log->write(ss<<"Determinant: "<<determinant<<" so selected right");
                        #endif
                        cone->setPos(BoundPos::right);
                        right_list.push_back(cone.get());
                    }
                }
                else
                {
                    auto cone_position = cone->getCoordinates();
                    auto previous_cone_left = (left_list.empty()) ? processed_cone_list_left.back()->getCoordinates() : left_list.back()->getCoordinates();
                    auto previous_cone_right = (right_list.empty()) ? processed_cone_list_right.back()->getCoordinates() : right_list.back()->getCoordinates();
                    #ifdef DEBUG_CLASSIFICATION
                    log->write(ss<<"Entered non-determinant check");
                    log->write(ss<<"Cone position, x = "<<cone->getCoordinates().x<<", y = "<<cone->getCoordinates().y);
                    log->write(ss<<"previous cone position left, x = "<<previous_cone_left.x<<", y = "<<previous_cone_left.y);
                    log->write(ss<<"previous cone position right, x = "<<previous_cone_right.x<<", y = "<<previous_cone_right.y);
                    #endif
                    if (distBetweenPoints(cone_position, previous_cone_left)<distBetweenPoints(cone_position, previous_cone_right))
                    {
                        #ifdef DEBUG_CLASSIFICATION
                        log->write(ss<<"Selected left");
                        #endif
                        cone->setPos(BoundPos::left);
                        left_list.push_back(cone.get());
                    }
                    else
                    {
                        #ifdef DEBUG_CLASSIFICATION
                        log->write(ss<<"Selected right");
                        #endif
                        cone->setPos(BoundPos::right);
                        right_list.push_back(cone.get());
                    }
                    #ifdef DEBUG_CLASSIFICATION
                    log->write(ss<<"Distance from left = "<<distBetweenPoints(cone_position, previous_cone_left));
                    log->write(ss<<"Distance from right = "<<distBetweenPoints(cone_position, previous_cone_right));
                    #endif
                }
            break;
        }
        #ifdef DEBUG_CLASSIFICATION
        log->write(ss<<"Printout of list states");
        log->write(ss<<"Left list size: "<<left_list.size()<<", Right list size: "<<right_list.size(), true);
        #endif
    }
    return std::make_pair(left_list, right_list);
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
    for (int i = 0; i<x_points_dummy.back(); i++)
    {
        set_points.push_back({x_points_dummy[i], y_points_dummy[i]});
    }

    tk::spline centreline_spline;
    centreline_spline.set_points(x_points_dummy, y_points_dummy, true);
    std::vector<coord> vis_coords;
    for (double i = 0; i<x_points_dummy.back(); i=i+2)
    {
        vis_coords.push_back({i, centreline_spline(i)});
    }

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

    //Find slope at point
    double delta = 0.05;
    coord a = {best_spline_x, centreline_spline(best_spline_x)};
    coord b = {best_spline_x+delta, centreline_spline(best_spline_x+delta)};
    double slope = atan2((b.y-a.y), (b.x-a.x));
    output.slope = slope;
    output.slope_rads = slope*(180/3.142);
    std::cout<<"Slope:"<<slope<<std::endl;

    //Find goal via arc length - messy approximation version
    double distance_so_far = 0;
    double index = best_spline_x;
    a = {best_spline_x, centreline_spline(best_spline_x)};
    while (distance_so_far<desired_dist)
    {
        index+=0.1;
        b = {index, centreline_spline(index)};
        distance_so_far += sqrt(pow(b.x-a.x, 2) + pow(b.y-a.y, 2));
        a = b;
    }
    std::cout<<"Distance so far: "<<distance_so_far<<std::endl;
    output.goal = a;
    std::cout<<"End goal: x("<<a.x<<"), y("<<a.y<<")"<<std::endl;
    
    return output;
    //END DUMMY
}

coord getNearestPointOnArc(const coord &point)
{
    double t_val = 0;
    double error_val = std::numeric_limits<double>::max();
}

bool Track::checkShapeOverlap()
{

}

bool Track::checkForCollision(const coord &carPos, const double &carDirection)
{
    
}

void Track::extractNewConesInRange (std::vector<std::unique_ptr<Cone>> &cones_to_extract, std::vector<std::unique_ptr<Cone>> &extracted_cones, const std::unique_ptr<Car> &car)
{
    auto car_pos = car->getPosition().p;
    std::vector<std::unique_ptr<Cone>> in_range;

    auto result = std::partition(cones_to_extract.begin(), cones_to_extract.end(), [&car_pos](const std::unique_ptr<Cone> &cone)
    {
        auto distance = distBetweenPoints(cone->getCoordinates(), car_pos);
        return (distance>MIN_CONE_FRAME_RANGE && distance<MAX_CONE_FRAME_RANGE);
    });

    if (result!=cones_to_extract.end())
    {
        std::move(cones_to_extract.begin(), result, std::back_inserter(extracted_cones));
        cones_to_extract.erase(cones_to_extract.begin(), result);
    }
}
