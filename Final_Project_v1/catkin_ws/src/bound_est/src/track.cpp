#include "track.h"
#include "boundGlobals.h"


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
    path_analysis = std::make_unique<PathAnalysis>();
    #ifdef DEBUG
    boundaries_log = std::make_unique<BoundaryLogger>("FIND_BOUNDS_AND_SLOPES", "findBoundaryPointsAndSlopes()", reset_logs);
    #endif
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
    if (new_cones.size()<=0) return;
    //Begin finding cones within valid ranges to find reference path
    extractNewConesInRange(new_cones, cones_within_range, car);
    if (cones_within_range.size()<=4) return;
    #ifdef VISUALISE
	    visualisation->showNewCones(new_cones);
        visualisation->showFramedCones(cones_within_range);
    #endif
    //Find entry point into next track section
    coord entry_point = (centre_coords.size()<1) ? car->getPosition().p : centre_coords.back();
    //Seperate cones into left and right
    auto seperated_cones = seperateConeList(cones_within_range);
    if (seperated_cones.first.size()<=0 || seperated_cones.second.size()<=0) return;
//Find end point goal of section
	coord section_end = findEndGoal(entry_point, seperated_cones);
    #ifdef VISUALISE
        visualisation->showLeftCones(seperated_cones.first);
        visualisation->showRightCones(seperated_cones.second);
    #endif
    //Get paths traversing framed track region
    auto paths = triangulate->getTraversingPaths(cones_within_range, entry_point, section_end, seperated_cones);
    #ifdef VISUALISE
        visualisation->showViablePaths(paths);
    #endif
    //Add new coordinates into final result

    auto best_path = path_analysis->findBestPath(paths, section_end, cones_within_range, seperated_cones.first, seperated_cones.second);
    std::move(best_path.begin(), best_path.end(), std::back_inserter(centre_coords));


    //Move new cones into processed cones
    std::move(cones_within_range.begin(), cones_within_range.end(), std::back_inserter(processed_cone_list)); 
    //Move seperated classified cones into processed cones 
    std::move(seperated_cones.first.begin(), seperated_cones.first.end(), std::back_inserter(processed_cone_list_left));
    std::move(seperated_cones.second.begin(), seperated_cones.second.end(), std::back_inserter(processed_cone_list_right));
    #ifdef VISUALISE
	    visualisation->showOldCones(processed_cone_list);
    #endif
        cones_within_range.clear(); 
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

coord Track::findEndGoal(const coord &last_point, const std::pair<std::vector<const Cone *>, std::vector<const Cone *>> &seperated_cone_lists)
{
	auto left_cone_best = findFurthestConeFromPoint(last_point, seperated_cone_lists.first);
	auto right_cone_best = findFurthestConeFromPoint(last_point, seperated_cone_lists.second);
	auto endPoint = findMidpoint(left_cone_best.first->getCoordinates(), right_cone_best.first->getCoordinates());
	return endPoint;
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
    #ifdef DEBUG
    std::unique_ptr<BoundaryLogger> log = std::make_unique<BoundaryLogger>("DEBUG_CLASSIFICATION", "Classification", reset_logs);
    std::stringstream ss;
    log->write(ss<<"Cone list size to be sorted = "<<cone_list.size(), true);
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

                    #ifdef DEBUG
                    log->write(ss<<"Entered determinant check");
                    log->write(ss<<"Cone position, x = "<<cone->getCoordinates().x<<", y = "<<cone->getCoordinates().y);
                    log->write(ss<<"Coord car, x = "<<a.x<<", y = "<<a.y);
                    log->write(ss<<"Coord car projection, x = "<<b.x<<", y = "<<b.y);
                    #endif
                    double determinant = (c.x-a.x)*(b.y-a.y)-(c.y-a.y)*(b.x-a.x); //d=(x−x1)(y2−y1)−(y−y1)(x2−x1)
                    if (determinant<0) 
                    {
                        #ifdef DEBUG
                        log->write(ss<<"Determinant: "<<determinant<<" so selected left");
                        #endif
                        cone->setPos(BoundPos::left);
                        left_list.push_back(cone.get());
                    }
                    else 
                    {
                        #ifdef DEBUG
                        log->write(ss<<"Determinant: "<<determinant<<" so selected right");
                        #endif
                        cone->setPos(BoundPos::right);
                        right_list.push_back(cone.get());
                    }
                }
                else
                {
                    auto cone_position = cone->getCoordinates();
                    std::vector<const Cone *> *left_compare{nullptr};
                    std::vector<const Cone *> *right_compare{nullptr};

                    if (left_list.empty()) left_compare = &processed_cone_list_left;
                    else left_compare = &left_list;
                    if (right_list.empty()) right_compare = &processed_cone_list_right;
                    else right_compare = &right_list;

                    auto previous_cone_left = findClosestConeToPoint(cone_position, *left_compare).first->getCoordinates();
                    auto previous_cone_right = findClosestConeToPoint(cone_position, *right_compare).first->getCoordinates();
                    #ifdef DEBUG
                    log->write(ss<<"Entered non-determinant check");
                    log->write(ss<<"Cone position, x = "<<cone->getCoordinates().x<<", y = "<<cone->getCoordinates().y);
                    log->write(ss<<"previous cone position left, x = "<<previous_cone_left.x<<", y = "<<previous_cone_left.y);
                    log->write(ss<<"previous cone position right, x = "<<previous_cone_right.x<<", y = "<<previous_cone_right.y);
                    #endif
                    if (distBetweenPoints(cone_position, previous_cone_left)<distBetweenPoints(cone_position, previous_cone_right))
                    {
                        #ifdef DEBUG
                        log->write(ss<<"Selected left");
                        #endif
                        cone->setPos(BoundPos::left);
                        left_list.push_back(cone.get());
                    }
                    else
                    {
                        #ifdef DEBUG
                        log->write(ss<<"Selected right");
                        #endif
                        cone->setPos(BoundPos::right);
                        right_list.push_back(cone.get());
                    }
                    #ifdef DEBUG
                    log->write(ss<<"Distance from left = "<<distBetweenPoints(cone_position, previous_cone_left));
                    log->write(ss<<"Distance from right = "<<distBetweenPoints(cone_position, previous_cone_right));
                    #endif
                }
            break;
        }
        #ifdef DEBUG
        log->write(ss<<"Printout of list states");
        log->write(ss<<"Left list size: "<<left_list.size()<<", Right list size: "<<right_list.size(), true);
        #endif
    }
    return std::make_pair(left_list, right_list);
}


std::vector<MPC_targets> Track::getReferencePath(const double &dist_between_points, const int &number_of_points)
{
    #ifdef DEBUG
    std::unique_ptr<BoundaryLogger> log = std::make_unique<BoundaryLogger>("GET_REFERENCE_PATH", "getReferencePath()", reset_logs);
    std::stringstream ss;
    log->write(ss<<"At entering function, important variables were...");
    log->write(ss<<"Requested distance between points of "<<distBetweenPoints);
    log->write(ss<<"Requested number of points of "<<number_of_points, true);
    #endif
    //Check if sufficient centre coordinates available.
    if (centre_coords.size()<=1)
    {
        std::cerr<<"No centreline coordinates to request. Error earlier in processing?"<<std::endl;
        #ifdef DEBUG
        log->write(ss<<"Insufficient centre line coordinates. Centreline currently has size "<<centre_coords.size()<<". Returning from function.", true);
        #endif
        return {};
    }

    //Santisise inputs
    if (dist_between_points<=0 || number_of_points<=0)
    {
        std::cerr<<"getCentreLine given 0 or negative-valued parameters"<<std::endl;
        #ifdef DEBUG
        log->write(ss<<"Inappropriate input parameters, either 0 or negative valued. Returning from function", true);
        #endif
        return {};
    }    

    double dist = pow(dist_between_points, 2);  //distBetweenPoints function does not square root result for speed

    #ifdef DEBUG
    log->write(ss<<"First, finding centre coordinate closest to car position");
    #endif
    //Find centre coord closest to car position
    auto car_pos = getCar()->getPosition().p;
    #ifdef DEBUG
    log->write(ss<<"Car has position x("<<car_pos.x<<"), y("<<car_pos.y<<")");
    #endif
    int best_dist_index{0};
    double best_dist{distBetweenPoints(centre_coords[best_dist_index], car_pos)};
    #ifdef DEBUG
    log->write(ss<<"Coordinate at index 0 with position x("<<centre_coords[best_dist_index].x<<"), y("<<centre_coords[best_dist_index].y<<") has distance to car of "<<best_dist);
    #endif
    for (int i = 1; i<centre_coords.size(); i++)
    {
        auto new_dist = distBetweenPoints(centre_coords[i], car_pos);
        #ifdef DEBUG
        log->write(ss<<"Coordinate at index "<<i<<" with position x("<<centre_coords[i].x<<"), y("<<centre_coords[i].y<<") has distance to car of "<<new_dist);
        #endif
        if (new_dist<best_dist)
        {
            #ifdef DEBUG
            log->write(ss<<"Selected as new best centre coordinate");
            #endif
            best_dist_index = i;
            best_dist = new_dist;
        }
    }
    #ifdef DEBUG
    log->write(ss<<"Final coordinate selected at index "<<best_dist_index<<" with position x("<<centre_coords[best_dist_index].x<<"), y("<<centre_coords[best_dist_index].y<<") and distance to car of "<<best_dist, true);
    #endif

    #ifdef DEBUG
    log->write(ss<<"Next, getting centre coordinates");
    #endif
    //Get centre coords
    auto centre_coord_pair = interpolateCentreCoordsDiscrete(best_dist_index, number_of_points, dist_between_points); 
    #ifdef DEBUG
    log->write(ss<<"Not enough centre coordinates with given distance. New temporary distance between points of "<<centre_coord_pair.second);
    #endif

    #ifdef DEBUG
    int debug_centre_index {0};
    log->write(ss<<"Centre path coordinates calculated: ");
    for (auto coord : centre_coord_pair.first)
    {
        ss<<"Coordinate "<<++debug_centre_index<<" with position x("<<coord.x<<"), y("<<coord.y<<")"<<std::endl;
    }
    log->write(ss, true);
    #endif

    #ifdef DEBUG
    log->write(ss<<"Next, getting Boundary points and slopes");
    #endif
    //Get boundary positions and slopes
    auto left_boundary_positions = findBoundaryPointsAndSlopes(processed_cone_list_left, centre_coord_pair.first);
    auto right_boundary_positions = findBoundaryPointsAndSlopes(processed_cone_list_right, centre_coord_pair.first);

    #ifdef DEBUG
    int debug_left_index {0};
    log->write(ss<<"Centre path coordinates calculated: ");
    for (auto pos : left_boundary_positions)
    {
        ss<<"Position "<<++debug_left_index<<" with coordinate x("<<pos.p.x<<"), y("<<pos.p.y<<") and angle of "<<pos.phi<<std::endl;
    }
    log->write(ss, true);
    int debug_right_index {0};
    for (auto pos : right_boundary_positions)
    {
        ss<<"Position "<<++debug_right_index<<" with coordinate x("<<pos.p.x<<"), y("<<pos.p.y<<") and angle of "<<pos.phi<<std::endl;
    }
    log->write(ss, true);
    #endif

    //Setup necessary structs and variables with scope above loop.
    MPC_targets output_struct;
    std::vector<MPC_targets> output_vec;

    for (int i = 0; i<number_of_points; i++)
    {
        output_struct.reference_point = centre_coord_pair.first[i];
        output_struct.left_boundary = left_boundary_positions[i];
        output_struct.right_boundary = right_boundary_positions[i];
        output_vec.push_back(output_struct);
    }
    #ifdef VISUALISE
	    visualisation->showReferencePath(output_vec);
    #endif

    return output_vec;
}

coord Track::getClosestPointOnLine (const coord &a, const coord &b, const coord &p)
{
    coord a_to_p {p.x-a.x, p.y-a.y};
    coord a_to_b {b.x-a.x, b.y-a.y};
    double a_to_b_squared = pow(a_to_b.x, 2)+pow(a_to_b.y,2);
    double dot_prod = a_to_p.x*a_to_b.x + a_to_p.y*a_to_b.y;
    double normalized_dist = dot_prod/a_to_b_squared;
    // if (normalized_dist <0) normalized_dist = 0;
    // if (normalized_dist >1) normalized_dist = 1;
    return {a.x+a_to_b.x*normalized_dist, a.y+a_to_b.y*normalized_dist};
}

std::vector<Pos> Track::findBoundaryPointsAndSlopes(const std::vector<const Cone *> &cones, const std::vector<coord> &coord_list)
{
    #ifdef DEBUG
    std::stringstream ss;
    #endif
    std::vector<Pos> output_vec;
    Pos pos;
    int closest_index;
    int second_closest_index;
    double closest_dist{};
    double second_closest_dist{};

    #ifdef DEBUG
    int debug_index{0};
    boundaries_log->write(ss<<"Entering loops to find nearest cones for each reference point");
    #endif
    for (auto &coord : coord_list)
    {
        #ifdef DEBUG
        boundaries_log->write(ss<<"Finding boundary points for coordinate "<<++debug_index<<" with position x("<<coord.x<<"), y("<<coord.y<<")");
        #endif
        int closest_index{0};
        double closest_dist{distBetweenPoints(cones[closest_index]->getCoordinates(), coord)};
        #ifdef DEBUG
        boundaries_log->write(ss<<"Starting closest distance of "<<closest_dist<<" at "<<closest_index);
        #endif
        for (int i = 1; i<cones.size(); i++)
        {
            auto distance = distBetweenPoints(cones[i]->getCoordinates(), coord);
            #ifdef DEBUG
            boundaries_log->write(ss<<"Next cone with position x("<<cones[i]->getCoordinates().x<<"), y("<<cones[i]->getCoordinates().y<<") has distance of "<<distance);
            #endif
            if (distance<closest_dist)
            {
                closest_index = i;
                closest_dist = distance;
                #ifdef DEBUG
                boundaries_log->write(ss<<"This is better than previous closest distance, so new closest distance "<<closest_dist<<" at index "<<i);
                #endif
            }
        }
        #ifdef DEBUG
        boundaries_log->write(ss<<"Loop ended", true);
        #endif
        auto cone_1_coords = cones[closest_index]->getCoordinates();
        
        double distance_previous = std::numeric_limits<double>::max();
        double distance_next = std::numeric_limits<double>::max();
        if ((closest_index-1)>=0)
        {
            distance_previous = distBetweenPoints(coord, cones[closest_index-1]->getCoordinates());
        }
        if ((closest_index+1)<cones.size())
        {
            distance_next = distBetweenPoints(coord, cones[closest_index+1]->getCoordinates());
        }
        auto cone_2_coords = (distance_previous<distance_next) ? cones[closest_index-1]->getCoordinates() : cones[closest_index+1]->getCoordinates();
        #ifdef DEBUG
        boundaries_log->write(ss<<"Final cone coordinates for cone 1 are x("<<cone_1_coords.x<<"), y("<<cone_1_coords.y<<") and cone 2 are x("<<cone_2_coords.x<<"), y("<<cone_2_coords.y<<")");
        #endif

        pos.p = getClosestPointOnLine(cone_1_coords, cone_2_coords, coord);
        double slope = (cone_2_coords.y-cone_1_coords.y)/(cone_2_coords.x-cone_1_coords.x);
        #ifdef DEBUG
        boundaries_log->write(ss<<"Closest point on line between line between cones and position x("<<coord.x<<"), y("<<coord.y<<") calculated to be x("<<pos.p.x<<"), y("<<pos.p.y<<") and slope calculated to be "<<slope ,true);
        #endif
        pos.phi = slope;
        output_vec.push_back(pos);
    }
    return output_vec;
}

std::pair<std::vector<coord>, double> Track::interpolateCentreCoordsDiscrete(const int &original_index, const int &number_of_points, const double &distance)
{
    #ifdef DEBUG
    std::unique_ptr<BoundaryLogger> log = std::make_unique<BoundaryLogger>("INTERPOLATE_CENTRE", "interpolateCentreCoordsDiscrete()", reset_logs);
    std::stringstream ss;
    log->write(ss<<"At entering function, important variables were...");
    log->write(ss<<"Original index of "<<original_index);
    log->write(ss<<"Requested distance between points of "<<distance);
    log->write(ss<<"Requested number of points of "<<number_of_points, true);
    #endif
    double temp_distance = distance;
    //Check available distance versus requested distance
    double available_dist{0};
    for (int i = (original_index+1); i<centre_coords.size(); i++)
    {
        available_dist+=distBetweenPoints(centre_coords[i-1], centre_coords[i]);
    }
    available_dist -= 0.01; //compensate for double rounding
    #ifdef DEBUG
    log->write(ss<<"Actual available distance is "<<available_dist);
    #endif
    if (available_dist<(number_of_points*distance))
    {
        temp_distance = available_dist/number_of_points;
        #ifdef DEBUG
        log->write(ss<<"Insufficient distance available so distance between points reduced to "<<temp_distance, true);
        #endif
    }

    //Move into developing points
    std::vector<coord> output_vec;
    int current_index{original_index};
    coord current_coord {centre_coords[original_index]};
    double next_distance;

    for (int i = 0; i<number_of_points; i++)
    {
        bool point_found{false};
        next_distance = temp_distance;
        #ifdef DEBUG
        log->write(ss<<"Finding point "<<i+1<< " of "<<number_of_points);
        #endif
        while (!point_found)
        {
            //Distance between next pair of centre coordinates
            auto coord_distance = distBetweenPoints(current_coord, centre_coords[current_index+1]);
            #ifdef DEBUG
            double debug_dist = 0;
            coord temp_coord = current_coord;
            //Check available distance versus requested distance
            if (current_index+1<centre_coords.size())
            {
                for (int i = (original_index+1); i<centre_coords.size(); i++)
                {
                    debug_dist+=distBetweenPoints(temp_coord, centre_coords[i]);
                    temp_coord = centre_coords[i];
                }
            }
            log->write(ss<<"Current centre coordinate x("<<centre_coords[current_index].x<<"), y("<<centre_coords[current_index].y<<")");
            log->write(ss<<"Distance remaining to final centre coord is "<<debug_dist);
            log->write(ss<<"Distance between current coord x("<<current_coord.x<<"), y("<<current_coord.y<<") and next centre coord x("<<centre_coords[current_index+1].x
                        <<"), y("<<centre_coords[current_index+1].y<<") is "<<coord_distance<<" and distance to travel till next point is "<<next_distance);
            log->write(ss<<"Distance until next coord should now be "<<((coord_distance-next_distance<0) ? 0 : coord_distance-next_distance));
            log->write(ss<<" ");
            #endif
            //If next set of coordinates closer than remaining distance, reduce remaining distance and start calculating from new centre coord
            if (coord_distance<next_distance)
            {
                next_distance -= coord_distance;
                current_index++;
                current_coord = centre_coords[current_index];
                #ifdef DEBUG
                log->write(ss<<"This is less than the required distance, so distance to travel reduced to "<<next_distance<<" and new coord set to x("<<centre_coords[current_index].x
                        <<"), y("<<centre_coords[current_index].y<<")");
                #endif
            }
            //If next set of coordinates equally distant to remaining distance, next_distance to 0 and add next point at next centre_coord
            else if (coord_distance == next_distance)
            {
                point_found = true;
                next_distance = 0;
                current_index++;
                current_coord = centre_coords[current_index];
                output_vec.push_back(current_coord);
                #ifdef DEBUG
                log->write(ss<<"This is equal to the required distance, so distance to travel reduced to 0, and new coord set to x("<<centre_coords[current_index].x
                        <<"), y("<<centre_coords[current_index].y<<") and this point added to vector");
                #endif
            }
            else if (coord_distance>next_distance)
            {
                auto coeff = next_distance/coord_distance;
                current_coord = {current_coord.x+coeff*(centre_coords[current_index+1].x-current_coord.x)
                                , current_coord.y+coeff*(centre_coords[current_index+1].y-current_coord.y)};
                output_vec.push_back(current_coord);
                point_found = true;
                next_distance = 0;
                #ifdef DEBUG
                log->write(ss<<"This is more than the required distance, so distance to travel reduced to 0, and new coord set to x("<<current_coord.x
                        <<"), y("<<current_coord.y<<") and this point added to vector. Distance to next centre coord is "<<distBetweenPoints(current_coord, centre_coords[current_index+1]));
                #endif
            }
            if(current_index+1>=centre_coords.size())
            {
                #ifdef DEBUG
                log->write(ss<<"Error. Overrun available distance", true);
                #endif
                std::cerr<<"Error. Overrun available distance"<<std::endl;
                return std::make_pair(output_vec, std::numeric_limits<double>::min());
            }
            #ifdef DEBUG
            log->write(ss<<"Distance to travel at end of loop is "<<next_distance, true);
            #endif
        }
        #ifdef DEBUG
        log->write(ss<<"Moving to next point");
        #endif
    }
    return std::make_pair(output_vec, temp_distance);
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
