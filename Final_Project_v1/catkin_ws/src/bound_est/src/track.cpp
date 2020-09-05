#include "track.h"
#include "boundGlobals.h"


Track::Track(std::shared_ptr<Visualisation> visualisation_cont)
{
    #ifdef VISUALISE
    if (visualisation_cont!=nullptr)
    {
        visualisation = visualisation_cont;
    }
    else std::cerr << "ERROR: Visualise pre-processor statement defined but nullptr Visualisation object passed to Track object"<<std::endl;
    #endif

    triangulate = std::make_unique<Triangulation>(visualisation_cont);
    car = std::make_unique<Car>();
    path_analysis = std::make_unique<PathAnalysis>(visualisation_cont, getCar());
    #ifdef DEBUG
    boundaries_log = std::make_unique<BoundaryLogger>("FIND_BOUNDS_AND_SLOPES", "findBoundaryPointsAndSlopes()", reset_logs);
    #endif
}

void Track::addCone(const double &x, const double &y, const BoundPos &pos)
{
    auto cone_check = checkConePos({x, y});
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

std::pair<Track::ConeError, Cone *> Track::checkConePos(const Coord &point)
{
    //If vectors empty, std::find_if() returns last so no undefned behaviour at first check.
    //Check if cone in already processed cones
    auto it = std::find_if(processed_cone_list.begin(), processed_cone_list.end(), [&point] (auto const &cone)
    {
        return withinCircleOfRadius(point, cone->getCoordinates(), REPEATED_CONE_RADIUS);
    });
    if (it!=processed_cone_list.end()) return std::make_pair(ConeError::overwrite, it->get());

    //Check if cone in cones yet to be processed
    it = std::find_if(new_cones.begin(), new_cones.end(), [&point] (auto const &cone)
    {
        return withinCircleOfRadius(point, cone->getCoordinates(), REPEATED_CONE_RADIUS);
    });
    if (it!=new_cones.end()) return std::make_pair(ConeError::overwrite, it->get());

    return std::make_pair(ConeError::valid, nullptr);
}

void Track::processNextSection(int path_length_limit)
{
    if (new_cones.size()<=0)
    {
        return;
    }
    //Begin finding cones within valid ranges to find reference path
    #ifdef VISUALISE
	    visualisation->showNewCones(new_cones);
    #endif
    auto framed_cones = extractConesInFrame();
    if (framed_cones.size()<=4) return;
    #ifdef VISUALISE
        visualisation->showFramedCones(framed_cones);
    #endif
    //Seperate cones into left and right
    auto seperated_cones = seperateConeList(framed_cones);
    if (seperated_cones.first.size()<=2 || seperated_cones.second.size()<=2)
    {
        #ifdef DEBUG
        std::cerr<<"Failed seperated cones check"<<std::endl;
        #endif
        std::move(framed_cones.begin(), framed_cones.end(), std::back_inserter(new_cones));
        return;
    }
    auto result = (seperated_cones.first.size()>seperated_cones.second.size()) ? seperated_cones.first.size()-seperated_cones.second.size() : seperated_cones.second.size()-seperated_cones.first.size();
    if (result>1)   //Unbalanced frame - rebalance
    {
        if (seperated_cones.first.size()>seperated_cones.second.size())
        {
            rebalanceCones(framed_cones, seperated_cones.first, result);
        }
        else
        {
            rebalanceCones(framed_cones, seperated_cones.second, result);
        }      
    }

    #ifdef VISUALISE
    visualisation->showLeftCones(seperated_cones.first);
    visualisation->showRightCones(seperated_cones.second);
    #endif

    //Find entry point into next track section
    Coord entry_point;
    if (centre_coords.size()<1)
    {
        entry_point = car->getPosition().p;
    }
    else
    {
        entry_point = centre_coords.back();
    }

    //Find end point goal of section
	Coord section_end = findEndGoal(entry_point, seperated_cones);

    //Get paths traversing framed track region
    auto paths = triangulate->getTraversingPaths(framed_cones, seperated_cones, entry_point, section_end, path_length_limit);
    #ifdef VISUALISE
        //visualisation->showViablePaths(paths);
    #endif

    //Add new coordinates into final result
    auto best_path = path_analysis->findBestPath(paths, section_end, framed_cones, seperated_cones.first, seperated_cones.second);
    if (!best_path.empty()) std::move(best_path.begin(), best_path.end(), std::back_inserter(centre_coords));
    //Outlier centre coords check
    if (centre_coords.size()>3 && (centre_coords.size()-centre_coords_checked)>2)
    {
        removeCentreCoordOutliers();
    }
    //Move new cones into processed cones
    std::move(framed_cones.begin(), framed_cones.end(), std::back_inserter(processed_cone_list)); 
    //Move seperated classified cones into processed cones 
    std::move(seperated_cones.first.begin(), seperated_cones.first.end(), std::back_inserter(processed_cone_list_left));
    std::move(seperated_cones.second.begin(), seperated_cones.second.end(), std::back_inserter(processed_cone_list_right));
    #ifdef VISUALISE
	    visualisation->showOldCones(processed_cone_list);
        visualisation->showCentreCoords(centre_coords);
    #endif 
    if (centre_coords.size()>NUM_CENTRELINE_COORDS_BEFORE_CHECK_TRACK_COMPLETE) track_complete = checkIfTrackComplete(centre_coords.back());
}
   
void Track::rebalanceCones(std::vector<std::unique_ptr<Cone>> &framed_cones, std::vector<const Cone *> &boundary_cones, const unsigned long &number_to_remove)
{
    int number_removed = 0;
    auto car_pos = car->getPosition().p;
    while (number_removed<=number_to_remove)
    {  
        auto furthest_cone = findFurthestConeFromPointWithIndex(car_pos, boundary_cones);
        for (int i = 0; i<framed_cones.size(); i++)
        {
            if ((*framed_cones[i])==furthest_cone.first)
            {
                if ((i+1)<framed_cones.size())
                {
                    std::rotate(framed_cones.begin()+i, framed_cones.begin()+i+1, framed_cones.end());
                }
                boundary_cones[furthest_cone.second] = boundary_cones.back();
                boundary_cones.pop_back();
                number_removed++;
                break;
            }
        }
    }
    std::move(framed_cones.end()-number_to_remove, framed_cones.end(), std::back_inserter(new_cones));
    framed_cones.erase(framed_cones.end()-number_to_remove, framed_cones.end());
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

Coord Track::findEndGoal(const Coord &last_point, const std::pair<std::vector<const Cone *>, std::vector<const Cone *>> &seperated_cone_lists)
{
	auto left_cone_best = findFurthestConeFromPoint(last_point, seperated_cone_lists.first);
	auto right_cone_best = findFurthestConeFromPoint(last_point, seperated_cone_lists.second);
    auto closest_to_left_best = findClosestConeToPoint(left_cone_best.first->getCoordinates(), seperated_cone_lists.second);
    auto closest_to_right_best = findClosestConeToPoint(right_cone_best.first->getCoordinates(), seperated_cone_lists.first);


    if (closest_to_left_best.first!=right_cone_best.first || closest_to_right_best.first!=left_cone_best.first)
    {
        if (right_cone_best.second>left_cone_best.second)
        {
            left_cone_best.first = closest_to_right_best.first;
        }
        else
        {
            right_cone_best.first = closest_to_left_best.first;
        }
    }
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
                    Coord a = car->getPosition().p;
                    auto angle = car->getPosition().phi;
                    Coord b = {cos(angle)+a.x, sin(angle)+a.y};
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


MPC_targets Track::getReferencePath(const std::vector<double> &distances, float boundary_compress)
{  
    #ifdef DEBUG
    std::unique_ptr<BoundaryLogger> log = std::make_unique<BoundaryLogger>("GET_REFERENCE_PATH", "getReferencePath()", reset_logs);
    std::stringstream ss;
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

    //Sanitise inputs
    auto negative_check = std::find_if(distances.begin(), distances.end(), [](const double &distance)
    {
        return (distance<=0);
    });
    if (distances.empty()|| negative_check!=distances.end())
    {
        std::cerr<<"getReferencePath given empty vector or vector containing negative distances. Returning from function."<<std::endl;
        #ifdef DEBUG
        log->write(ss<<"Inappropriate input parameters, given empty vector or vector containing negative distances. Returning from function", true);
        #endif
        return MPC_targets{};
    }    
    if (boundary_compress<=0.0 || boundary_compress>1.0)
    {
        std::cerr<<"Invalid boundary compression requested. Reverting to no compression"<<std::endl;
        boundary_compress = 1.0;
    }
    #ifdef DEBUG
    log->write(ss<<"At entering function, important variables were...");
    log->write(ss<<"Requested number of points of "<<distances.size(), true);
    log->write(ss<<"List of centre coordinates is as follows:");
    for (int i = 0; i<centre_coords.size(); i++)
    {
        ss<<"x("<<centre_coords[i].x<<"), y("<<centre_coords[i].y<<")"<<std::endl;
    }
    log->write(ss, true);
    #endif

    #ifdef DEBUG
    log->write(ss<<"First, finding centre coordinate closest to car position");
    #endif
    //Find centre coord closest to car position
    auto car_pos = getCar()->getPosition().p;
    #ifdef DEBUG
    log->write(ss<<"Car has position x("<<car_pos.x<<"), y("<<car_pos.y<<")");
    #endif

    auto nearest_centre = getClosestPointOnCentreLine(car_pos);

    #ifdef DEBUG
    double best_dist{distBetweenPoints(nearest_centre.first, car_pos)};
    log->write(ss<<"Final coordinate selected at index "<<nearest_centre.second<<" with position x("<<nearest_centre.first.x<<"), y("<<nearest_centre.first.y<<") and distance to car of "<<best_dist, true);
    log->write(ss<<"Next, getting centre coordinates");
    #endif
    //Get centre coords
    if (nearest_centre.second == (centre_coords.size()-1) && !track_complete)
    {
        std::cerr<<"Error, request for reference path no more centre-line path available. Returning empty path";
        return MPC_targets{};
    }
    auto centre_points = interpolateCentreCoordsDiscrete(nearest_centre.second, nearest_centre.first, distances); 

    #ifdef DEBUG
    int debug_centre_index {0};
    log->write(ss<<"Centre path coordinates calculated: ");
    for (auto coord : centre_points)
    {
        ss<<"Coordinate "<<++debug_centre_index<<" with position x("<<coord.x<<"), y("<<coord.y<<")"<<std::endl;
    }
    log->write(ss, true);
    #endif

    #ifdef DEBUG
    log->write(ss<<"Next, getting Boundary points and slopes");
    #endif
    //Get boundary positions and slopes
    int boundary_index = centre_points.size()/2;
    auto left_boundary_position = findBoundaryPointAndSlope(processed_cone_list_left, centre_points[boundary_index], boundary_compress);
    auto right_boundary_position = findBoundaryPointAndSlope(processed_cone_list_right, centre_points[boundary_index], boundary_compress);

    //Setup necessary structs and variables with scope above loop.
    MPC_targets output_struct;
    std::move(centre_points.begin(), centre_points.end(), std::back_inserter(output_struct.reference_points));
    output_struct.left_boundary = left_boundary_position;
    output_struct.right_boundary = right_boundary_position;
    #ifdef VISUALISE
	    visualisation->showReferencePath(output_struct);
    #endif
    return output_struct;
}

Coord Track::getClosestPointOnLine (const Coord &a, const Coord &b, const Coord &p)
{
    Coord a_to_p {p.x-a.x, p.y-a.y};
    Coord a_to_b {b.x-a.x, b.y-a.y};
    double a_to_b_squared = pow(a_to_b.x, 2)+pow(a_to_b.y,2);
    double dot_prod = a_to_p.x*a_to_b.x + a_to_p.y*a_to_b.y;
    double normalized_dist = dot_prod/a_to_b_squared;
    return {a.x+a_to_b.x*normalized_dist, a.y+a_to_b.y*normalized_dist};
}

Pos Track::findBoundaryPointAndSlope(const std::vector<const Cone *> &cones, const Coord &coord, const float &boundary_compress)
{
    #ifdef DEBUG
    std::stringstream ss;
    #endif
    std::vector<Pos> output_vec;
    Pos pos;
    int second_closest_index;
    #ifdef DEBUG
    boundaries_log->write(ss<<"Finding boundary points for coordinate with position x("<<coord.x<<"), y("<<coord.y<<")");
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
    if (boundary_compress<1.0)
    {

        auto dist = distBetweenPoints(pos.p, coord);
        pos.p = {(coord.x-pos.p.x)*(1-boundary_compress)+pos.p.x, ((coord.y-pos.p.y)*(1-boundary_compress)+pos.p.y)};
    }
    double slope = (cone_2_coords.y-cone_1_coords.y)/(cone_2_coords.x-cone_1_coords.x);
    #ifdef DEBUG
    boundaries_log->write(ss<<"Closest point on line between line between cones and position x("<<coord.x<<"), y("<<coord.y<<") calculated to be x("<<pos.p.x<<"), y("<<pos.p.y<<") and slope calculated to be "<<slope ,true);
    #endif
    pos.phi = slope;  
    return pos;
}

std::vector<Coord> Track::interpolateCentreCoordsDiscrete(const int &original_index, const Coord &start_point, const std::vector<double> &distances)
{
    #ifdef DEBUG
        std::unique_ptr<BoundaryLogger> log = std::make_unique<BoundaryLogger>("INTERPOLATE_CENTRE", "interpolateCentreCoordsDiscrete()", reset_logs);
        std::stringstream ss;
        log->write(ss<<"At entering function, important variables were...");
        log->write(ss<<"Original index of "<<original_index);
        log->write(ss<<"Requested number of points of "<<distances.size(), true);
    #endif
    //Check available distance versus requested distance
    bool resized_distance{false};
    double temp_distance;
    if (!track_complete)
    {
        double available_dist{0};
        available_dist+=distBetweenPoints(start_point, centre_coords[original_index+1]);
        if (centre_coords.size()>=original_index+2)
        {
            for (int i = (original_index+2); i<centre_coords.size(); i++)
            {
                available_dist+=distBetweenPoints(centre_coords[i-1], centre_coords[i]);
            }
        }
        available_dist -= 0.01; //compensate for double rounding
        #ifdef DEBUG
            log->write(ss<<"Actual available distance is "<<available_dist);
        #endif
        if (available_dist<std::accumulate(distances.begin(), distances.end(), 0.0))
        {
            resized_distance = true;
            temp_distance = (available_dist/distances.size())*0.95;
            #ifdef DEBUG
            log->write(ss<<"Insufficient distance available so distance between points reduced to "<<temp_distance, true);
            #endif
        }
    }
    //Move into developing points
    std::vector<Coord> output_vec;
    int current_index{original_index};
    Coord current_coord {start_point};
    double next_distance;

    for (int i = 0; i<distances.size(); i++)
    {
        bool point_found{false};
        next_distance = (resized_distance) ? temp_distance : distances[i];
        #ifdef DEBUG
            log->write(ss<<"Finding point "<<i+1<< " of "<<distances.size());
        #endif
        while (!point_found)
        {
            //Distance between next pair of centre coordinates
            auto coord_distance = distBetweenPoints(current_coord, centre_coords[current_index+1]);
            #ifdef DEBUG
            double debug_dist = 0;
            Coord temp_coord = current_coord;
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
                if (track_complete)
                {
                    current_index = 0;
                }
                else
                {
                    #ifdef DEBUG
                    log->write(ss<<"Error. Overrun available distance", true);
                    #endif
                    std::cerr<<"Error. Overrun available distance"<<std::endl;
                    return output_vec;
                }
            }
            #ifdef DEBUG
            log->write(ss<<"Distance to travel at end of loop is "<<next_distance, true);
            #endif
        }
        #ifdef DEBUG
        log->write(ss<<"Moving to next point");
        #endif
    }
    return output_vec;
}

std::vector<std::unique_ptr<Cone>> Track::extractConesInFrame()
{
    auto car_pos = car->getPosition();
    std::vector<std::unique_ptr<Cone>> framed_cones;
    /*CircleSection circle_sec;
    Rect frame;
    if (centre_coords.empty())
    {
        circle_sec = formCircleSection(car_pos.p, car_pos.phi, MAX_VISION_CONE_FRAME_RANGE, CONE_VISION_ARC);
        //frame = projectTrackFramePoints(car->getPosition(), TRACK_FRAME_WIDTH_DIV_2, TRACK_FRAME_LENGTH);
    }
    else
    {
        double direction = atan2(centre_coords.back().y-car_pos.p.y, centre_coords.back().x-car_pos.p.y);
        circle_sec = formCircleSection(car_pos.p, direction, MAX_VISION_CONE_FRAME_RANGE, CONE_VISION_ARC);
        //frame = frame = projectTrackFramePoints({car->getPosition().p, direction}, TRACK_FRAME_WIDTH_DIV_2, TRACK_FRAME_LENGTH);
    }
    auto result = std::partition(new_cones.begin(), new_cones.end(), [&circle_sec, &frame](const std::unique_ptr<Cone> &cone)
    {
        //return (checkIfPointInCircleSection(circle_sec, cone->getCoordinates()) && frame.containsPoint(cone->getCoordinates()));
        return (checkIfPointInCircleSection(circle_sec, cone->getCoordinates())); 
    });
    #ifdef VISUALISE
    visualisation->showCarVision(circle_sec);
    visualisation->showTrackFrame(frame);
    #endif
    if (result!=new_cones.end() && std::distance(new_cones.begin(), result)>MIN_FRAMED_CONES_TO_PROCESS)
    {
        std::move(new_cones.begin(), result, std::back_inserter(framed_cones));
        new_cones.erase(new_cones.begin(), result);
    }*/
    std::move(new_cones.begin(), new_cones.end(), std::back_inserter(framed_cones));
    new_cones.erase(new_cones.begin(), new_cones.end());
    return framed_cones;
}

bool Track::trackIsComplete()
{
    return track_complete;
}

std::pair<Coord, int> Track::getClosestPointOnCentreLine(const Coord &point)
{
    Coord final_point_to_check;
    auto nearest_coord_index = findClosestCentreCoordIndex(point);
    int second_coord_index;
    if (centre_coords.size()>1)
    {
        if (track_complete)
        {
            second_coord_index = (centre_coords.size()>nearest_coord_index+1) ? nearest_coord_index+1 : 0;
        }
        else
        {
            second_coord_index = (centre_coords.size()>nearest_coord_index+1) ? nearest_coord_index+1 : nearest_coord_index-1;
        }
        final_point_to_check = getClosestPointOnLine(centre_coords[nearest_coord_index], centre_coords[second_coord_index], point);
    }
    else final_point_to_check = centre_coords[nearest_coord_index];
    return std::make_pair(final_point_to_check, nearest_coord_index);
}

bool Track::carIsInsideTrack()
{
    if (centre_coords.size() <=0 ) return true;
    auto car_pos = car->getPosition();
    auto closest_point = getClosestPointOnCentreLine(car_pos.p).first;
    auto closest_cone = findClosestConeToPoint(closest_point, processed_cone_list);
    auto car_edges = projectCarPoints(car_pos);
    int furthest_point_index{0};
    double furthest_dist{distBetweenPoints(car_edges[0], closest_point)};
    for (int i = 1; i<car_edges.size(); i++)
    {
        auto dist = distBetweenPoints(car_edges[i], closest_point);
        if (dist>furthest_dist)
        {
            furthest_point_index = i;
            furthest_dist = dist;
        }
    }
    #ifdef VISUALISE
    visualisation->showBoundaryCircle(closest_cone.second, closest_point, closest_cone.first->getCoordinates(), closest_point);
    std::vector<Coord> non_crit_rect_points;
    auto inside_boundaries = withinCircleOfRadius(car_edges[furthest_point_index], closest_point, closest_cone.second);
    visualisation->showCarBoundaryPoints(car_edges, furthest_point_index, inside_boundaries);
    return inside_boundaries;
    #else
    return withinCircleOfRadius(car_edges[furthest_point_index], closest_point, closest_cone.second);
    #endif
}

bool Track::pointIsInsideTrack(const Coord &point)
{
    auto closest_point = getClosestPointOnCentreLine(point);
    auto radius = findClosestConeToPoint(point, processed_cone_list).second;
    return withinCircleOfRadius(point, closest_point.first, radius);
}   

int Track::findClosestCentreCoordIndex(const Coord &point)
{
    if (centre_coords.size()<=1) return 0;
    int best_dist_index{0};
    double best_dist{distBetweenPoints(centre_coords[best_dist_index], point)};
    for (int i = 1; i<centre_coords.size(); i++)
    {
        auto new_dist = distBetweenPoints(centre_coords[i], point);
        if (new_dist<best_dist)
        {
            best_dist_index = i;
            best_dist = new_dist;
        }
    }
    return best_dist_index;
}

bool Track::checkIfTrackComplete(const Coord &last_centre_point)
{
    if (withinCircleOfRadius(last_centre_point, centre_coords[0], TRACK_COMPLETE_CHECK_RADIUS))
    {
        double div_dist = distBetweenPoints(last_centre_point, centre_coords[0])/NUM_POINTS_TO_CHECK_FOR_OUT_OF_BOUNDS;
        auto direction = atan2(centre_coords[0].y-centre_coords[0].y, centre_coords[0].x-centre_coords[0].x);
        for (int i = 1; i<=NUM_POINTS_TO_CHECK_FOR_OUT_OF_BOUNDS; i++)
        {
            Coord projected_point = {last_centre_point.x + div_dist*cos(direction), last_centre_point.y + div_dist*sin(direction)}; 
            if (!pointIsInsideTrack(projected_point)) return false;
        }
        #ifdef VISUALISE
	        visualisation->clearMapping();
        #endif
        return true;
    }
    return false;
}

inline std::vector<Coord> Track::projectCarPoints(const Pos &pos)
{
    std::vector<Coord> output;  
    output.push_back(rotateToAngle({CarParams.length_div_2, CarParams.width_div_2}, pos));
    output.push_back(rotateToAngle({CarParams.length_div_2, -CarParams.width_div_2}, pos));
    output.push_back(rotateToAngle({-CarParams.length_div_2, -CarParams.width_div_2}, pos));
    output.push_back(rotateToAngle({-CarParams.length_div_2, CarParams.width_div_2}, pos));
    return output;
}

inline Rect Track::projectTrackFramePoints(const Pos &pos, const double &width_div_2, const double &length)
{ 
    Rect output;  
    if (clockwise_track)
    {
        output.points[1] = (rotateToAngle({length, width_div_2*3}, pos));
        output.points[2] = (rotateToAngle({0, width_div_2*3}, pos));
    }
    else
    {      
        output.points[1] = (rotateToAngle({length, width_div_2}, pos));
        output.points[2] = (rotateToAngle({0, width_div_2}, pos));
    }
    
    if (counter_clockwise_track)
    {
        output.points[0] = (rotateToAngle({length, -width_div_2*3}, pos));
        output.points[3] = (rotateToAngle({0, -width_div_2*3}, pos));
    }
    else
    {
        output.points[0] = (rotateToAngle({length, -width_div_2}, pos));
        output.points[3] = (rotateToAngle({0, -width_div_2}, pos));
    }

    return output;
}

void Track::removeCentreCoordOutliers()
{
    int i {centre_coords_checked};
    for (i; i<(centre_coords.size()-1); i++)
    {
        auto coord_to_check = centre_coords[i];
        double direction_prev_check = atan2((centre_coords[i].y-centre_coords[i-1].y), (centre_coords[i].x-centre_coords[i-1].x));
        double direction_next_check = atan2((centre_coords[i+1].y-centre_coords[i].y), (centre_coords[i+1].x-centre_coords[i].x));
        double direction_skip_check = atan2((centre_coords[i+1].y-centre_coords[i-1].y), (centre_coords[i+1].x-centre_coords[i-1].x));
        
        if (abs(abs(direction_skip_check)-abs(direction_prev_check))>(abs(abs(direction_skip_check)-abs(direction_next_check))+OUTLINE_COORD_TOLERANCE))
        {
            centre_coords.erase(centre_coords.begin() + i);
            i--;
        }
        else
        {
            centre_coords_checked++;
        }
    }
}

void Track::checkForLap()
{
    if (centre_coords.size()<=0) return;
    if (inside_start_zone)
    {
        inside_start_zone = withinCircleOfRadius(car->getPosition().p, centre_coords[0], CHECK_LAP_RADIUS);
        if (!inside_start_zone && check_for_next_lap)
        {
            num_laps_raced++;
            check_for_next_lap = false;
        } 
    }
    else
    {
        inside_start_zone = withinCircleOfRadius(car->getPosition().p, centre_coords[0], CHECK_LAP_RADIUS);
        if (!check_for_next_lap) check_for_next_lap = !(withinCircleOfRadius(car->getPosition().p, centre_coords[0], DISTANCE_FROM_START_BEFORE_CHECK_FOR_NEXT_LAP));
    }
}

int Track::getLapsRaced()
{
    return num_laps_raced;
}