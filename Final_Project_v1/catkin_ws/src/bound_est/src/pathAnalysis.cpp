#include "pathAnalysis.h"

PathAnalysis::PathAnalysis(std::shared_ptr<Visualisation> visualisation_cont, const Car *car_r)
{
    car = car_r;
    #ifdef VISUALISE
	if (visualisation_cont!=nullptr)
	{
		visualisation = visualisation_cont;
	}
	else std::cerr << "ERROR: Visualise pre-processor statement defined but nullptr Visualisation object passed to Triangulation object";
	#endif
}

const std::vector<Coord> &PathAnalysis::findBestPath(const std::vector<std::vector<Coord>> &paths, const Coord &end_goal, const std::vector<std::unique_ptr<Cone>> &cone_list, const std::vector<const Cone *> &left_cones, const std::vector<const Cone *> &right_cones)
{
    #ifdef DEBUG
    path_log = std::make_unique<BoundaryLogger>("PATH_ANALYSIS", "Path Analysis", reset_logs);
    #endif
    if(paths.size()<=0)
    {
        std::cerr<<"Error, findBestPath given empty vector of paths"<<std::endl;
        return {};
    }
    #ifdef DEBUG
    std::stringstream ss;
    path_log->write(ss<<"Beginning analysis of following list of paths:");
	int path_count{0};
	for (auto &path : paths)
	{
		int point_count{0};
		for (auto &point : path)
		{
			ss<<"Point "<<++point_count<<" at x("<<point.x<<") y("<<point.y<<")"<<std::endl;
		}
		path_log->write(ss, true);
	}
	#endif
	std::vector<double> cost_scores;
	cost_scores.resize(paths.size());
    #ifdef DEBUG
    path_log->write(ss<<"Scores gathered as follows:", true);
    #endif
	int index{0};
	for (auto pathIt = paths.begin(); pathIt!=paths.end(); pathIt++)
	{
        #ifdef DEBUG
        path_log->write(ss<<"For path "<<index+1);
        #endif
        cost_scores[index] = ((*pathIt).size() <= 0) ? std::numeric_limits<double>::max() : cost_scores[index] = findPathCost(*pathIt, end_goal, cone_list, left_cones, right_cones);
        index++;
	}
    auto best_index = findLowestCostPath(cost_scores);	
    #ifdef DEBUG
    path_log->write(ss<<"Best index selected to be "<<best_index<<" which equates to path "<<best_index+1<<". Returning analysis to track.", true);
	#endif
	return paths[best_index];	//return path associated with lowest cost function score
}

double PathAnalysis::findPathCost(const std::vector<Coord> &path, const Coord &end_goal, const std::vector<std::unique_ptr<Cone>> &cone_list, const std::vector<const Cone *> &left_cones, const std::vector<const Cone *> &right_cones)
{
    #ifdef DEBUG
    std::stringstream ss;
    #endif
    //Find cone contact scores
    auto a1 = classifiedConeContactCost(path, left_cones, right_cones);
    auto a2 = unclassifiedConeContactCost(path, cone_list);

    //Find distance from end goal
    auto b = pow(distBetweenPoints(path.back(), end_goal)*DISTANCE_END_COEFF, 2);

    //Compare number of cones on each side
    auto c1 = classifiedConesEachSide(path, left_cones, right_cones);
    auto c2 = unclassifiedConesEachSide(path, cone_list);

    //Find if each point advancing torwards goal
    auto d = advancingToGoal(path, end_goal);

    //Check for repeats
    auto e = checkForRepeatNodes(path);

    #ifdef DEBUG
    double output = (a1 + a2 + b + c1 + c2 + d + e);
    path_log->write(ss<<"Nearest classified cone contacts = "<<a1);
    path_log->write(ss<<"Nearest unclassified cone contacts = "<<a2);
    path_log->write(ss<<"Distance to end goal = "<<b);
    path_log->write(ss<<"Classified cones on each side = "<<c1);
    path_log->write(ss<<"Unclassified cones on each side = "<<c2);
    path_log->write(ss<<"Advancing to end goal throughout path = "<<d);
    path_log->write(ss<<"Check for repeat nodes = "<<e);
    path_log->write(ss<<"Total calculated score: "<<a1<<" + "<<a2<<" + "<<b<<" + "<<c1<<" + "<<c2<<" + "<<d<<" + "<<e<<" = "<<output, true);
    return output;
    #else
    return (a1+a2+b+c1+c2+d+e);
    #endif

}

int PathAnalysis::findLowestCostPath(const std::vector<double> &cost_scores)
{
    #ifdef DEBUG
    std::stringstream ss;
    path_log->write(ss<<"List of scores:");
	int path_count{0};
	for (auto &score : cost_scores)
	{
		path_log->write(ss<<"Path "<<++path_count<<" has score "<<score);
	}
	#endif
    int index = 0;
    for (int i = 1; i<cost_scores.size(); i++)
    {
        if (cost_scores[i]<cost_scores[index])
        {
            index = i;
        }
    }
    return index;
}

double PathAnalysis::classifiedConeContactCost(const std::vector<Coord> &path, const std::vector<const Cone *> &left_cones, const std::vector<const Cone *> &right_cones)
{
    double cost {0};
    for (auto &coord : path)
    {
        auto left_result = findClosestConeToPoint(coord, left_cones);
        auto right_result = findClosestConeToPoint(coord, right_cones);
        cost = cost + pow((left_result.second - right_result.second), 2);
    }
    return cost/path.size();
}

double PathAnalysis::unclassifiedConeContactCost(const std::vector<Coord> &path, const std::vector<std::unique_ptr<Cone>> &cone_list)
{
    double cost {0};
    std::vector<double> dists;
    for (auto &coord : path)
    {
        auto left_result = findClosestConeToPoint(coord, cone_list);
        dists.push_back(left_result.second);
    }
    for (int i = 1; i<dists.size(); i++)
    {
        cost+= pow((dists[i]-dists[i-1]), 2);
    }
    return cost/path.size();
}


double PathAnalysis::classifiedConesEachSide(const std::vector<Coord> &path, const std::vector<const Cone *> &left_cones, const std::vector<const Cone *> &right_cones)
{
    auto car_pos = car->getPosition();
    Rect left;
    left.a = {car_pos.p.x - RECT_CHECK_WIDTH*cos(atan(car_pos.phi)), car_pos.p.y + RECT_CHECK_HEIGHT_DIV_2*sin(atan(car_pos.phi))};
    left.b = {car_pos.p.x - cos(atan(car_pos.phi)), car_pos.p.y + RECT_CHECK_HEIGHT_DIV_2*sin(atan(car_pos.phi))};
    left.c = {car_pos.p.x - cos(atan(car_pos.phi)), car_pos.p.y - RECT_CHECK_HEIGHT_DIV_2*sin(atan(car_pos.phi))};
    left.d = {car_pos.p.x - RECT_CHECK_WIDTH*cos(atan(car_pos.phi)), car_pos.p.y - RECT_CHECK_HEIGHT_DIV_2*sin(atan(car_pos.phi))};
    Rect right;
    right.a = {car_pos.p.x + cos(atan(car_pos.phi)), car_pos.p.y + RECT_CHECK_HEIGHT_DIV_2*sin(atan(car_pos.phi))};
    right.b = {car_pos.p.x + RECT_CHECK_WIDTH*cos(atan(car_pos.phi)), car_pos.p.y + RECT_CHECK_HEIGHT_DIV_2*sin(atan(car_pos.phi))};
    right.c = {car_pos.p.x + RECT_CHECK_WIDTH*cos(atan(car_pos.phi)), car_pos.p.y - RECT_CHECK_HEIGHT_DIV_2*sin(atan(car_pos.phi))};
    right.d = {car_pos.p.x + cos(atan(car_pos.phi)), car_pos.p.y - RECT_CHECK_HEIGHT_DIV_2*sin(atan(car_pos.phi))};

    std::vector<int> detections;
    double cost {0};
    for (auto coord : path)
    {
        int left_cones_count{0};
        int right_cones_count{0};
        for (auto &cone : left_cones)
        {
            if (left.containsPoint(cone->getCoordinates())) left_cones_count++;
        }
        for (auto &cone : right_cones)
        {
            if (right.containsPoint(cone->getCoordinates())) right_cones_count++;
        }
        if (left_cones_count == 0 || right_cones_count == 0) cost+=NO_CONES_SPOTTED_COEFF;
        else
        {
            detections.push_back(left_cones_count-right_cones_count);
        }
    }
    for (int i = 1; i<detections.size(); i++)
    {
        cost+= pow((detections[i]-detections[i-1])*CONE_IMBALANCE_COEFF, 2);
    }
    return cost/path.size();
}

double PathAnalysis::unclassifiedConesEachSide(const std::vector<Coord> &path, const std::vector<std::unique_ptr<Cone>> &cone_list)
{
    auto car_pos = car->getPosition();
    Rect rect;
    rect.a = {car_pos.p.x - RECT_CHECK_WIDTH*cos(atan(car_pos.phi)), car_pos.p.y + RECT_CHECK_HEIGHT_DIV_2*sin(atan(car_pos.phi))};
    rect.b = {car_pos.p.x + RECT_CHECK_WIDTH*cos(atan(car_pos.phi)), car_pos.p.y + RECT_CHECK_HEIGHT_DIV_2*sin(atan(car_pos.phi))};
    rect.c = {car_pos.p.x + RECT_CHECK_WIDTH*cos(atan(car_pos.phi)), car_pos.p.y - RECT_CHECK_HEIGHT_DIV_2*sin(atan(car_pos.phi))};
    rect.d = {car_pos.p.x - RECT_CHECK_WIDTH*cos(atan(car_pos.phi)), car_pos.p.y - RECT_CHECK_HEIGHT_DIV_2*sin(atan(car_pos.phi))};

    double cost {0};
    std::vector<int> detections;
    for (auto &cone : cone_list)
    {
        int cone_count{0};
        if (rect.containsPoint(cone->getCoordinates())) cone_count++;
        if (cone_count == 0) cost+=NO_CONES_SPOTTED_COEFF;
    }
    for (int i = 1; i<detections.size(); i++)
    {
        cost+= pow((detections[i]-detections[i-1])*CONE_IMBALANCE_COEFF, 2);
    }
    return cost/path.size();
}

double PathAnalysis::advancingToGoal(const std::vector<Coord> &path, const Coord &end_goal)
{
    double dist {0};
    for (int i = 1; i<path.size(); i++)
    {
        auto progress = distBetweenPoints(end_goal, path[i])-distBetweenPoints(end_goal, path[i-1]);
        if (progress<0) dist += progress;
    }
    return pow(dist, 2)/path.size();
}

double PathAnalysis::checkForRepeatNodes(const std::vector<Coord> &path)
{
    int duplicates {0};
    for (int i = 0; i<path.size(); i++)
    {
        for (int j = 0; j<path.size(); j++)
        {
            if (path[i]==path[j])
            {
                duplicates++;
            }
        }
    }
    return NODE_REPEAT_COEFF*(duplicates-path.size());
}
