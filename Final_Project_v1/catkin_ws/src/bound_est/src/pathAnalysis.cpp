#include "pathAnalysis.h"

PathAnalysis::PathAnalysis()
{
    #ifdef DEBUG
    path_log = std::make_unique<BoundaryLogger>("PATH_ANALYSIS", "Path Analysis", reset_logs);
    #endif
}

const std::vector<coord> &PathAnalysis::findBestPath(const std::vector<std::vector<coord>> &paths, const coord &end_goal, const std::vector<std::unique_ptr<Cone>> &cone_list, const std::vector<const Cone *> &left_cones, const std::vector<const Cone *> &right_cones)
{
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

double PathAnalysis::findPathCost(const std::vector<coord> &path, const coord &end_goal, const std::vector<std::unique_ptr<Cone>> &cone_list, const std::vector<const Cone *> &left_cones, const std::vector<const Cone *> &right_cones)
{
    #ifdef DEBUG
    std::stringstream ss;
    #endif
    //Find cone contact scores
    auto c1 = classifiedConeContactCost(path, left_cones, right_cones);
    auto c2 = unclassifiedConeContactCost(path, cone_list);

    //Find distance from end goal
    auto d = distBetweenPoints(path.back(), end_goal);

    //Compare number of cones on each side

    //Find if each point advancing torwards goal
double constexpr UNCLASS_CONES_EACH_SIDE_COEFF{10.0};
double constexpr CLASS_CONES_SIDE_COEFF{10.0};
double constexpr ADVANCING_COEFF{10.0};
    #ifdef DEBUG
    double output = (CLASS_NEAREST_CONE_COEFF*c1 + UNCLASS_NEAREST_CONE_COEFF*c2 + DISTANCE_END_COEFF*d);
    path_log->write(ss<<"Nearest classified cone contacts = "<<c1);
    path_log->write(ss<<"Nearest unclassified cone contacts = "<<c2);
    path_log->write(ss<<"Distance to end goal = "<<d);
    path_log->write(ss<<"Total calculated score: "<<CLASS_NEAREST_CONE_COEFF<<"*"<<c1<<" + "<<UNCLASS_NEAREST_CONE_COEFF<<"*"<<c2<<" + "<<DISTANCE_END_COEFF<<"*"<<d<<" = "<<output, true);
    return output;
    #else
    return (CLASS_NEAREST_CONE_COEFF*c1 + UNCLASS_NEAREST_CONE_COEFF*c2 + DISTANCE_TO_END_COEFF*d);
    #endif

}

double PathAnalysis::classifiedConeContactCost(const std::vector<coord> &path, const std::vector<const Cone *> &left_cones, const std::vector<const Cone *> &right_cones)
{
    double cost {0};
    for (auto &coord : path)
    {
        auto left_result = findClosestConeToPoint(coord, left_cones);
        auto right_result = findClosestConeToPoint(coord, right_cones);
        cost = cost + (left_result.second - right_result.second)/path.size();
    }
    return abs(cost);
}

double PathAnalysis::unclassifiedConeContactCost(const std::vector<coord> &path, const std::vector<std::unique_ptr<Cone>> &cone_list)
{
    double cost {0};
    for (auto &coord : path)
    {
        auto left_result = findClosestConeToPoint(coord, cone_list);
        cost += left_result.second;
    }
    return cost/path.size();
}

int PathAnalysis::findLowestCostPath(const std::vector<double> &cost_scores)
{
    #ifdef DEBUG
    std::stringstream ss;
    path_log->write(ss<<"List of scores:");
	int path_count{0};
	for (auto &score : cost_scores)
	{
		path_log->write(ss<<"Path "<<path_count<<" has score "<<score);
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

double PathAnalysis::classifiedConesEachSide(const std::vector<coord> &path, const std::vector<const Cone *> &left_cones, const std::vector<const Cone *> &right_cones)
{

}

double PathAnalysis::unclassifiedConesEachSide(const std::vector<coord> &path, const std::vector<std::unique_ptr<Cone>> &cone_list)
{

}

double PathAnalysis::advancingToGoal(const std::vector<coord> &path, const coord &end_goal)
{

}
