#include "pathAnalysis.h"

PathAnalysis::PathAnalysis(const Car &car_p) : car(car_p)
{
}

const std::vector<coord> &PathAnalysis::findBestPath(const std::vector<std::vector<coord>> &paths, const std::vector<std::unique_ptr<Cone>> &cone_list)
{
	std::vector<double> cost_scores;
	cost_scores.reserve(paths.size());
	int index{0};
	for (auto pathIt = paths.begin(); pathIt!=paths.end(); pathIt++)
	{
		index++;
		cost_scores[index] = findPathCost(*pathIt);
	}
    auto bestIndex = findLowestCostPath(cost_scores);	
	return paths[bestIndex];	//return path associated with lowest cost function score
}

const std::vector<coord> &PathAnalysis::findBestPath(const std::vector<std::vector<coord>> &paths, const std::vector<std::unique_ptr<Cone>> &left_cones, const std::vector<std::unique_ptr<Cone>> &right_cones)
{
    
}

double PathAnalysis::findPathCost(const std::vector<coord> &path)
{
    
}

int PathAnalysis::findLowestCostPath(const std::vector<double> &cost_scores)
{
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
