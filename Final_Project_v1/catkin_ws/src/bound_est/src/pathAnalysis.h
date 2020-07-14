#pragma once

#include <vector>
#include <memory>


#include "definitions.h"
#include "car.h"
#include "cone.h"

double constexpr NEAREST_CONE_COEFF{1.0};
double constexpr DISTANCE_TO_END{1.0};


class PathAnalysis
{
    public: 
        PathAnalysis(const Car &car);
        ~PathAnalysis() = default;
        const std::vector<coord> &findBestPath(const std::vector<std::vector<coord>> &paths, const std::vector<std::unique_ptr<Cone>> &left_cones, const std::vector<std::unique_ptr<Cone>> &right_cones);
        const std::vector<coord> &findBestPath(const std::vector<std::vector<coord>> &paths, const std::vector<std::unique_ptr<Cone>> &cone_list);

    private: 
        double findPathCost(const std::vector<coord> &path);
        int findLowestCostPath(const std::vector<double> &cost_scores);
        const Car &car;
};