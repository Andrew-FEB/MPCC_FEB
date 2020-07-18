#pragma once

#include <vector>
#include <memory>


#include "definitions.h"
#include "car.h"
#include "cone.h"
#include "freeFunctions.h"
#include "boundaryLogger.h"
#include "boundGlobals.h"

double constexpr UNCLASS_NEAREST_CONE_COEFF{10.0};
double constexpr CLASS_NEAREST_CONE_COEFF{10.0};
double constexpr DISTANCE_END_COEFF{1.0};
double constexpr UNCLASS_CONES_EACH_SIDE_COEFF{10.0};
double constexpr CLASS_CONES_SIDE_COEFF{10.0};
double constexpr ADVANCING_COEFF{10.0};


class PathAnalysis
{
    public: 
        PathAnalysis();
        ~PathAnalysis() = default;
        const std::vector<coord> &findBestPath(const std::vector<std::vector<coord>> &paths, const coord &end_goal, const std::vector<std::unique_ptr<Cone>> &cone_list, const std::vector<const Cone *> &left_cones, const std::vector<const Cone *> &right_cones);

    private: 
        double findPathCost(const std::vector<coord> &path, const coord &end_goal, const std::vector<std::unique_ptr<Cone>> &cone_list, const std::vector<const Cone *> &left_cones, const std::vector<const Cone *> &right_cones);
        int findLowestCostPath(const std::vector<double> &cost_scores);
        double classifiedConeContactCost(const std::vector<coord> &path, const std::vector<const Cone *> &left_cones, const std::vector<const Cone *> &right_cones);
        double unclassifiedConeContactCost(const std::vector<coord> &path, const std::vector<std::unique_ptr<Cone>> &cone_list);
        double classifiedConesEachSide(const std::vector<coord> &path, const std::vector<const Cone *> &left_cones, const std::vector<const Cone *> &right_cones);
        double unclassifiedConesEachSide(const std::vector<coord> &path, const std::vector<std::unique_ptr<Cone>> &cone_list);
        double advancingToGoal(const std::vector<coord> &path, const coord &end_goal);


        #ifdef DEBUG
        std::unique_ptr<BoundaryLogger> path_log;
        #endif
};