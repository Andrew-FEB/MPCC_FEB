#pragma once

#include <vector>
#include <memory>


#include "definitions.h"
#include "car.h"
#include "cone.h"
#include "freeFunctions.h"
#include "boundaryLogger.h"
#include "boundGlobals.h"
#include "visualisation.h"

double constexpr UNCLASS_NEAREST_CONE_COEFF{10.0};
double constexpr CLASS_NEAREST_CONE_COEFF{10.0};
double constexpr UNCLASS_CONES_EACH_SIDE_COEFF{10.0};
double constexpr CLASS_CONES_SIDE_COEFF{10.0};
double constexpr ADVANCING_COEFF{10.0};


//Cone contact sizes   
double constexpr RECT_CHECK_WIDTH_DIV_2 {1.5};  
double constexpr RECT_CHECK_LENGTH_DIV_2 {6}; //5/2

//Punishment constants
double constexpr NODE_REPEAT_COEFF{50.0};
double constexpr NO_CONES_SPOTTED_COEFF {20.0};
double constexpr CONE_IMBALANCE_COEFF {20.0};

//Coefficients
double constexpr DISTANCE_END_COEFF{10.0};


class PathAnalysis
{
    public: 
        PathAnalysis(std::shared_ptr<Visualisation> visualisation_cont, const Car *car);
        ~PathAnalysis() = default;
        std::vector<Coord> findBestPath(const std::vector<std::vector<Coord>> &paths, const Coord &end_goal, const std::vector<std::unique_ptr<Cone>> &cone_list, const std::vector<const Cone *> &left_cones, const std::vector<const Cone *> &right_cones);

    private: 
        double findPathCost(const std::vector<Coord> &path, const Coord &end_goal, const std::vector<std::unique_ptr<Cone>> &cone_list, const std::vector<const Cone *> &left_cones, const std::vector<const Cone *> &right_cones);
        int findLowestCostPath(const std::vector<double> &cost_scores);
        double classifiedConeContactCost(const std::vector<Coord> &path, const std::vector<const Cone *> &left_cones, const std::vector<const Cone *> &right_cones);
        double unclassifiedConeContactCost(const std::vector<Coord> &path, const std::vector<std::unique_ptr<Cone>> &cone_list);
        double classifiedConesEachSide(const std::vector<Coord> &path, const std::vector<const Cone *> &left_cones, const std::vector<const Cone *> &right_cones);
        double unclassifiedConesEachSide(const std::vector<Coord> &path, const std::vector<std::unique_ptr<Cone>> &cone_list);
        double advancingToGoal(const std::vector<Coord> &path, const Coord &end_goal);
        double checkForRepeatNodes(const std::vector<Coord> &path);

        #ifdef DEBUG
        std::unique_ptr<BoundaryLogger> path_log;
        # endif
        #ifdef VISUALISE
        std::shared_ptr<Visualisation> visualisation{ nullptr };
        #endif
        const Car *car;
};