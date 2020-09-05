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

/**
 * Coefficients for adjusting scaling of different cost parameters
 */
double constexpr UNCLASS_NEAREST_CONE_COEFF{10.0};
double constexpr CLASS_NEAREST_CONE_COEFF{10.0};
double constexpr UNCLASS_CONES_EACH_SIDE_COEFF{10.0};
double constexpr CLASS_CONES_EACH_SIDE_COEFF{10.0};
double constexpr ADVANCING_COEFF{10.0};
double constexpr DISTANCE_END_COEFF{10.0};

//Punishment constants
double constexpr NODE_REPEAT_COEFF{50.0};   //Cost added if node repeated in path
double constexpr NO_CONES_SPOTTED_COEFF {20.0}; //Cost added if no cones spotted in cone check
double constexpr CONE_IMBALANCE_COEFF {20.0};   //Cost added for each extra cone on one side versus the other side


//Cone contact vision rectangle sizes   
double constexpr RECT_CHECK_WIDTH_DIV_2 {1.5};  //Width of rectangle to check for cones around car for path checks
double constexpr RECT_CHECK_LENGTH_DIV_2 {6};   //Height of rectangle to check for cones around car for path checks





class PathAnalysis
{
    public: 
        PathAnalysis(std::shared_ptr<Visualisation> visualisation_cont, const Car *car);
        ~PathAnalysis() = default;
        /**
         * returns the vector of coordinates with the lowest overall cost according to all checks done by pathAnalysis.
         * Provide vector of vectors of coordinates
         */
        std::vector<Coord> findBestPath(const std::vector<std::vector<Coord>> &paths, const Coord &end_goal, const std::vector<std::unique_ptr<Cone>> &cone_list, const std::vector<const Cone *> &left_cones, const std::vector<const Cone *> &right_cones);

    private: 
        //Applies all cost calculation functions to given path
        double findPathCost(const std::vector<Coord> &path, const Coord &end_goal, const std::vector<std::unique_ptr<Cone>> &cone_list, const std::vector<const Cone *> &left_cones, const std::vector<const Cone *> &right_cones);
        //returns index of lowest cost path
        int findLowestCostPath(const std::vector<double> &cost_scores);
        /**
         * For each point in path, add the square of the difference
         * between distance to nearest left cone and nearest right cone
         */
        double classifiedConeContactCost(const std::vector<Coord> &path, const std::vector<const Cone *> &left_cones, const std::vector<const Cone *> &right_cones);
        /**
         * For each point in path, add the square of the difference
         * between distance to nearest cone at point n and nearest 
         * cone at point n-1
         */
        double unclassifiedConeContactCost(const std::vector<Coord> &path, const std::vector<std::unique_ptr<Cone>> &cone_list);
        /**
         * For each point in path, draw rectangle around car
         * (sizes of rectangle determined by RECT_CHECK_WIDTH_DIV_2
         * and RECT_CHECK_LENGTH_DIV_2) and compare number of left
         * boundary cones seen vs right boundary cones
         */
        double classifiedConesEachSide(const std::vector<Coord> &path, const std::vector<const Cone *> &left_cones, const std::vector<const Cone *> &right_cones);
         /**
         * For each point in path, draw rectangle around car
         * (sizes of rectangle determined by RECT_CHECK_WIDTH_DIV_2
         * and RECT_CHECK_LENGTH_DIV_2) and compare number of boundary
         * cones seen at point n vs point n-1
         */
        double unclassifiedConesEachSide(const std::vector<Coord> &path, const std::vector<std::unique_ptr<Cone>> &cone_list);
        /**
         * For each point in path, check if closer to end of section than
         * at point n-1. If further, add square of difference in distances to
         * cost
         */
        double advancingToGoal(const std::vector<Coord> &path, const Coord &end_goal);
        /**
         * Add fixed cost for every repeated point in path
         */
        double checkForRepeatNodes(const std::vector<Coord> &path);

        #ifdef DEBUG
        std::unique_ptr<BoundaryLogger> path_log;
        # endif
        #ifdef VISUALISE
        std::shared_ptr<Visualisation> visualisation{ nullptr };
        #endif
        const Car *car;
};