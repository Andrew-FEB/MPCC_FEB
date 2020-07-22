#ifndef TRACK_H
#define TRACK_H

#include <vector>
#include <memory>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <math.h>
#include <limits>

#include "cone.h"
#include "car.h"
#include "visualisation.h"
#include "triangulation.h"
#include "definitions.h"
#include "pathAnalysis.h"
#include "include/spline/spline.h"
#include "boundaryLogger.h"
#include "freeFunctions.h"

double constexpr REPEATED_CONE_RADIUS = 0.5;
double constexpr MIN_CONE_FRAME_RANGE = 0.0;
double constexpr MAX_CONE_FRAME_RANGE = 20;
double constexpr TRACK_COMPLETE_CHECK_RADIUS = 7.5;
int constexpr NUM_POINTS_TO_CHECK_FOR_OUT_OF_BOUNDS = 5;

class Track
{
public:
    Track(std::shared_ptr<Visualisation> visualisation_cont);
    ~Track() = default;
    void addCone(const double &x, const double &y, const BoundPos &pos);
    std::vector<const Cone *> getConeList();
    std::vector<const Cone *> getNewCones();
    Car *getCar();
    friend std::ostream& operator<<(std::ostream& os, Track &track);
    std::vector<MPC_targets> getReferencePath(const double &dist_between_points, const int &number_of_points);
    bool trackIsComplete();
    void processNextSection();
    bool carIsInsideTrack();
    enum class ConeError
    {
        valid,
        overwrite,
        outlier
    };

private:
    std::pair<ConeError, Cone *> checkConePos(const coord &point);
    std::pair<std::vector<const Cone *>, std::vector<const Cone *>> seperateConeList(std::vector<std::unique_ptr<Cone>> &coneList);    //0 = left, 1 = right
    void extractNewConesInRange (std::vector<std::unique_ptr<Cone>> &cones_to_extract, std::vector<std::unique_ptr<Cone>> &extracted_cones, const std::unique_ptr<Car> &car);
    std::pair<std::vector<coord>, double> interpolateCentreCoordsDiscrete(const int &original_index, const coord &start_point, const int &number_of_points, const double &distance);
    std::vector<Pos> findBoundaryPointsAndSlopes(const std::vector<const Cone *> &cone_list, const std::vector<coord> &coord_list);
    coord getClosestPointOnLine (const coord &a, const coord &b, const coord &p);
    coord findEndGoal(const coord &last_point, const std::pair<std::vector<const Cone *>, std::vector<const Cone *>> &seperated_cone_lists);
    bool pointIsInsideTrack(const coord &point);
    int findClosestCentreCoordIndex(const coord &point);
    std::pair<coord, int> getClosestPointOnCentreLine(const coord &point);
    bool checkIfTrackComplete(const coord &last_centre_point);

    //Cone lists
    std::vector<std::unique_ptr<Cone>> cones_within_range;
    std::vector<std::unique_ptr<Cone>> new_cones;
    std::vector<std::unique_ptr<Cone>> processed_cone_list;
    std::vector<const Cone*> processed_cone_list_left;
    std::vector<const Cone*> processed_cone_list_right;

    //Custom objects
    std::unique_ptr<Car> car;
    std::unique_ptr<Triangulation> triangulate;
    std::unique_ptr<PathAnalysis> path_analysis;

    //Final reference path and condition indicators
    std::vector<coord> centre_coords;
    bool track_complete{false};

    #ifdef VISUALISE
    std::shared_ptr<Visualisation> visualisation;
    #endif
    #ifdef DEBUG
    std::unique_ptr<BoundaryLogger> boundaries_log;
    #endif


};

#endif // TRACK_H
