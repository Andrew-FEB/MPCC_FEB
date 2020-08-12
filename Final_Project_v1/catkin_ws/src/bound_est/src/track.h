#ifndef TRACK_H
#define TRACK_H

#include <vector>
#include <memory>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <math.h>
#include <limits>
#include <numeric>

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
double constexpr MAX_VISION_CONE_FRAME_RANGE = 25;
double constexpr CONE_VISION_ARC = 0.2;    //Value from 0 to 1 describing how much of circle around current direction can be seen
double constexpr TRACK_COMPLETE_CHECK_RADIUS = 12.5;
double constexpr MAX_TRACK_WIDTH = 3.0;
double constexpr OUTLINE_COORD_TOLERANCE = 1.0;
double constexpr DISTANCE_FROM_START_BEFORE_CHECK_FOR_NEXT_LAP = 10.0;
int constexpr NUM_POINTS_TO_CHECK_FOR_OUT_OF_BOUNDS = 5;
int constexpr NUM_CENTRELINE_COORDS_BEFORE_CHECK_TRACK_COMPLETE = 10;
int constexpr MIN_FRAMED_CONES_TO_PROCESS = 4;
double constexpr TRACK_FRAME_LENGTH = 25;
double constexpr TRACK_FRAME_WIDTH_DIV_2 = 7.0;

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
    MPC_targets getReferencePath(const std::vector<double> &distances, float boundary_compress);
    bool trackIsComplete();
    void processNextSection();
    bool carIsInsideTrack();
    int getLapsRaced();
    void checkForLap();
    enum class ConeError
    {
        valid,
        overwrite,
        outlier
    };

private:
    std::pair<ConeError, Cone *> checkConePos(const Coord &point);
    std::pair<std::vector<const Cone *>, std::vector<const Cone *>> seperateConeList(std::vector<std::unique_ptr<Cone>> &coneList);    //0 = left, 1 = right
    std::vector<std::unique_ptr<Cone>> extractConesInFrame();
    void rebalanceCones(std::vector<std::unique_ptr<Cone>> &framed_cones, std::vector<const Cone *> &boundary_cones, const unsigned long &number_to_remove);
    std::vector<Coord> interpolateCentreCoordsDiscrete(const int &original_index, const Coord &start_point, const std::vector<double> &distances);
    Pos findBoundaryPointAndSlope(const std::vector<const Cone *> &cones, const Coord &coord, const float &boundary_compress);
    Coord getClosestPointOnLine (const Coord &a, const Coord &b, const Coord &p);
    Coord findEndGoal(const Coord &last_point, const std::pair<std::vector<const Cone *>, std::vector<const Cone *>> &seperated_cone_lists);
    bool pointIsInsideTrack(const Coord &point);
    int findClosestCentreCoordIndex(const Coord &point);
    std::pair<Coord, int> getClosestPointOnCentreLine(const Coord &point);
    bool checkIfTrackComplete(const Coord &last_centre_point);
    inline std::vector<Coord> projectCarPoints(const Pos &pos);
    inline Rect projectTrackFramePoints(const Pos &pos, const double &width_div_2, const double &length);
    void removeCentreCoordOutliers();

    //Cone lists
    std::vector<std::unique_ptr<Cone>> new_cones;
    std::vector<std::unique_ptr<Cone>> processed_cone_list;
    std::vector<const Cone*> processed_cone_list_left;
    std::vector<const Cone*> processed_cone_list_right;

    //Custom objects
    std::unique_ptr<Car> car;
    std::unique_ptr<Triangulation> triangulate;
    std::unique_ptr<PathAnalysis> path_analysis;

    //Final reference path and condition indicators
    std::vector<Coord> centre_coords;
    int centre_coords_checked{0};

    //Racing metadata
    bool track_complete{false};
    int num_laps_raced{0};
    bool inside_start_zone{true};
    bool check_for_next_lap{false};

    #ifdef VISUALISE
    std::shared_ptr<Visualisation> visualisation;
    #endif
    #ifdef DEBUG
    std::unique_ptr<BoundaryLogger> boundaries_log;
    #endif

};

#endif // TRACK_H
