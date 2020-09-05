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

//checkConePos()
double constexpr REPEATED_CONE_RADIUS = 0.5;    //Radius around existing cone where new cone is considered to be a repeat

//NOT CURRENTLY IN USE. Left for example
//extractConesInFrame()
//Circle section
double constexpr MAX_VISION_CONE_FRAME_RANGE = 25;  //Value indicating radius of circle section around car 
double constexpr CONE_VISION_ARC = 0.2;    //Value from 0 to 1 describing how much of circle around current direction can be seen
//Rectangle
double constexpr TRACK_FRAME_LENGTH = 25;   //Value indicating length of vision rectangle projected in front of car
double constexpr TRACK_FRAME_WIDTH_DIV_2 = 7.0; //Value indicating width of vision rectangle projected in front of car

//checkTrackComplete()
double constexpr TRACK_COMPLETE_CHECK_RADIUS = 12.5;    //Radius around first centre line point checked to see if final centre line point is completing track
int constexpr NUM_POINTS_TO_CHECK_FOR_OUT_OF_BOUNDS = 5;    //Number of points to check if within point

//checkForLap()
double constexpr CHECK_LAP_RADIUS = 6.0; //Used as radius to check if car is crossing start point and lap completed
double constexpr DISTANCE_FROM_START_BEFORE_CHECK_FOR_NEXT_LAP = 15.0;  //Used as radius to indicate distance before lap checks may resume (to prevent multiple triggers)

//removeCentreCoordOutliers()
double constexpr OUTLINE_COORD_TOLERANCE = 1.0; //How strong a direction shift the function will accept before treating a centre coord as erroneous

//processNextSection()
int constexpr NUM_CENTRELINE_COORDS_BEFORE_CHECK_TRACK_COMPLETE = 10;   //Number of centre line coords required before track complete checks start
int constexpr MIN_FRAMED_CONES_TO_PROCESS = 4;  //Minimum number of cones in frame before section will be processed


class Track
{
public:
    Track(std::shared_ptr<Visualisation> visualisation_cont);
    ~Track() = default;
    //Add cone at position x and y to track. If track boundary known, specify as pos
    void addCone(const double &x, const double &y, const BoundPos &pos);
    std::vector<const Cone *> getConeList();
    std::vector<const Cone *> getNewCones();
    Car *getCar();
    friend std::ostream& operator<<(std::ostream& os, Track &track);
    /**
     * Request reference path for MPC calculations
     * Provide vector of distances between each point along reference path. Number of distances = number of points
     * Boundary compress will shift boundaries towards centre line. 1.0 = normal, boundary_compress<1.0 = shifted inwards.
     */
    MPC_targets getReferencePath(const std::vector<double> &distances, float boundary_compress);
    //returns true if track fully mapped
    bool trackIsComplete();
    //Attempt to process next frame of track, if enough suitable cones available, to add to known centre line. path_limit_length = max number of nodes in paths through track sections
    void processNextSection(int path_length_limit = 30);
    //returns true if car is currently within track boundaries
    bool carIsInsideTrack();
    //returns number of laps completed so far
    int getLapsRaced();
    //Checks if laps completed needs to be updated
    void checkForLap();
    enum class ConeError
    {
        valid,
        overwrite
    };

private:
    //Checks if cone has already been added to track. Returns ConeError::overwrite if yes and overwrites existing cone, otherwise returns ConeError::valid and adds to track
    std::pair<ConeError, Cone *> checkConePos(const Coord &point);
    /**
     * Creates list of pointers to cones on left boundary and on right boundary, attempting to estimate boundary where not provided at input.
     * Returns pair, where pair.first = left boundary and pair.second = left boundary
     */
    std::pair<std::vector<const Cone *>, std::vector<const Cone *>> seperateConeList(std::vector<std::unique_ptr<Cone>> &coneList);
    /**
     * Currently not in use. Shows example code for an approach to extracting cones from new cones
     * that satisfy a condition. Can be used to limit number of cones processed in a frame
     * using distance checks, overlaying an area against the track area, etc.
     */
    std::vector<std::unique_ptr<Cone>> extractConesInFrame();
    /**
     * Iteratively removes cones from framed cones that satisfy conditions 1: furthest from car's current position and  2: present in boundary cones
     * until cones removed = number_to_remove. Removed cones removed both from framed_cones and boundary_cones. Unique pointers returned to new cones
     * to be used in future frames
     */
    void rebalanceCones(std::vector<std::unique_ptr<Cone>> &framed_cones, std::vector<const Cone *> &boundary_cones, const unsigned long &number_to_remove);
    /**
     * Returns number of coordinates equal to distances.size() that are interpolated along centre coordinate line, starting from start_point
     * and with original index = index of centre line point placed before start_point
     */
    std::vector<Coord> interpolateCentreCoordsDiscrete(const int &original_index, const Coord &start_point, const std::vector<double> &distances);
    /**
     * Returns closest point to point between nearest centre line coordinate and centre line coordinate AFTER it
     * Example = car at point (X, Y). Nearest centre coordinate at index n with position (a, b). Nearest point to position (X, Y) on line between
     * centre coordinate at index n and index n+1 returned. If no index n+1 exists, n-1 used. 
     */
    std::pair<Coord, int> getClosestPointOnCentreLine(const Coord &point);
    /** 
     * Returns a Pos struct with nearest point to coord on line described by connecting all cone locations linearly, and slope of boundary
     * at that point. Boundary compress will shift point towards coord, with 1.0 = no shift and 0.5 = shifted to half the distance
     */
    Pos findBoundaryPointAndSlope(const std::vector<const Cone *> &cones, const Coord &coord, const float &boundary_compress);
    //Finds nearest point on line a-b to p and returns as coordinate
    Coord getClosestPointOnLine (const Coord &a, const Coord &b, const Coord &p);
    /**
     * Attempts to find furthest point in a track section described by two boundaries marked by the left and right boundaries of seperated_cone_lists
     * Will first try to find the furthest cone from last_point in each boundary and take the mid-point between them. If the furthest cones on each boundary
     * are not found to be closest to one another, the overall furthest cone and its closest cone on the nearest boundary will instead be used
     */
    Coord findEndGoal(const Coord &last_point, const std::pair<std::vector<const Cone *>, std::vector<const Cone *>> &seperated_cone_lists);
    /**
     * Check if point inside track. Done by finding nearest point on centre line to point, then finding nearest cone to centre line point. 
     * They are used to draw a circle, and point is found to be outside of track if it is not within that circle
     */
    bool pointIsInsideTrack(const Coord &point);
    //returns index of closest point on centre line to point
    int findClosestCentreCoordIndex(const Coord &point);
    //returns true if track complete checks satisfied and mapping can be deactivated
    bool checkIfTrackComplete(const Coord &last_centre_point);
    //Projects points to the four corners of the car according to car measurements in CarParams (definitions.h)
    inline std::vector<Coord> projectCarPoints(const Pos &pos);
    //Projects points in front of car to act as track vision frame (used in extractConesInFrame())
    inline Rect projectTrackFramePoints(const Pos &pos, const double &width_div_2, const double &length);
    /**
     * Attempts to remove erroneous centre coordinates from list by checking for sharp shift in direction of one coordinate
     * relative to all other directions measured from neighbouring coordinates
     */
    void removeCentreCoordOutliers();

    //Cone lists
    std::vector<std::unique_ptr<Cone>> new_cones;   //Cones yet to be processed
    std::vector<std::unique_ptr<Cone>> processed_cone_list; //Cones that have been processed as part of a frame already
    std::vector<const Cone*> processed_cone_list_left;  //Left boundary cones that have been processed as part of a frame already
    std::vector<const Cone*> processed_cone_list_right; //Right boundary cones that have been processed as part of a frame already

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
