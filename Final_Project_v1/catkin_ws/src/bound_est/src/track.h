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

double constexpr minRadiusSquared = 2.0; //0.707^2
double constexpr MIN_CONE_FRAME_RANGE = 0.0;
double constexpr MAX_CONE_FRAME_RANGE = 225.0;  //15²

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
    MPC_targets getCentreLine(const double &desired_dist);
    bool checkForCollision(const coord &carPos, const double &carDirection);
    void processNextSection();
    enum class ConeError
    {
        valid,
        overwrite,
        outlier
    };

private:
    coord getNearestPointOnArc(const coord &point);  //TODO
    std::pair<ConeError, Cone *> checkConePos(const double &x, const double &y);
    void recalcSpline();
    bool checkShapeOverlap();
    std::pair<std::vector<const Cone *>, std::vector<const Cone *>> seperateConeList(std::vector<std::unique_ptr<Cone>> &coneList);    //0 = left, 1 = right
    void extractNewConesInRange (std::vector<std::unique_ptr<Cone>> &cones_to_extract, std::vector<std::unique_ptr<Cone>> &extracted_cones, const std::unique_ptr<Car> &car);

    std::vector<std::unique_ptr<Cone>> processed_cone_list;
    std::vector<const Cone*> processed_cone_list_left;
    std::vector<const Cone*> processed_cone_list_right;
    std::vector<std::unique_ptr<Cone>> new_cones;
    std::unique_ptr<Car> car;
    std::unique_ptr<Triangulation> triangulate;
    #ifdef VISUALISE
    std::shared_ptr<Visualisation> visualisation;
    #endif
    std::unique_ptr<PathAnalysis> path_analysis;
    std::vector<coord> centre_coords;
};

#endif // TRACK_H
