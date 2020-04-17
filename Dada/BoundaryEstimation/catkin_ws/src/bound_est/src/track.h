#ifndef TRACK_H
#define TRACK_H

#include <vector>
#include <memory>
#include <iostream>
#include <algorithm>
#include <cmath>

#include "cone.h"
#include "car.h"
#include "visualisation.h"
#include "triangulation.h"
#include "definitions.h"
#include "include/spline/spline.h"

double constexpr minRadiusSquared = 2.0; //0.707^2
double constexpr maxRadiusSquared = 1000000000.0; //3162^2

class Track
{
public:
    Track(std::shared_ptr<Visualisation> visualise_cont);
    ~Track() = default;
    void addCone(const double &x, const double &y, const BoundPos &pos);
    std::vector<const Cone *> getConeList();
    Car *getCar();
    friend std::ostream& operator<<(std::ostream& os, Track &track);
    MPC_targets getCentreLine(const double &desired_dist);
    bool checkForCollision(const coord &carPos, const double &carDirection);

    enum class ConeError
    {
        valid,
        overwrite,
        outlier
    };

private:
    std::pair<ConeError, Cone *> checkConePos(const double &x, const double &y);
    void processTrackSect();
    void recalcSpline();
    bool checkShapeOverlap();

    std::vector<std::unique_ptr<Cone>> coneList;
    std::unique_ptr<Car> car;
    std::unique_ptr<Triangulation> triangulate;
    std::shared_ptr<Visualisation> visualise;
    std::vector<coord> centreCoords;
};

#endif // TRACK_H
