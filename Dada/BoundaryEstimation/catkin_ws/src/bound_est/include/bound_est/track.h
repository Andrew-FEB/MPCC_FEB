#ifndef TRACK_H
#define TRACK_H

#include <vector>
#include <memory>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <optional>

#include "cone.h"
#include "car.h"
#include "visualise.h"
#include "triangulation.h"
#include "definitions.h"

double constexpr minRadiusSquared = 0.5; //0.707^2
double constexpr maxRadiusSquared = 400.0; //20^2

class Track
{
public:
    Track(std::optional<std::shared_ptr<Visualise>> visualise_cont);
    ~Track() = default;
    void addCone(const double &x, const double &y, const BoundPos &pos);
    void updateCar(const double &x, const double &y);
    const carDims &getCarDims(); 
    std::vector<const Cone *> getConeList();
    friend std::ostream& operator<<(std::ostream& os, Track &track);


    inline double squaref(const double &i)    //gcc not compliant - no powf. If different compiler used, replace inline function
    {
        return (i*i);
    }
    enum class ConeError
    {
        valid,
        overwrite,
        outlier
    };

private:
    std::pair<ConeError, std::optional<Cone *>> checkConePos(const double &x, const double &y);
    void processTrackSect();

    std::vector<std::unique_ptr<Cone>> coneList;
    std::unique_ptr<Car> car{ nullptr };
    std::unique_ptr<Triangulation> triangulate{ nullptr };
        
    std::shared_ptr<Visualise> visualise{ nullptr };
};

#endif // TRACK_H
