#pragma once

inline double distBetweenPoints(const coord &a, const coord &b)
{
	return sqrt((pow((a.x-b.x), 2) + pow((a.y-b.y),2)));
}

inline std::pair<const Cone *, double> findClosestConeToPoint (const coord &point, const std::vector<const Cone *> &cones)
{
	std::pair<const Cone *, double> output = std::make_pair(nullptr, std::numeric_limits<double>::max());
    for (auto &cone_p : cones)
    {
        auto dist = distBetweenPoints(point, cone_p->getCoordinates());
        if (dist<output.second)
        {
            output.second = dist;
            output.first = cone_p;
        }
    }
    return output;
}

inline std::pair<const Cone *, double> findClosestConeToPoint (const coord &point, const std::vector<std::unique_ptr<Cone>> &cones)
{
	std::pair<const Cone *, double> output = std::make_pair(nullptr, std::numeric_limits<double>::max());
    for (auto &cone : cones)
    {
        auto dist = distBetweenPoints(point, cone->getCoordinates());
        if (dist<output.second)
        {
            output.second = dist;
            output.first = cone.get();
        }
    }
    return output;
}

inline std::pair<const Cone *, double> findFurthestConeFromPoint (const coord &point, const std::vector<const Cone *> &cones)
{
	std::pair<const Cone *, double> output = std::make_pair(nullptr, std::numeric_limits<double>::min());
    for (auto &cone_p : cones)
    {
        auto dist = distBetweenPoints(point, cone_p->getCoordinates());
        if (dist>output.second)
        {
            output.second = dist;
            output.first = cone_p;
        }
    }
    return output;
}

inline std::pair<const Cone *, double> findFurthestConeFromPoint (const coord &point, std::vector<std::unique_ptr<Cone>> &cones)
{
	std::pair<const Cone *, double> output = std::make_pair(nullptr, std::numeric_limits<double>::min());
    for (auto &cone : cones)
    {
        auto dist = distBetweenPoints(point, cone->getCoordinates());
        if (dist>output.second)
        {
            output.second = dist;
            output.first = cone.get();
        }
    }
    return output;
}

inline coord findMidpoint(const coord &a, const coord &b)
{
	coord output{ (a.x + b.x) / 2, (a.y + b.y) / 2 };
	return output;
}
