#pragma once

inline double distBetweenPoints(const Coord &a, const Coord &b)
{
	return sqrt((pow((a.x-b.x), 2) + pow((a.y-b.y),2)));
}

inline std::pair<const Cone *, double> findClosestConeToPoint (const Coord &point, const std::vector<const Cone *> &cones)
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

inline std::pair<const Cone *, double> findClosestConeToPoint (const Coord &point, const std::vector<std::unique_ptr<Cone>> &cones)
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

inline std::pair<const Cone *, double> findFurthestConeFromPoint (const Coord &point, const std::vector<const Cone *> &cones)
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

inline std::pair<const Cone *, double> findFurthestConeFromPoint (const Coord &point, std::vector<std::unique_ptr<Cone>> &cones)
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

inline Coord findMidpoint(const Coord &a, const Coord &b)
{
	Coord output{ (a.x + b.x) / 2, (a.y + b.y) / 2 };
	return output;
}

inline bool withinCircleOfRadius(const Coord &point_to_check, const Coord &circle_origin, const double &radius)
{
    return ((pow( (point_to_check.x-circle_origin.x) , 2) + pow( (point_to_check.y-circle_origin.y) , 2)) <= radius);
}

inline Coord rotateToAngle(const Coord &point, const Pos &original_pos)
{
    return {(point.x*cos(original_pos.phi)-point.y*sin(original_pos.phi)+original_pos.p.x), (point.x*sin(original_pos.phi)+point.y*cos(original_pos.phi)+original_pos.p.y)};
}