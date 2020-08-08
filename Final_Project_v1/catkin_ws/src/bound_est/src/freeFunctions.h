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
	std::pair<const Cone *, double> output = std::make_pair(nullptr, std::numeric_limits<double>::lowest());
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
	std::pair<const Cone *, double> output = std::make_pair(nullptr, std::numeric_limits<double>::lowest());
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


inline std::pair<const Cone *, int> findFurthestConeFromPointWithIndex (const Coord &point, const std::vector<const Cone *> &cones)
{
	std::pair<const Cone *, int> output = std::make_pair(nullptr, -1);
    double furthest_dist = std::numeric_limits<double>::lowest();
    for (int i = 0; i<cones.size(); i++)
    {
        auto dist = distBetweenPoints(point, cones[i]->getCoordinates());
        if (dist>furthest_dist)
        {
            furthest_dist = dist;
            output.first = cones[i];
            output.second = i;
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
    return ((pow( (point_to_check.x-circle_origin.x) , 2) + pow( (point_to_check.y-circle_origin.y) , 2)) <= pow(radius,2));
}

inline Coord rotateToAngle(const Coord &point, const Pos &original_pos)
{
    return {(point.x*cos(original_pos.phi)-point.y*sin(original_pos.phi)+original_pos.p.x), (point.x*sin(original_pos.phi)+point.y*cos(original_pos.phi)+original_pos.p.y)};
}

inline CircleSection formCircleSection(const Coord &origin, const double &direction, const double &radius, const double &percentage)
{
    CircleSection sec;
    sec.origin = origin;
    double arc_change = M_PI*percentage; //Not 2*M_PI as we want half the angle to either side
    sec.start_angle = direction-arc_change;
    sec.end_angle = direction+arc_change;
    sec.radius = radius;
    return sec;
}

inline bool checkIfPointInCircleSection(const CircleSection &circle_section, const Coord &point)
{
    double polar_angle = atan2(point.y-circle_section.origin.y, point.x-circle_section.origin.x);
    double polar_radius = distBetweenPoints(circle_section.origin, point);
    return (polar_angle>=circle_section.start_angle && polar_angle<=circle_section.end_angle && polar_radius<=circle_section.radius);
}