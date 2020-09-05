#pragma once

/**
 * Returns distance between two coordinates.
 * Currently applies square root despite computational load
 * If removed, some algorithms affected so testing required
 */
inline double distBetweenPoints(const Coord &a, const Coord &b)
{
	return sqrt((pow((a.x-b.x), 2) + pow((a.y-b.y),2)));
}

/**
 * Returns closest cone to a coordinate
 * when given vector of cone pointers
 * pair.first = pointer to closest cone
 * pair.second = distance to closest cone
 */
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

/**
 * Returns closest cone to a coordinate
 * when given vector of unique pointers to cones
 * pair.first = pointer to closest cone
 * pair.second = distance to closest cone
 */
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

/**
 * Returns furthest cone to a coordinate
 * when given vector of cone pointers
 * pair.first = pointer to furthest cone
 * pair.second = distance to furthest cone
 */
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

/**
 * Returns furthest cone to a coordinate
 * when given vector of unique pointers to cones
 * pair.first = pointer to furthest cone
 * pair.second = distance to furthest cone
 */
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

/**
 * Returns furthest cone to a coordinate
 * when given vector of cone pointers
 * pair.first = pointer to furthest cone
 * pair.second = index of furthest cone in provided vector
 */
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

/**
 * Returns midpoint between two coordinates as Coord
 */
inline Coord findMidpoint(const Coord &a, const Coord &b)
{
	Coord output{ (a.x + b.x) / 2, (a.y + b.y) / 2 };
	return output;
}

/**
 * Returns true when coordinate is inside circle drawn by given circle variables
 */
inline bool withinCircleOfRadius(const Coord &point_to_check, const Coord &circle_origin, const double &radius)
{
    return ((pow( (point_to_check.x-circle_origin.x) , 2) + pow( (point_to_check.y-circle_origin.y) , 2)) <= pow(radius,2));
}

/**
 * Translates point to align with angle and direction of given Pos variable
 * So for example a point could be calculated relative to the origin
 * and then translated to be relative to the given Pos
 */
inline Coord rotateToAngle(const Coord &point, const Pos &original_pos)
{
    return {(point.x*cos(original_pos.phi)-point.y*sin(original_pos.phi)+original_pos.p.x), (point.x*sin(original_pos.phi)+point.y*cos(original_pos.phi)+original_pos.p.y)};
}

/**
 * Creates circle section given origin, direction, radius of circle section and percentage of full circle it should contain
 * (0.5 = semi-circle with quarter to left of direction and quarter to right)
 */
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

/**
 * returns true if given coordinate within circle section
 */
inline bool checkIfPointInCircleSection(const CircleSection &circle_section, const Coord &point)
{
    double polar_angle = atan2(point.y-circle_section.origin.y, point.x-circle_section.origin.x);
    double polar_radius = distBetweenPoints(circle_section.origin, point);
    return (polar_angle>=circle_section.start_angle && polar_angle<=circle_section.end_angle && polar_radius<=circle_section.radius);
}