#pragma once

#include <vector>
#include <memory>
#include <cmath>

#include "cone.h"
#include "visualisation.h"
#include "definitions.h"
#include "tree.h"
#include "boundaryLogger.h"
#include "freeFunctions.h"

/**
 * Tolerance in metres for best-first checks when placing nodes
 * Example: If potential node is 2.5m FURTHER AWAY from section exit THAN parent node
 * and BEST_FIRST_TOLERANCE = 3, then potential node ACCEPTED.
 * If BEST_FIRST_TOLERANCE = 2, potential node REFUSED
 */
constexpr double BEST_FIRST_TOLERANCE {3};	

class Triangulation
{

public:
	Triangulation(std::shared_ptr<Visualisation> visualisation_cont);
	~Triangulation() = default;
	/**
	 * Returns vector of all paths inside track section bounded by cone_list and seperated cone_lists.
	 * Cone list and seperated cone_lists should consist of THE SAME CONES. THIS IS NOT CHECKED.
	 * entry_point = start coordinate for paths traversing track section.
	 * exit_point = end coordinate for paths traversing track section.
	 * max_path_length = maximum number of nodes inside paths delivered.
	 */
	std::vector<std::vector<Coord>> getTraversingPaths(const std::vector<std::unique_ptr<Cone>>& cone_list, const std::pair<std::vector<const Cone *>, std::vector<const Cone *>> &seperated_cone_lists,
															const Coord &entry_point, const Coord &exit_point, int max_path_length);
  
private:
	//Uses delaunator library to apply delaunay triangulation to cone positions and section entry point. Returns vector of triangle structs lacking boundary information
	std::vector<Triang> findTrianglePoints(const std::vector<std::unique_ptr<Cone>>& coneList, const Coord &entry_point);
	//Cross-checks cone list and triangle_list to update triangle_list with BoundPos information
	void placeConesInTriangle(std::vector<Triang> &triangle_list, const std::vector<std::unique_ptr<Cone>> &coneList);
	//Returns true if (previous_distance_from_goal+tolerance)>distance_from_goal
	bool movingToGoal(const double &distance_from_goal, const double &tolerance, const double &previous_distance_from_goal);
	/**
	 * Creates tree, finds viable nodes in triangle list and adds to tree until all viable nodes have been added. Returns all paths delivered by tree.
	 * entry_point = start coordinate for paths traversing track section.
	 * exit_point = end coordinate for paths traversing track section.
	 * triangle_list = list of triangles spanning track section
	 * max_path_length = maximum number of nodes inside paths delivered.
	 */
	std::vector<std::vector<Coord>> findViablePaths(const Coord &entry_point, const Coord &exit_pont, std::vector<Triang> &triangle_list, int max_path_length);
	/**
	 * Returns vector of triangles touching point. 
	 * If is_midpoint true, searches for all triangles that have point as a middle point of a line section
	 * If is_midpoint false, searches for all triangles that have point as a vertex
	 */
	std::vector<Triang> findTrianglesAroundPoint(bool is_midpoint, const std::vector<Triang> &triangle_list, const Coord &point);
	/**
	 * Returns first traversable position in section for special case where current position is a vertex in the triangulation.
	 * In all other cases current position is line section midpoint
	 */
	Coord findFirstMidpoint(std::vector<Triang> &vec, const Coord &entry_point);
	/**
	 * Returns list of all coordinates at line section midpoints inside triangles
	 * Current_position = current position in search, to prevent it being re-added to search
	 */
	std::vector<Coord> findMidsInTriangVec(std::vector<Triang> &triangles, const Coord &current_position);

	std::shared_ptr<Visualisation> visualisation{ nullptr };
};

std::ostream& operator<<(std::ostream& os, std::vector<Triang> &triangle_list);
std::ostream& operator<<(std::ostream& os, std::vector<Coord> &vec);
std::ostream& operator<<(std::ostream& os, Tree &tree);
