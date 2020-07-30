#pragma once

#include <unordered_map>
#include <stdint.h>
#include <cmath>

//Cone classifier
enum class BoundPos
{
    undefined,
    left,
    right,
	end,
	offramp
};

//Translate BoundPos enum to string
const static std::unordered_map<BoundPos, std::string> BoundStrTranslate
{
    {BoundPos::undefined, "undefined"},
    {BoundPos::left, "left"},	//small blue
    {BoundPos::right, "right"},	//small yellow
	{BoundPos::end, "Start/End"},	//Big orange
	{BoundPos::offramp, "Offramp"}	//small orange
};

//Convenience struct for Coordinates
struct Coord
{
	double x;
	double y;

	bool operator==(const Coord& a) const
	{
		return (x==a.x && y == a.y);
	}
};

//triangle struct for Triangulation
struct Triang
{
	Coord a;
	BoundPos aPos{BoundPos::undefined};
	Coord b;
	BoundPos bPos{BoundPos::undefined};
	Coord c;
	BoundPos cPos{BoundPos::undefined};

	inline bool touches(const Coord &p) const
	{
		return ((a.x == p.x && a.y == p.y) 
		|| (b.x == p.x && b.y == p.y) 
		|| (c.x == p.x && c.y == p.y));
	}
	
	inline bool onVertices(const Coord &p) const
	{
		return (pointsAreEqual(p, {(a.x + b.x) / 2, (a.y + b.y) / 2}) 
		||(pointsAreEqual(p, { (b.x + c.x) / 2, (b.y + c.y) / 2 })) 
		||(pointsAreEqual(p, { (c.x + a.x) / 2, (c.y + a.y) / 2 })));
	}

	inline bool pointsAreEqual(const Coord &a, const Coord &b) const
	{
		return (a.x == b.x && a.y == b.y);
	}

	inline bool lineOnBoundary(const BoundPos &a, const BoundPos &b) const
	{
		return (a==b);
	}

	inline Coord findMidpoint(const Coord &a, const Coord &b)
{
	Coord output{ (a.x + b.x) / 2, (a.y + b.y) / 2 };
	return output;
}

	inline bool pointsOnBoundary(const Coord &x, const Coord &y) const
	{
		const BoundPos *first;
		const BoundPos *second;
		//Cleaner method??
		if (pointsAreEqual(x, a)) first = &aPos;
		else if (pointsAreEqual(x, b)) first = &bPos;
		else first = &cPos;

		if (pointsAreEqual(y, a)) second = &aPos;
		else if (pointsAreEqual(y, b)) second = &bPos;
		else second = &cPos;

		return ((*first)==(*second));
	}
};

struct Rect
{
	std::array<Coord, 4> points;

	inline double dotProductThreePoints(Coord origin, Coord b, Coord c)
	{
		Coord vec_1 = {b.x-origin.x, b.y-origin.y};
		Coord vec_2 = {c.x-origin.x, c.y-origin.y};
		return (vec_1.x*vec_2.x+vec_1.y*vec_2.y);
	}

	inline bool containsPoint(const Coord &p)
	{
		auto dot_ab_ap = dotProductThreePoints(points[0], points[1], p);
		auto dot_ab_ab = dotProductThreePoints(points[0], points[1], points[1]);
		auto dot_bc_bp = dotProductThreePoints(points[1], points[2], p);
		auto dot_bc_bc = dotProductThreePoints(points[1], points[2], points[2]);
		return (0<=dot_ab_ap && dot_ab_ap<=dot_ab_ab && 0<=dot_bc_bp && dot_bc_bp<=dot_bc_bc);
	}
};

/**
 * Structs regarding MPCC
 */
typedef struct TireForces {
    double Ffy;
    double Frx;
    double Fry;
} TireForces;

typedef struct TrackContraints {
    double centreSlope;
	double centreX;
	double centreY;
    double upSlope;
	double upX;
	double upY;
    double lowSlope;
	double lowX;
	double lowY;
} TrackContraints;

typedef struct ControlInputs {
	double D; // throttle [-1, 1]
	double delta; // steering angle [-0.506, 0.506]
} ControlInputs;

typedef struct Vel {
	double vx;
	double vy;
	double omega;
} Vel;

typedef struct Pos {
	Coord p;
	double phi;
} Pos;

constexpr struct CarParams {
	// Vehicle parameters TODO - get real values for all these
	double Iz = 2873; // [kg m^2] moment of inertia of the vehicle ASK FOR THIS VALUE!!!!!!!
	double m = 1573;  // [kg] mass of the vehicle
	double lf = 1.35; // [m] length of the front part of the vehicle (this is from front axle to COG)
	double lr = 1.35; // length of the rear part of the vehicle  (this is from front axle to COG)
	double length_div_2 = 0.6; // JUNK NUMBER
	double width_div_2 = 0.2;


	double weightF = lr/(lf + lr);
	double weightR = lf/(lf + lr);

	// TODO Tire Parameters (TBD) - THESE STILL NEED TO BE CHANGED TO THE ONES CORRESPONDING TO THE FORMULA
	double Cm1 = 17303; // Motor Model
	double Cm2 = 175;  	// Motor Model
	double Crr = 120;  	// Rolling Resistance
	double Cd = 0.5 * 1.225 * 0.35 * 2.5;  // Drag

	// Tire Force Curve
	// Rear tires
	double Br = 13;
	double Cr = 2;
	double Dr = weightF * m * 9.81 * 1.2;

	// Front tires
	double Bf = 13;
	double Cf = 2;
	double Df = weightR * m * 9.81 * 1.2;

	// Friction ellipse
	double p_long = 0.9;
	double p_ellipse = 0.95;
} CarParams;

struct MPC_targets
{
	Coord reference_point;
	Pos left_boundary;
	Pos right_boundary;
};

struct CircleSection
{
	Coord origin;
	double radius;
	double start_angle;
	double end_angle;
};


