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

//Convenience struct for coordinates
struct coord
{
	double x;
	double y;

	bool operator==(const coord& a) const
	{
		return (x==a.x && y == a.y);
	}
};

//triangle struct for Triangulation
struct triang
{
	coord a;
	BoundPos aPos{BoundPos::undefined};
	coord b;
	BoundPos bPos{BoundPos::undefined};
	coord c;
	BoundPos cPos{BoundPos::undefined};

	inline bool touches(const coord &p) const
	{
		return ((a.x == p.x && a.y == p.y) 
		|| (b.x == p.x && b.y == p.y) 
		|| (c.x == p.x && c.y == p.y));
	}
	
	inline bool onVertices(const coord &p) const
	{
		return (pointsAreEqual(p, {(a.x + b.x) / 2, (a.y + b.y) / 2}) 
		||(pointsAreEqual(p, { (b.x + c.x) / 2, (b.y + c.y) / 2 })) 
		||(pointsAreEqual(p, { (c.x + a.x) / 2, (c.y + a.y) / 2 })));
	}

	inline bool pointsAreEqual(const coord &a, const coord &b) const
	{
		return (a.x == b.x && a.y == b.y);
	}

	inline bool lineOnBoundary(const BoundPos &a, const BoundPos &b) const
	{
		return (a==b);
	}

	inline coord findMidpoint(const coord &a, const coord &b)
{
	coord output{ (a.x + b.x) / 2, (a.y + b.y) / 2 };
	return output;
}

	inline bool pointsOnBoundary(const coord &x, const coord &y) const
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

//Convenience struct for Car dimensions
typedef struct carDims
{
	double length{0};
	double width{0};
} carDims;


typedef struct rect
{
	double a;
	double b;
	double c;
	double d;
} rect;

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
	coord p;
	double phi;
} Pos;

constexpr struct CarParams {
	// Vehicle parameters TODO - get real values for all these
	double Iz = 2873; // [kg m^2] moment of inertia of the vehicle ASK FOR THIS VALUE!!!!!!!
	double m = 1573;  // [kg] mass of the vehicle
	double lf = 1.35; // [m] length of the front part of the vehicle (this is from front axle to COG)
	double lr = 1.35; // length of the rear part of the vehicle  (this is from front axle to COG)
	double length = 24; // JUNK NUMBER
	double width = 12;


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

typedef struct MPC_timestep_targets
{
	coord nearest_point;
	coord goal;
	double slope;
	double slope_rads;
} MPC_targets;

inline double distBetweenPoints(const coord &a, const coord &b)
{
	return (pow((a.x-b.x), 2) + pow((a.y-b.y),2));
}