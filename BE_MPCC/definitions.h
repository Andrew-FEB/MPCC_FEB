#pragma once

#include <unordered_map>


//Cone classifier
enum class BoundPos
{
    undefined,
    left,
    right
};

//Translate BoundPos enum to string
const static std::unordered_map<BoundPos, std::string> BoundStrTranslate
{
    {BoundPos::undefined, "undefined"},
    {BoundPos::left, "left"},
    {BoundPos::right, "right"}
};

//Convenience struct for coordinates
typedef struct coord
{
	double x;
	double y;
} coord;

//triangle struct for Triangulation
typedef struct triangle
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

		return lineOnBoundary(*first, *second);
	}
} triang;

//Convenience struct for Car dimensions
typedef struct carDims
{
	double length{ 0 };
	double width{ 0 };
} carDims;


typedef struct rect
{
	double a;
	double b;
	double c;
	double d;
}rect;