#include "mip.h"

namespace cubes {
	Pos cube1;
	Pos cube2;
	Pos cube3;
	Pos cube4;

	Pos startPos(150, 150, 0);
	
	void goStart() {

	}
}

struct Pos
{
	int x;
	int y;
	float Fi;

	Pos(int pX, int pY, float pFi)
	{
		x = pX;
		y = pY;
		Fi = pFi;
	}

	void set(int pX, int pY, float pFi)
	{
		x = pX;
		y = pY;
		Fi = pFi;
	}
};