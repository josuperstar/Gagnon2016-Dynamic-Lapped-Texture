/*
  FLUIDS v.1 - SPH Fluid Simulator for CPU and GPU
  Copyright (C) 2008. Rama Hoetzlein, http://www.rchoetzlein.com

  ZLib license
  This software is provided 'as-is', without any express or implied
  warranty.  In no event will the authors be held liable for any damages
  arising from the use of this software.

  Permission is granted to anyone to use this software for any purpose,
  including commercial applications, and to alter it and redistribute it
  freely, subject to the following restrictions:

  1. The origin of this software must not be misrepresented; you must not
     claim that you wrote the original software. If you use this software
     in a product, an acknowledgment in the product documentation would be
     appreciated but is not required.
  2. Altered source versions must be plainly marked as such, and must not be
     misrepresented as being the original software.
  3. This notice may not be removed or altered from any source distribution.
*/

#ifndef _GRID_SET_H_
	#define _GRID_SET_H_

	#include <iostream>
	#include <vector>
	#include <stdio.h>
	#include <stdlib.h>
	#include <math.h>
	
	#include "SpaceStructuredSet.h"
	#include "vector.h"	
	#include "Point.h"
	#include "Particle.h"
	using namespace Math;
	typedef signed int		xref;
	
	#define MAX_NEIGHBOR		80
	
	#define MAX_PARAM			32

	// Scalar params
	#define PNT_DRAWMODE		0
		#define PNT_SPHERE			0
		#define PNT_POINT			1
	#define PNT_DRAWSIZE		1		
	#define POINT_GRAV			2
	#define PLANE_GRAV			3

	// Vector params
	#define EMIT_POS			0
	#define EMIT_ANG			1
	#define EMIT_DANG			2
	#define EMIT_SPREAD			3
	#define EMIT_RATE			4
	#define POINT_GRAV_POS		5	
	#define PLANE_GRAV_DIR		6	


	#define BPOINT				0
	#define BPARTICLE			1

	/*
	struct Point {
		Vector3DF		pos;
		DWORD			clr;
		int				next;
	};
	
	struct Particle {	
		Vector3DF		pos;
		DWORD			clr;
		int				next;
		Vector3DF		vel;			
		Vector3DF		vel_eval;		
		unsigned short	age;
	};
	*/


#define SetDimension 729

typedef struct {
	int						m_Frame;		

	// Spatial Grid
	Point**		 				m_Grid;
	int*						m_GridCnt;
	int						m_GridTotal;			// total # cells
	Vector3DF					m_GridMin;				// volume of grid (may not match domain volume exactly)
	Vector3DF					m_GridMax;
	Vector3DF					m_GridRes;				// resolution in each axis
	Vector3DF					m_GridSize;				// physical size in each axis
	Vector3DF					m_GridDelta;
	float						m_GridCellsize;
	int						m_GridCell[27];

	int GetSize() 
	{ 
		return 3*sizeof(int) + 5*sizeof(Vector3DF) + sizeof(float) + 2 * m_GridTotal * sizeof(int);
	}

} Grid;

	class GridSet : public SpaceStructuredSet {
	public:
		GridSet ();
		virtual ~GridSet();

		void	virtual ComputeNeighbors();
		// Spatial Subdivision

		void virtual Update();

		void Grid_Setup ( Vector3DF min, Vector3DF max, float sim_scale, float cell_size, float border );		
		void Grid_Create ();
		void ConstructStructure ();	
		void Grid_Draw ( float* view_mat );		
		void Grid_FindCells (Vector3DF p, float radius, int gridCell[27]);
		void Grid_FindCells ( Vector3DF p, float radius );
		int Grid_FindCell ( Vector3DF p );
		Vector3DF GetGridRes ()		{ return grid->m_GridRes; }
		Vector3DF GetGridMin ()		{ return grid->m_GridMin; }
		Vector3DF GetGridMax ()		{ return grid->m_GridMax; }
		Vector3DF GetGridDelta ()	{ return grid->m_GridDelta; }
		int GetGridCell ( int x, int y, int z );
		Point* getFirstGridParticle(int gc);
		Point* firstGridParticle( int gc);
		Point* nextGridParticle();

		Grid* GetGrid(){ return grid;}

		void UpdateParticle(Point* particle, const Vector3DF& oldPos, const Vector3DF& newPos);


	protected:
		Grid	*grid;
		static Point* m_pcurr;
	};

#endif
