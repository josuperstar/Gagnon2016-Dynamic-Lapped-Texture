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

#ifndef DEF_POINT_SET
	#define DEF_POINT_SET

	#include <iostream>
	#include <vector>
	#include <stdio.h>
	#include <stdlib.h>
	#include <math.h>
	
	#include "SpaceStructuredSet.h"
	#include <Math/vector.h>	
	#include <Model3D/Point.h>
	#include <Model3D/Particle.h>
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

	class PointSet : public SpaceStructuredSet {
	public:
		PointSet ();

		/*
		// Point Sets
		Point* GetPoint ( int n )		{ return (Point*) GetElem(0, n); }		
		int	NumPoints ()				{ return NumElem(0); }		
		
		// Particle system	
		Particle* GetParticle ( int n )		{ return (Particle*) GetElem(0, n); }	;

		// Parameters			
		void SetParam (int p, float v )		{ m_Param[p] = v; }
		void SetParam (int p, int v )		{ m_Param[p] = (float) v; }
		float GetParam ( int p )			{ return (float) m_Param[p]; }
		Vector3DF GetVec ( int p )			{ return m_Vec[p]; }
		void SetVec ( int p, Vector3DF v )	{ m_Vec[p] = v; }
		void Toggle ( int p )				{ m_Toggle[p] = !m_Toggle[p]; }		
		bool GetToggle ( int p )			{ return m_Toggle[p]; }
		void SetToggle(int p, bool v)		{ m_Toggle[p] = v;}
		*/


		// Spatial Subdivision
		void Grid_Setup ( Vector3DF min, Vector3DF max, float sim_scale, float cell_size, float border );		
		void Grid_Create ();
		void Grid_InsertParticles ();	
		void Grid_Draw ( float* view_mat );		
		void Grid_FindCells (Vector3DF p, float radius, int gridCell[27]);
		void Grid_FindCells ( Vector3DF p, float radius );
		int Grid_FindCell ( Vector3DF p );
		Vector3DF GetGridRes ()		{ return m_GridRes; }
		Vector3DF GetGridMin ()		{ return m_GridMin; }
		Vector3DF GetGridMax ()		{ return m_GridMax; }
		Vector3DF GetGridDelta ()	{ return m_GridDelta; }
		int GetGridCell ( int x, int y, int z );
		Point* getFirstGridParticle(int gc);
		Point* firstGridParticle(int gc);
		Point* nextGridParticle();

		/*
		unsigned short* getNeighborTable ( int n, int& cnt );
		float*			getNeighborDist ( int n );

		void	clearNeighborTable();		// Clear all neighbors for all particles
		void	clearNeighborTable(int n);	// Clear all neighbors for particle 'n'
		void	addNeighbor(int n, int j, float dist);	// Add a neighbor for particle 'n'
		*/
	protected:
		int							m_Frame;		

		//// Parameters
		//double						m_Param [ MAX_PARAM ];			// see defines above
		//Vector3DF					m_Vec [ MAX_PARAM ];
		//bool						m_Toggle [ MAX_PARAM ];
		


		// Spatial Grid
		std::vector< Point* >			m_Grid;
		std::vector< int >			m_GridCnt;
		int							m_GridTotal;			// total # cells
		Vector3DF					m_GridMin;				// volume of grid (may not match domain volume exactly)
		Vector3DF					m_GridMax;
		Vector3DF					m_GridRes;				// resolution in each axis
		Vector3DF					m_GridSize;				// physical size in each axis
		Vector3DF					m_GridDelta;
		float						m_GridCellsize;
		int							m_GridCell[27];

		// Neighbor Table
		//unsigned short				m_NC[65536];			// neighbor table (600k)
		//unsigned short				m_Neighbor[65536][MAX_NEIGHBOR];	
		//float						m_NDist[65536][MAX_NEIGHBOR];

		static Point* m_pcurr;
	};

#endif
