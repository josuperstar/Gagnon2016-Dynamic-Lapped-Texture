#ifndef _SPACESTRUCTUREDSET_H
	#define _SPACESTRUCTUREDSET_H

	#include <iostream>
	#include <vector>
	#include <stdio.h>
	#include <stdlib.h>
	#include <math.h>
	
	#include "Set.h"
	#include "vector.h"
	#include "Particle.h"
	#include "Point.h"

	// TODO: NEEDS REFACTORING!!!! We should never put a "using namespace" directive in a .h.
	// This goes against the purpose of namespaces.
	using namespace Model3D;
	using namespace Math;
	using namespace std;
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

	
	class SpaceStructuredSet : public Set {
	public:
		SpaceStructuredSet ();
		virtual ~SpaceStructuredSet() {}

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
		


		std::vector<Point*>&	getNeighborTable ( int n );
		std::vector<float>&		getNeighborDist ( int n );
		//std::vector<float*>*		getNeighbors(Vector3DF p, float radius);

		void	virtual ComputeNeighbors();
		void	clearNeighborTable();		// Clear all neighbors for all particles
		void	clearNeighborTable(int n);	// Clear all neighbors for particle 'n'
		void	addNeighbor(int n, Point* j, float dist);	// Add a neighbor for particle 'n'

		virtual void	Update();
		virtual void	ConstructStructure();
		virtual vector<Point*>	GetNeighborsList(int index,float radius);

		void CreatePointList();


	protected:
		int							m_Frame;		

		// Parameters
		double						m_Param [ MAX_PARAM ];			// see defines above
		Vector3DF					m_Vec [ MAX_PARAM ];
		bool						m_Toggle [ MAX_PARAM ];
		




		// Neighbor Table
		std::vector<std::vector<Point*> >	m_Neighbor;		// Neighbors indexes
		std::vector<std::vector<float> >	m_NDist;		// Neighbors distances

		static int m_pcurr;
		std::vector<Point*> pointList;
	};
#endif
