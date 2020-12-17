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
#include "point_set.h"

Point* PointSet::m_pcurr = 0x0;

PointSet::PointSet ()
{	
	m_GridRes.Set ( 0, 0, 0 );
	m_pcurr = 0x0;
	//Reset ();
}

int PointSet::GetGridCell ( int x, int y, int z )
{
	return (int) ( (z*m_GridRes.y + y)*m_GridRes.x + x);
}


Point* PointSet::getFirstGridParticle(int gc)
{
	if (gc > m_Grid.size())
		return 0x0;

	return m_Grid[gc];
}

Point* PointSet::firstGridParticle ( int gc)
{
	m_pcurr = m_Grid [ gc ];
	if ( m_pcurr == 0x0 ) return 0x0;
	return m_pcurr;
}

Point* PointSet::nextGridParticle ()
{
	Point* pnt = 0x0;
	if ( m_pcurr != 0x0 ) {
		pnt = m_pcurr;
		m_pcurr = pnt->next;
	}	
	return pnt;
}
//
//unsigned short* PointSet::getNeighborTable ( int n, int& cnt )
//{
//	cnt = m_NC[n];
//	if ( cnt == 0 ) return 0x0;
//	return &m_Neighbor[n][0];
//}
//
//float* PointSet::getNeighborDist(int n)
//{
//	return &m_NDist[n][0];
//}
//
//void PointSet::clearNeighborTable()
//{
//	for (int n=0; n<65536; ++n)
//	{
//		clearNeighborTable(n);
//	}
//}
//
//void PointSet::clearNeighborTable(int n)
//{
//	m_NC[n] = 0;
//}
//
//void PointSet::addNeighbor(int n, int j, float dist)
//{
//	if (m_NC[n] < MAX_NEIGHBOR)
//	{
//		m_Neighbor[n][m_NC[n]] = j;
//		m_NDist[n][m_NC[n]] = dist;
//		++m_NC[n];
//	}
//}

// Ideal grid cell size (gs) = 2 * smoothing radius = 0.02*2 = 0.04
// Ideal domain size = k*gs/d = k*0.02*2/0.005 = k*8 = {8, 16, 24, 32, 40, 48, ..}
//    (k = number of cells, gs = cell size, d = simulation scale)
void PointSet::Grid_Setup ( Vector3DF min, Vector3DF max, float sim_scale, float cell_size, float border )
{
	float world_cellsize = cell_size / sim_scale;
	m_Grid.clear ();
	m_GridMin = min;	m_GridMin -= border;
	m_GridMax = max;	m_GridMax += border;
	m_GridSize = m_GridMax;
	m_GridSize -= m_GridMin;
	m_GridCellsize = world_cellsize;
	m_GridRes.x = ceil ( m_GridSize.x / world_cellsize );		// Determine grid resolution
	m_GridRes.y = ceil ( m_GridSize.y / world_cellsize );
	m_GridRes.z = ceil ( m_GridSize.z / world_cellsize );
	m_GridSize.x = m_GridRes.x * cell_size / sim_scale;				// Adjust grid size to multiple of cell size
	m_GridSize.y = m_GridRes.y * cell_size / sim_scale;
	m_GridSize.z = m_GridRes.z * cell_size / sim_scale;
	m_GridDelta = m_GridRes;		// delta = translate from world space to cell #
	m_GridDelta /= m_GridSize;
	m_GridTotal = (int)(m_GridRes.x * m_GridRes.y * m_GridRes.z);

	m_Grid.clear ();
	m_GridCnt.clear ();

	m_Grid.reserve ( m_GridTotal );
	m_GridCnt.reserve ( m_GridTotal );	
	for (int n=0; n < m_GridTotal; n++) {
		m_Grid.push_back ( 0x0 );
		m_GridCnt.push_back ( 0 );
	}

}

void PointSet::Grid_InsertParticles ()
{
	char *dat1, *dat1_end;
	Point *p;
	int gs;
	int gx, gy, gz;
	
	dat1_end = mBuf[0].data + NumPoints()*mBuf[0].stride;
	for ( dat1 = mBuf[0].data; dat1 < dat1_end; dat1 += mBuf[0].stride ) 
		((Point*) dat1)->next = 0x0;

	for (int n=0; n < m_GridTotal; n++) {
		m_Grid[n] = 0x0;
		m_GridCnt[n] = 0;
	}

	dat1_end = mBuf[0].data + NumPoints()*mBuf[0].stride;
	for ( dat1 = mBuf[0].data; dat1 < dat1_end; dat1 += mBuf[0].stride ) {
		p = (Point*) dat1;
		gx = (int)( (p->pos.x - m_GridMin.x) * m_GridDelta.x);		// Determine grid cell
		gy = (int)( (p->pos.y - m_GridMin.y) * m_GridDelta.y);
		gz = (int)( (p->pos.z - m_GridMin.z) * m_GridDelta.z);
		gs = (int)( (gz*m_GridRes.y + gy)*m_GridRes.x + gx);		
		if ( gs >= 0 && gs < m_GridTotal ) {
			p->next = m_Grid[gs];
			m_Grid[gs] = p;
			++m_GridCnt[gs];
		}
	}
}

int PointSet::Grid_FindCell ( Vector3DF p )
{
	int gc;
	Vector3DI cell;
	cell.x = (int) (p.x - m_GridMin.x) * m_GridDelta.x;
	cell.y = (int) (p.y - m_GridMin.y) * m_GridDelta.y;
	cell.z = (int) (p.z - m_GridMin.z) * m_GridDelta.z;
	gc = (int)( (cell.z*m_GridRes.y + cell.y)*m_GridRes.x + cell.x);
	if ( gc < 0 || gc > m_GridTotal ) return -1;
	return gc;
}

void PointSet::Grid_FindCells ( Vector3DF p, float radius )
{
	Grid_FindCells(p, radius, m_GridCell);
}

void PointSet::Grid_FindCells(Vector3DF p, float radius, int gridCell[27])
{
	Vector3DI sph_min;

	// Compute sphere range
	sph_min.x = (int)((-radius + p.x - m_GridMin.x) * m_GridDelta.x);
	sph_min.y = (int)((-radius + p.y - m_GridMin.y) * m_GridDelta.y);
	sph_min.z = (int)((-radius + p.z - m_GridMin.z) * m_GridDelta.z);
	if ( sph_min.x < 0 ) sph_min.x = 0;
	if ( sph_min.y < 0 ) sph_min.y = 0;
	if ( sph_min.z < 0 ) sph_min.z = 0;

	gridCell[0] = (int)((sph_min.z * m_GridRes.y + sph_min.y) * m_GridRes.x + sph_min.x);
	gridCell[1] = gridCell[0] + 1;
	gridCell[2] = (int)(gridCell[0] + m_GridRes.x);
	gridCell[3] = gridCell[2] + 1;

	if ( sph_min.z+1 < m_GridRes.z ) {
		gridCell[4] = (int)(gridCell[0] + m_GridRes.y*m_GridRes.x);
		gridCell[5] = gridCell[4] + 1;
		gridCell[6] = (int)(gridCell[4] + m_GridRes.x);
		gridCell[7] = gridCell[6] + 1;
	}
	if ( sph_min.x+1 >= m_GridRes.x ) {
		gridCell[1] = -1;		gridCell[3] = -1;		
		gridCell[5] = -1;		gridCell[7] = -1;
	}
	if ( sph_min.y+1 >= m_GridRes.y ) {
		gridCell[2] = -1;		gridCell[3] = -1;
		gridCell[6] = -1;		gridCell[7] = -1;
	}
}
