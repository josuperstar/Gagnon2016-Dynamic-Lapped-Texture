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
#include "GridSet.h"

Point* GridSet::m_pcurr = 0x0;

GridSet::GridSet ()
{
    grid = new Grid();


    grid->m_GridRes.Set ( 0, 0, 0 );
    m_pcurr = 0x0;
    //Reset ();
}


GridSet::~GridSet ()
{
    delete[] grid->m_Grid;
    delete[] grid->m_GridCnt;
    grid = 0;
}

int GridSet::GetGridCell ( int x, int y, int z )
{
    return (int) ( (z* grid->m_GridRes.y + y)* grid->m_GridRes.x + x);
}


Point* GridSet::getFirstGridParticle(int gc)
{
    //cout << "gc = "<<gc << " sizeof(m_Grid) = "<<sizeof(grid->m_Grid) <<endl;

    //if (gc > sizeof(grid->m_Grid) || sizeof(grid->m_Grid)==0 )
    //	return -1;

    return grid->m_Grid[gc];
}

Point* GridSet::firstGridParticle(int gc)
{
    m_pcurr = grid->m_Grid [ gc ];
    if ( m_pcurr == 0x0 ) return 0x0;
    return m_pcurr;
}

Point* GridSet::nextGridParticle()
{
    Point* pnt = 0x0;
    if ( m_pcurr != 0x0 ) {
        pnt = m_pcurr;
        m_pcurr = pnt->next;
    }
    return pnt;
}
//
//unsigned short* GridSet::getNeighborTable ( int n, int& cnt )
//{
//	cnt = m_NC[n];
//	if ( cnt == 0 ) return 0x0;
//	return &m_Neighbor[n][0];
//}
//
//float* GridSet::getNeighborDist(int n)
//{
//	return &m_NDist[n][0];
//}
//
//void GridSet::clearNeighborTable()
//{
//	for (int n=0; n<65536; ++n)
//	{
//		clearNeighborTable(n);
//	}
//}
//
//void GridSet::clearNeighborTable(int n)
//{
//	m_NC[n] = 0;
//}
//
//void GridSet::addNeighbor(int n, int j, float dist)
//{
//	if (m_NC[n] < MAX_NEIGHBOR)
//	{
//		m_Neighbor[n][m_NC[n]] = j;
//		m_NDist[n][m_NC[n]] = dist;
//		++m_NC[n];
//	}
//}

void GridSet::Update()
{

}


// Ideal grid cell size (gs) = 2 * smoothing radius = 0.02*2 = 0.04
// Ideal domain size = k*gs/d = k*0.02*2/0.005 = k*8 = {8, 16, 24, 32, 40, 48, ..}
//    (k = number of cells, gs = cell size, d = simulation scale)
void GridSet::Grid_Setup ( Vector3DF min, Vector3DF max, float sim_scale, float cell_size, float border )
{
    float world_cellsize = cell_size / sim_scale;
    grid->m_Grid = 0;
    grid->m_GridMin = min;
    grid->m_GridMin -= border;
    grid->m_GridMax = max;
    grid->m_GridMax += border;
    grid->m_GridSize = grid->m_GridMax;
    grid->m_GridSize -= grid->m_GridMin;
    grid->m_GridCellsize = world_cellsize;
    grid->m_GridRes.x = ceil ( grid->m_GridSize.x / world_cellsize );		// Determine grid resolution
    grid->m_GridRes.y = ceil ( grid->m_GridSize.y / world_cellsize );
    grid->m_GridRes.z = ceil ( grid->m_GridSize.z / world_cellsize );
    grid->m_GridSize.x = grid->m_GridRes.x * cell_size / sim_scale;				// Adjust grid size to multiple of cell size
    grid->m_GridSize.y = grid->m_GridRes.y * cell_size / sim_scale;
    grid->m_GridSize.z = grid->m_GridRes.z * cell_size / sim_scale;
    grid->m_GridDelta = grid->m_GridRes;		// delta = translate from world space to cell #
    grid->m_GridDelta /= grid->m_GridSize;
    grid->m_GridTotal = (int)(grid->m_GridRes.x * grid->m_GridRes.y * grid->m_GridRes.z);

    //grid->m_Grid = 0;
    //grid->m_GridCnt = 0;

    
    //grid->m_Grid.reserve ( grid->m_GridTotal );
    //grid->m_GridCnt.reserve ( grid->m_GridTotal );
    //for (int n=0; n < grid->m_GridTotal; n++) {
    //	grid->m_Grid.push_back ( -1 );
    //	grid->m_GridCnt.push_back ( 0 );
    //}
    

	cout << "world cellsize "	<< world_cellsize 	<< endl;
	cout << "grid size "		<< grid->m_GridSize.x << " "<<grid->m_GridSize.y << " "<<grid->m_GridSize.z 	<< endl;
	cout << "grid->m_GridRes.x "	<< grid->m_GridRes.x 	<< endl;
	cout << "grid->m_GridRes.y "	<< grid->m_GridRes.y 	<< endl;
	cout << "grid->m_GridRes.z "	<< grid->m_GridRes.z 	<< endl;
	cout << "grid->m_GridSize.x = grid->m_GridRes.x * cell_size / sim_scale "<< grid->m_GridRes.x * cell_size / sim_scale << endl;


    cout << "# # # # # # # # # # # # # # # # #"<<endl;
    cout << "initialize the grid of "<<grid->m_GridTotal<<" size : ";

    grid->m_Grid = new Point*[grid->m_GridTotal];
    grid->m_GridCnt = new int[grid->m_GridTotal];
    for (int n=0; n < grid->m_GridTotal; n++)
    {
        //cout <<"init the grid !"<<endl;
        grid->m_Grid[n] = 0x0 ;
        grid->m_GridCnt[n] = 0 ;
    }
    cout << " init the grid OK " <<endl;
    cout << "# # # # # # # # # # # # # # # # #"<<endl;
    cout << " "<<endl;
}

void GridSet::ConstructStructure ()
{

    cout <<"construct the structure : ";

    char *dat1, *dat1_end;
    Point *p;
    int gs;
    int gx, gy, gz;

    dat1_end = mBuf[0].data + NumPoints()*mBuf[0].stride;
    for ( dat1 = mBuf[0].data; dat1 < dat1_end; dat1 += mBuf[0].stride )
        ((Point*) dat1)->next = 0x0;

    for (int n=0; n < grid->m_GridTotal; n++)
    {
        grid->m_Grid[n] = 0x0;
        grid->m_GridCnt[n] = 0;
    }

    dat1_end = mBuf[0].data + NumPoints()*mBuf[0].stride;
    for ( dat1 = mBuf[0].data; dat1 < dat1_end; dat1 += mBuf[0].stride )
    {
        p = (Point*) dat1;
        gx = (int)( (p->pos.x - grid->m_GridMin.x) * grid->m_GridDelta.x);		// Determine grid cell
        gy = (int)( (p->pos.y - grid->m_GridMin.y) * grid->m_GridDelta.y);
        gz = (int)( (p->pos.z - grid->m_GridMin.z) * grid->m_GridDelta.z);
        gs = (int)( (gz*grid->m_GridRes.y + gy)*grid->m_GridRes.x + gx);
        if ( gs >= 0 && gs < grid->m_GridTotal )
        {

            p->next = grid->m_Grid[gs];
            grid->m_Grid[gs] = p;
            ++grid->m_GridCnt[gs];
            //cout << "pnext = "<< p->next <<endl;
        }
    }
    cout << " structure OK " <<endl;
}

int GridSet::Grid_FindCell ( Vector3DF p )
{
    int gc;
    Vector3DI cell;
    cell.x = (int) ((p.x - grid->m_GridMin.x) * grid->m_GridDelta.x);
    cell.y = (int) ((p.y - grid->m_GridMin.y) * grid->m_GridDelta.y);
    cell.z = (int) ((p.z - grid->m_GridMin.z) * grid->m_GridDelta.z);
    gc = (int)( (cell.z*grid->m_GridRes.y + cell.y)*grid->m_GridRes.x + cell.x);
    if ( gc < 0 || gc > grid->m_GridTotal ) return -1;
    return gc;
}

void GridSet::Grid_FindCells ( Vector3DF p, float radius )
{
    Grid_FindCells(p, radius, grid->m_GridCell);
}

void GridSet::Grid_FindCells(Vector3DF p, float radius, int gridCell[27])
{
	
#if 1	
	Vector3DI cell;
  cell.x = (int) ((p.x - grid->m_GridMin.x) * grid->m_GridDelta.x);
  cell.y = (int) ((p.y - grid->m_GridMin.y) * grid->m_GridDelta.y);
  cell.z = (int) ((p.z - grid->m_GridMin.z) * grid->m_GridDelta.z);
	
	int c = 0;
	for (int dx=-1; dx<=1; ++dx)
	{
		for (int dy=-1; dy<=1; ++dy)
		{
			for (int dz=-1; dz<=1; ++dz)
			{
				int gs = (int)( ((cell.z+dz)*grid->m_GridRes.y + (cell.y+dy))*grid->m_GridRes.x + (cell.x+dx));
				if (gs >=0 && gs < grid->m_GridTotal)
				{
					gridCell[c] = gs;
				}
				else
				{
					gridCell[c] = -1;
				}
				++c;
			}
		}
	}
	
	if (cell.x <= 0)
	{
		gridCell[0] = -1;
		gridCell[1] = -1;
		gridCell[2] = -1;
		gridCell[3] = -1;
		gridCell[4] = -1;
		gridCell[5] = -1;
		gridCell[6] = -1;
		gridCell[7] = -1;
		gridCell[8] = -1;
	}
	if ((cell.x+1) >= grid->m_GridRes.x)
	{
		gridCell[18] = -1;
		gridCell[19] = -1;
		gridCell[20] = -1;
		gridCell[21] = -1;
		gridCell[22] = -1;
		gridCell[23] = -1;
		gridCell[24] = -1;
		gridCell[25] = -1;
		gridCell[26] = -1;
	}
	if (cell.y <= 0)
	{
		gridCell[0] = -1;
		gridCell[1] = -1;
		gridCell[2] = -1;
		gridCell[9] = -1;
		gridCell[10] = -1;
		gridCell[11] = -1;
		gridCell[18] = -1;
		gridCell[19] = -1;
		gridCell[20] = -1;
	}
	if ((cell.y+1) >= grid->m_GridRes.y)
	{
		gridCell[6] = -1;
		gridCell[7] = -1;
		gridCell[8] = -1;
		gridCell[15] = -1;
		gridCell[16] = -1;
		gridCell[17] = -1;
		gridCell[24] = -1;
		gridCell[25] = -1;
		gridCell[26] = -1;
	}
	if (cell.z <= 0)
	{
		gridCell[0] = -1;
		gridCell[9] = -1;
		gridCell[18] = -1;
		gridCell[3] = -1;
		gridCell[12] = -1;
		gridCell[21] = -1;
		gridCell[6] = -1;
		gridCell[15] = -1;
		gridCell[24] = -1;
	}
	if ((cell.z+1) >= grid->m_GridRes.z)
	{
		gridCell[2] = -1;
		gridCell[11] = -1;
		gridCell[20] = -1;
		gridCell[5] = -1;
		gridCell[14] = -1;
		gridCell[23] = -1;
		gridCell[8] = -1;
		gridCell[17] = -1;
		gridCell[26] = -1;
	}
	
#else
    Vector3DI sph_min;

    // Compute sphere range
    sph_min.x = (int)((-radius + p.x - grid->m_GridMin.x) * grid->m_GridDelta.x);
    sph_min.y = (int)((-radius + p.y - grid->m_GridMin.y) * grid->m_GridDelta.y);
    sph_min.z = (int)((-radius + p.z - grid->m_GridMin.z) * grid->m_GridDelta.z);
    if ( sph_min.x < 0 ) sph_min.x = 0;
    if ( sph_min.y < 0 ) sph_min.y = 0;
    if ( sph_min.z < 0 ) sph_min.z = 0;


    //cout << "sph_min.x " << sph_min.x <<" sph_min.x " << sph_min.y << " sph_min.z " << sph_min.z<<endl;

    gridCell[0] = (int)((sph_min.z * grid->m_GridRes.y + sph_min.y) * grid->m_GridRes.x + sph_min.x);
    gridCell[1] = gridCell[0] + 1;
    gridCell[2] = (int)(gridCell[0] + grid->m_GridRes.x);
    gridCell[3] = gridCell[2] + 1;

    if ( sph_min.z+1 < grid->m_GridRes.z )
    {
        gridCell[4] = (int)(gridCell[0] + grid->m_GridRes.y*grid->m_GridRes.x);
        gridCell[5] = gridCell[4] + 1;
        gridCell[6] = (int)(gridCell[4] + grid->m_GridRes.x);
        gridCell[7] = gridCell[6] + 1;
    }
    if ( sph_min.x+1 >= grid->m_GridRes.x )
    {
        gridCell[1] = -1;
        gridCell[3] = -1;
        gridCell[5] = -1;
        gridCell[7] = -1;
    }
    if ( sph_min.y+1 >= grid->m_GridRes.y )
    {
        gridCell[2] = -1;
        gridCell[3] = -1;
        gridCell[6] = -1;
        gridCell[7] = -1;
    }

    /*
    for (int i=0;i<27;i++)
    {
    	cout << "grid " <<i<<" = "<<gridCell[i]<<endl;
    }
    */
#endif

}

void GridSet::UpdateParticle(Point* particle, const Vector3DF& oldPos, const Vector3DF& newPos)
{
    Vector3DI 	oldGridCell;
    Vector3DI 	gridCell;
    Point*		pNext;
    int			oldIndex;
    int			newIndex;

    // Compute old and new grid cell index
    oldGridCell.x = (int)((oldPos.x - grid->m_GridMin.x) * grid->m_GridDelta.x);
    oldGridCell.y = (int)((oldPos.y - grid->m_GridMin.y) * grid->m_GridDelta.y);
    oldGridCell.z = (int)((oldPos.z - grid->m_GridMin.z) * grid->m_GridDelta.z);
    oldIndex = (int)( (oldGridCell.z*grid->m_GridRes.y + oldGridCell.y)
                      *grid->m_GridRes.x + oldGridCell.x);

    gridCell.x = (int)((newPos.x - grid->m_GridMin.x) * grid->m_GridDelta.x);
    gridCell.y = (int)((newPos.y - grid->m_GridMin.y) * grid->m_GridDelta.y);
    gridCell.z = (int)((newPos.z - grid->m_GridMin.z) * grid->m_GridDelta.z);
    newIndex = (int)( (gridCell.z*grid->m_GridRes.y + gridCell.y)*grid->m_GridRes.x + gridCell.x);

    if (oldIndex != newIndex)
    {
        // Remove from old cell
        if (grid->m_Grid[oldIndex] == particle)
        {
            grid->m_Grid[oldIndex] = particle->next;
            --grid->m_GridCnt[oldIndex];
        }
        else
        {
            pNext = grid->m_Grid[oldIndex];
            bool isFound = false;
            int i = 0;
            do
            {
                if (pNext->next == particle)
                {
                    pNext->next = particle->next;
                    --grid->m_GridCnt[oldIndex];
                    isFound = true;
                }
                else
                {
                    pNext = pNext->next;
                    ++i;
                }

            } while (!isFound && (i<grid->m_GridCnt[oldIndex]));
        }

        // Add into new cell! :D
        particle->next = grid->m_Grid[newIndex];
        grid->m_Grid[newIndex] = particle;
        ++grid->m_GridCnt[newIndex];
    }

}

void GridSet::ComputeNeighbors()
{
    //char *dat1, *dat1_end;
    //Fluid* p;
    //Fluid* pcurr;
    //int pndx;
    //int i, cnt = 0;
    //float dx, dy, dz, sum, dsq, c;
    //float d, d2, mR, mR2;
    //float radius = system.GetParam(SPH_SMOOTHRADIUS) / system.GetParam(SPH_SIMSCALE);
    //d = system.GetParam(SPH_SIMSCALE);
    //d2 = d*d;
    //mR = system.GetParam(SPH_SMOOTHRADIUS);
    //mR2 = mR*mR;
    //int gridCell[27];

    //ushort stride = GetStride(0);
    //char* dataStart = GetStart(0);
    //dat1_end = dataStart + NumPoints()*stride;
    //i = 0;
    //for ( dat1 = dataStart; dat1 < dat1_end; dat1 += stride, i++ ) {
    //	p = (Fluid*) dat1;

    //	system.clearNeighborTable(i);
    //
    //	Grid_FindCells ( p->pos, radius, gridCell );
    //	for (int cell=0; cell < 8; cell++) {
    //		if ( gridCell[cell] != -1 ) {
    //			pndx = getFirstGridParticle(gridCell[cell]);
    //			while ( pndx != -1 )
    //			{
    //				pcurr = (Fluid*) (dataStart + pndx*stride);
    //				if ( pcurr == p ) {pndx = pcurr->next; continue; }
    //				dx = ( p->pos.x - pcurr->pos.x)*d;		// dist in cm
    //				dy = ( p->pos.y - pcurr->pos.y)*d;
    //				dz = ( p->pos.z - pcurr->pos.z)*d;
    //				dsq = (dx*dx + dy*dy + dz*dz);
    //
    //				if ( mR2 > dsq ) {
    //					((SpaceStructuredSet*)system.GetSet())->addNeighbor(i, pndx, sqrt(dsq));
    //				}
    //				pndx = pcurr->next;
    //			}
    //		}
    //		gridCell[cell] = -1;
    //	}
    //}



}
