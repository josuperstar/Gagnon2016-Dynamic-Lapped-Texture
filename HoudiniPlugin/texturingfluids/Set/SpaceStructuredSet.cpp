#include "SpaceStructuredSet.h"

int SpaceStructuredSet::m_pcurr = -1;

SpaceStructuredSet::SpaceStructuredSet ()
{	
	
	m_pcurr = -1;
	//Reset ();
}


void SpaceStructuredSet::Update()
{
	

}

void SpaceStructuredSet::ComputeNeighbors()
{

}

void SpaceStructuredSet::ConstructStructure()
{

}

vector<Point*> SpaceStructuredSet::GetNeighborsList(int index, float radius)
{
	return this->pointList;
}

void SpaceStructuredSet::CreatePointList()
{
	pointList.clear();

	Point* p;
	int stride = GetStride(0);
	char *dat1, *dat1_end;
	dat1_end = GetStart(0) + NumPoints()*stride;
	int i = 0;
	for ( dat1 = GetStart(0); dat1 < dat1_end; dat1 += stride, ++i )
	{
		p = (Point*) dat1;
		p->id = i;
		pointList.push_back(p);
	}
}

std::vector<Point*>& SpaceStructuredSet::getNeighborTable ( int n )
{
	if (m_Neighbor.size()<=n)
	{
		m_Neighbor.resize(n+1);
		m_NDist.resize(n+1);
	}
	
	return m_Neighbor[n];
}

std::vector<float>& SpaceStructuredSet::getNeighborDist(int n)
{
	if (m_Neighbor.size()<=n)
	{
		m_Neighbor.resize(n+1);
		m_NDist.resize(n+1);
	}
	
	return m_NDist[n];
}

void SpaceStructuredSet::clearNeighborTable()
{
	for (int n=0; n<m_Neighbor.size(); ++n)
	{
		clearNeighborTable(n);
	}
}

void SpaceStructuredSet::clearNeighborTable(int n)
{
	if (m_Neighbor.size() > n)
	{
		m_Neighbor[n].clear();
		m_NDist[n].clear();
	}
}

void SpaceStructuredSet::addNeighbor(int n, Point* j, float dist)
{
	if (m_Neighbor.size()<=n)
	{
		m_Neighbor.resize(n+1);
		m_NDist.resize(n+1);
	}

	if (m_Neighbor[n].size() < MAX_NEIGHBOR)
	{
		m_Neighbor[n].push_back(j);
		m_NDist[n].push_back(dist);
	}
}

