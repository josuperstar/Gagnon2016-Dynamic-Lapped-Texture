#include "KDTreeFactory.h"
#include "../KdTree.h"

// -----------------------------------------------------------------------------
// Constructor/Destructor
KdTreeFactory::KdTreeFactory()
{}

KdTreeFactory::~KdTreeFactory()
{}

// -----------------------------------------------------------------------------
// Public functions
SpaceStructuredSet* KdTreeFactory::InitializeStructureSet()
{
	KdTree* set = new KdTree();
	//set->CreateKdTree(pointList,3);

	return set;
}

void KdTreeFactory::CreateStructureSet(SpaceStructuredSet* set, std::vector<Point*> pointList)
{
	((KdTree*)set)->CreateKdTree(NULL,pointList,0,3);
}
