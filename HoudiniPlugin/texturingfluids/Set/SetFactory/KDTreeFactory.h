#ifndef KDTREEFACTORY_H
#define KDTREEFACTORY_H

#include "StructureFactory.h"


class KdTreeFactory : public StructureFactory
{
public:
	KdTreeFactory();
	~KdTreeFactory();

	virtual SpaceStructuredSet* InitializeStructureSet();
	virtual void CreateStructureSet(SpaceStructuredSet* set, vector<Point*> pointList);


};
#endif	// SPHFACTORY_H