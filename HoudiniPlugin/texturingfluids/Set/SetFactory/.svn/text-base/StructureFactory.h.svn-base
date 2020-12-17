#ifndef STRUCTUREFACTORY_H
#define STRUCTUREFACTORY_H

#include "../SpaceStructuredSet.h"
#include "Point.h"

using namespace std;
class StructureFactory
{
public:
	StructureFactory();
	~StructureFactory();

	virtual SpaceStructuredSet* InitializeStructureSet() = 0;
	virtual void CreateStructureSet(SpaceStructuredSet* set, vector<Point*> pointList) = 0;	
};

#endif	// SPHFACTORY_H
