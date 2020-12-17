#ifndef GRIDFACTORY_H
#define GRIDFACTORY_H
#include <Math/vector.h>
#include "StructureFactory.h"


class GridSetFactory : public StructureFactory
{
public:
	GridSetFactory();
	~GridSetFactory();

	virtual SpaceStructuredSet* InitializeStructureSet();
	virtual void CreateStructureSet(SpaceStructuredSet* set, vector<Point*> pointList);
	void SetVolumeDimensions(Vector3DF d){volumeDimension = d;}

private :
	Vector3DF volumeDimension;

};
#endif	// SPHFACTORY_H