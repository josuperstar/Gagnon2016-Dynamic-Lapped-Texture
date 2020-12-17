#ifndef SPATIALGRID_H
#define SPATIALGRID_H

// NOTE: Due to inlining and use of templates, all the function definitions can be found
// 		 in SpatialGrid.hpp which is "included" at the end of the file.

//#include "Math/Vec3.h"


#include <list>
#include <vector>


using namespace TexturingFluids;

// Class SpatialGrid : Grid for spatial partitionning
template<class T>
class SpatialGrid
{
public:
    SpatialGrid(){};
    SpatialGrid(double cellSize, const Vec3f& volumeMin, const Vec3f& volumeMax);
	~SpatialGrid();

    void initGrid(double cellSize, const Vec3f& volumeMin, const Vec3f& volumeMax);
	void clear();

    bool insert(const T& element, const Vec3f& position);
    bool insert(const T& element, const Vec3f& AABBmin, const Vec3f& AABBmax);

    void getElements(const Vec3f& position, double radius, std::vector<T*>& elements);
    void getElements(const Vec3f& position, double radius, std::vector<const T*>& elements) const;
	void getElements(int ix, int iy, int iz, std::vector<T*>& elements);

	// Get grid infos
	unsigned int getNbCells() const { return _grid.size(); }
	unsigned int getResX() const { return _resX; }
	unsigned int getResY() const { return _resY; }
	unsigned int getResZ() const { return _resZ; }
	double getCellSize() const { return _cellSize; }
    //Vec3d getVolumeStart() const { return _volMin; }

	bool isCellEmpty(int ix, int iy, int iz) const { return _grid[getGridIndex(ix, iy, iz)].empty(); }

private:
	int getGridIndex(int ix, int iy, int iz) const;
    int getXIndex(const Vec3f& position) const;
    int getYIndex(const Vec3f& position) const;
    int getZIndex(const Vec3f& position) const;
	int getXIndex(double xPos) const;
	int getYIndex(double yPos) const;
	int getZIndex(double zPos) const;

private:
	std::vector<std::vector<T> >	_grid;
	
	unsigned int	_resX;
	unsigned int	_resY;
	unsigned int	_resZ;
	double			_cellSize;
    Vec3f		_volMin;
};	


// Include definitions
#include "SpatialGrid.hpp"

#endif	// SPATIALGRID_H
