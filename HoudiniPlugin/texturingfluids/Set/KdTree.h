#ifndef _KDTREE_H_
#define _KDTREE_H_

#include "Element.h"
#include "Tree.h"
#include "KdTreeNode.h"
#include <vector>
#include "Model3D/Point.h"
using namespace std;

class KdTree : public Tree
{
public :
	KdTree();
	~KdTree();

	virtual void	Update();
	virtual void	ConstructStructure();
	virtual vector<Point*>	GetNeighborsList(int index,float radius);

	KdTreeNode* CreateKdTree(KdTreeNode* parent,vector<Point*> pointList, int depth, int k);

private :
	//vector<KdTreeNode> nodes;
	KdTreeNode* node;

	void KdTreeNodeNeighbors(KdTreeNode* root, Point*p, double radius, vector<Point*> &result);
};

#endif