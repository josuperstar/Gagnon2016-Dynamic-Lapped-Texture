#ifndef _KDTREENODE_H_
#define _KDTREENODE_H_

#include "Node.h"
#include "../Model3D/Point.h"
#include <vector>
using namespace std;

class KdTreeNode : public Node
{
public :
	KdTreeNode();

//private :
	KdTreeNode* parent;
	KdTreeNode* LeftChild;
	KdTreeNode* RightChild;
	Point* location;
	int axis;
	
};

#endif