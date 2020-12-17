#ifndef _NODE_H_
#define _NODE_H_

#include <vector>
using namespace std;

class Node
{
public :
	Node() {}
	void SetIsLeaf(bool value) {isLeaf = value;}
	bool IsLeaf() {return isLeaf;}
private :
	
	vector<Node*> children;
protected :
	bool isLeaf;

};

#endif