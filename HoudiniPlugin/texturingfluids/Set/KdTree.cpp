#include "KdTree.h"

#include "SortAlgorithms/MedianSelection.h"

KdTree::KdTree()
{

}

KdTree::~KdTree()
{

}

void	KdTree::Update()
{

}
void	KdTree::ConstructStructure()
{
	CreatePointList();
	this->node  = this->CreateKdTree(NULL, this->pointList,0,3);

}

KdTreeNode* KdTree::CreateKdTree(KdTreeNode* _parent, vector<Point*> pointList, int depth, int k)
{

	KdTreeNode* node = new KdTreeNode();
	node->axis = depth % k; 


	MedianSelection selection;
	Point* median = selection.Select(pointList,0,pointList.size()-1,(int)(pointList.size()/2),node->axis);
	if (!median)
	{
		return NULL;
	}
	vector<Point*> pointBeforeMedian;
	vector<Point*> pointAfterMedian;
	

	for (vector<Point*>::iterator it = pointList.begin(); it!=pointList.end(); ++it)
	{
		if ((*it)->id == median->id)
			continue;

		if ((*it)->GetValueByAxis(node->axis) < median->GetValueByAxis(node->axis) )
			pointBeforeMedian.push_back(*it);
		else if ((*it)->GetValueByAxis(node->axis) >= median->GetValueByAxis(node->axis))
			pointAfterMedian.push_back(*it);
	}



	node->parent = _parent;
	node->location = median;

	node->LeftChild = NULL;
	node->RightChild  = NULL;

	if (pointBeforeMedian.size() > 0)
	{
		KdTreeNode* before = CreateKdTree(node,pointBeforeMedian,depth+1,3);
		if (before != NULL)
			node->LeftChild = before;
	}
	if(pointAfterMedian.size() > 0)
	{
		KdTreeNode* after = CreateKdTree(node,pointAfterMedian,depth+1,3);
		if (after != NULL)
			node->RightChild = after;
	}

	if (pointAfterMedian.size() == 0&& pointBeforeMedian.size() == 0)
		node->SetIsLeaf(true);
	return node;
}

vector<Point*>	KdTree::GetNeighborsList(int index,float radius)
{
	Point* point = this->pointList[index];

	vector<Point*> list;

	KdTreeNodeNeighbors(this->node,point,radius,list);

	return list;
	
}

void KdTree::KdTreeNodeNeighbors(KdTreeNode* here, Point*p, double radius, vector<Point*> &result)
{

	double d = 0;

	//cout <<"AXIS "<<here->axis<<endl;

	if (here->location)
	{
		
		d = Point::Distance(here->location,p);
		if (d <= radius)
		{
			//cout << here->location->id <<" is inside"<<endl;
			result.push_back(here->location);
				
		}
		/*else
		{
			cout <<"is not inside" <<endl;
		}*/
	}
	else
		return;


	
	float traveler = here->location->GetValueByAxis(here->axis);
	float current = p->GetValueByAxis(here->axis);
	float min = current-radius;
	float max = current+radius;

	if (d <= radius)
	{
		if (here->LeftChild)
		{
			KdTreeNodeNeighbors(here->LeftChild,p, radius, result);	
		}
		if (here->RightChild)
		{
			KdTreeNodeNeighbors(here->RightChild,p, radius, result);	
		}
	}
	else 
	{
		if (traveler < max && traveler > min)// && !here->IsLeaf())
		{
			//cout << "the traveler is under ther point"<<endl;
			if (here->LeftChild)
			{
				KdTreeNodeNeighbors(here->LeftChild,p, radius, result);	
			}
			if (here->RightChild)
			{
				KdTreeNodeNeighbors(here->RightChild,p, radius, result);	
			}
		}
		
		if (traveler < max && traveler < min)// && !here->IsLeaf())
		{
			//cout << "the traveler is is above ther point"<<endl;
			if (here->RightChild)
			{
				KdTreeNodeNeighbors(here->RightChild,p, radius, result);	
			}
		}
		if (traveler > max)// && !here->IsLeaf())
		{
			//cout << "the traveler is is above ther point"<<endl;
			if (here->LeftChild)
			{
				KdTreeNodeNeighbors(here->LeftChild,p, radius, result);	
			}
		}
	}
	/*
	else if (traveler == current)// && !here->IsLeaf())
	{
		if (here->LeftChild)
		{
			KdTreeNodeNeighbors(here->LeftChild,p, radius, result);	
		}
		if (here->RightChild)
		{
			KdTreeNodeNeighbors(here->RightChild,p, radius, result);	
		}
	}
	*/

	
	//return here;
}