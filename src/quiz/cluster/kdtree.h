/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}

	~Node()
	{
		delete left;
		delete right;
	}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	~KdTree()
	{
		delete root;
	}

	void insert_helper(Node** root_node, uint depth, std::vector<float> new_point, int id) 
	{
		// if the traverse tree is empty, then replace the root node as the passing point
		if (*root_node == NULL) 
		{
			*root_node = new Node(new_point, id);
		}
		else // still not empty
		{
			uint pos = depth % 2;
			if ((*root_node)->point[pos] > new_point[pos])
				insert_helper(&(*root_node)->left, depth+1, new_point, id);
			else
				insert_helper(&(*root_node)->right, depth+1, new_point, id);
		}
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insert_helper(&root, 0, point, id);
	}

    void search_helper(std::vector<float> target, Node** node, int depth, float distanceTol, std::vector<int>* ids)
	{
		if (*node != NULL) 
		{
			float x_diff = fabs((*node)->point[0] - target[0]);
			float y_diff = fabs((*node)->point[1] - target[1]);
			// if node is within the box
			if (x_diff <= distanceTol && y_diff <= distanceTol) 
			{
				float radius_to_target = sqrt(x_diff*x_diff+y_diff*y_diff);
				if (radius_to_target <= distanceTol) 
				{
					ids->push_back((*node)->id);
					std::cout << "find point!" << std::endl;
					std::cout << "x is : " << (*node)->point[0] << std::endl;
					std::cout << "y is : " << (*node)->point[1] << std::endl;
				}
			}
			// looking for more points
			uint pos = depth % 2;
			
			// make sure to compare point with the box instead of target itself
			// otherwise, the search won't be complete
			if ((*node)->point[pos] > target[pos] - distanceTol) 
			{
				search_helper(target, &((*node)->left), depth+1, distanceTol, ids);
				std::cout << "search left!" << std::endl;
			}

			if ((*node)->point[pos] < target[pos] + distanceTol) 
			{
				search_helper(target, &((*node)->right), depth+1, distanceTol, ids);
				std::cout << "search right!" << std::endl;
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		search_helper(target, &root, 0, distanceTol, &ids);
		return ids;
	}
	

};




