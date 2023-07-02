#include <iostream>
#include <math.h>
#include <stdio.h>


struct Node  //树节点
{
	int id;
	Node* left;
	Node* right;

	Node(int setId)
	:	id(setId), left(NULL), right(NULL)
	{}
};


template <typename PointT>
struct KdTree  //树
{
	Node* root;
	typename pcl::PointCloud<PointT>::Ptr cloud;  // reference to pcl::PointCloud<PointT> object  对pcl:：PointCloud<PointT>对象的引用 

	KdTree(typename pcl::PointCloud<PointT>::Ptr pointcloud)
	: root(NULL), cloud(pointcloud)
	{}


    void insertHelper(Node** node, unsigned int depth, int id)  // double pointer (Node**) because root was defined as Node* (node pointer) originally, and then we pass the memory address
	{
		/* Creates a new node and places it in the right position in the k-d tree */

		if (*node == NULL)  // dereferencing to get value  // terminates when it hits a null node  取消引用以获取值//在其到达空节点时终止 
		{
			*node = new Node(id);  // pointing root pointer to new data  // TOTRY: use a pointer reference instead, then no need to dereference it (*node)  改用指针引用，则无需取消引用它（*node）
		}
		else  // traverse
		{
			unsigned int variable = depth % 3;  // 3D, always 0, 1 or 2

			if (variable == 0)  // point[0]: x value
            {
                if (cloud->points[id].x < cloud->points[(*node)->id].x)
                    insertHelper(&((*node)->left), depth + 1, id);  // passing address of dereferenced node's left child
                else
                    insertHelper(&((*node)->right), depth + 1, id);
            }
            else if (variable == 1)
            {
                if (cloud->points[id].y < cloud->points[(*node)->id].y)
                    insertHelper(&((*node)->left), depth + 1, id);
                else
                    insertHelper(&((*node)->right), depth + 1, id);
            }
            else if (variable == 2)
            {
                if (cloud->points[id].z < cloud->points[(*node)->id].z)
                    insertHelper(&((*node)->left), depth + 1, id);
                else
                    insertHelper(&((*node)->right), depth + 1, id);
            }
		}
    }
    

    void insertPointIndex(int id)
	{
		insertHelper(&root, 0, id);  // passing the address of root  输入根节点
	}


    void searchHelper(int id, Node** node, unsigned int depth, float distanceTol, std::vector<int>& nearbyPointIds)
	{

		if (*node != NULL)
		{
			// Checking if point in current node is inside the target box  检查该节点中的点是否在目标框内

			if (   (cloud->points[(*node)->id].x >= (cloud->points[id].x - distanceTol) && cloud->points[(*node)->id].x <= (cloud->points[id].x + distanceTol)) \
				&& (cloud->points[(*node)->id].y >= (cloud->points[id].y - distanceTol) && cloud->points[(*node)->id].y <= (cloud->points[id].y + distanceTol)) \
				&& (cloud->points[(*node)->id].z >= (cloud->points[id].z - distanceTol) && cloud->points[(*node)->id].z <= (cloud->points[id].z + distanceTol)) )
			{
				
				// Finding distance between 3D points (node x, y, z and target x, y, z)
				float distance = sqrt( pow((cloud->points[(*node)->id].x - cloud->points[id].x), 2.0) + pow((cloud->points[(*node)->id].y - cloud->points[id].y), 2.0) + pow((cloud->points[(*node)->id].z - cloud->points[id].z), 2.0));

				if (distance <= distanceTol)
				{
					nearbyPointIds.push_back((*node)->id);  // prev: nearbyPointIds.push_back(cloud->points[(*node)->id]);  // the point the node id refers to
				}
			}

			// TODO: check if indentation/ flow is correct

			// Checking box boundary to see where to move down next in the tree (left or right)

			unsigned int varToCompare = depth % 3;

			if (varToCompare == 0)  // considering x dimension
			{
				if ( (cloud->points[id].x - distanceTol) < cloud->points[(*node)->id].x )  // if left boundary of box is < node's x|y|z value, that box is in the left region 如果框的左边界是<节点的x|y|z值，则该框位于左侧区域 
					searchHelper(id, &((*node)->left), depth + 1, distanceTol, nearbyPointIds);
				if ( (cloud->points[id].x + distanceTol) > cloud->points[(*node)->id].x )  // if left edge of the box is greater than the node's x or y value, then that box is in the right region 如果框的左边缘大于节点的x或y值，则该框位于右侧区域
					searchHelper(id, &((*node)->right), depth + 1, distanceTol, nearbyPointIds);
			}
			else if (varToCompare == 1)  // considering y dimension
			{
				if ( (cloud->points[id].y - distanceTol) < cloud->points[(*node)->id].y )
					searchHelper(id, &((*node)->left), depth + 1, distanceTol, nearbyPointIds);
				if ( (cloud->points[id].y + distanceTol) > cloud->points[(*node)->id].y )
					searchHelper(id, &((*node)->right), depth + 1, distanceTol, nearbyPointIds);
			}
			else if (varToCompare == 2)  // considering z dimension
			{
				if ( (cloud->points[id].z - distanceTol) < cloud->points[(*node)->id].z )
					searchHelper(id, &((*node)->left), depth + 1, distanceTol, nearbyPointIds);
				if ( (cloud->points[id].z + distanceTol) > cloud->points[(*node)->id].z )
					searchHelper(id, &((*node)->right), depth + 1, distanceTol, nearbyPointIds);
			}
		}
    }


    std::vector<int> search(int id, float distanceTol)
	{
        std::vector<int> nearbyPointIds;  // TODO: make it a pointer
		searchHelper(id, &root, 0, distanceTol, nearbyPointIds);  // redefine root as Node, and remove &?, then node doesn't need to be dereferenced as *node in helper function
		return nearbyPointIds;
    }
};
