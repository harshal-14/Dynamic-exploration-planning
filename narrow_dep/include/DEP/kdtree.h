#include <iostream>
#include <octomap/OcTree.h>
#include <octomap/octomap.h>
#include <algorithm>
#include <stdlib.h>
#include <unordered_set>
#include <queue>
#include <algorithm>
using namespace octomap;
using std::cout; using std::endl;

// =====================RRT/KDTree NODE=================================
typedef struct Node{
	point3d p; // (x, y, z) see octomath::vector3
	double yaw; //radius for best yaw angle
	double num_voxels;
	double ig;           // information gain by distance and num voxels
	std::map<double, int> yaw_num_voxels; // record for each angle
	struct Node* left;   // kd-tree left
	struct Node* right;  // kd-tree right
	struct Node* tree_parent; // kd-tree parent
	struct Node* parent; // for backtracking
	struct Node* img_n;  // for multi goal a star
	double g; 			 // gvalue for a star search
	double f;            // fvalue in a star search
	bool update;         // if we need to reevaluate the info gain
	bool new_node; 		 // if the node is newly added
	std::unordered_set<Node*> adjNodes;
	/////narrows 
	bool iam_entry;
	
	//double entry_voxels;
	bool entry_explored=false;
	std::vector<point3d> wallpoints;
	bool came_from_innermap;
	//double thresh_high;
	//double thresh_low;
	Node (){}
	Node(point3d _p){
		p = _p;
		left = NULL;
		right = NULL;
		tree_parent = NULL;
		parent = NULL;
		g = 10000000;
		f = 10000000;
		update = false;
	}
} Node;

struct CompareNode{
	bool operator()(Node* n1, Node* n2){
		return n1->f > n2->f;
	}
};

struct GainCompareNode{
	bool operator()(Node* n1, Node* n2){
		return n1->num_voxels < n2->num_voxels;
	}
};

////narrows 
/*struct EntryCompareNode{
	bool operator()(Node* n1, Node* n2){
		return n1->entry_voxels < n2->entry_voxels;
	}
};*/
//======================================================================


//===================Class Declaration==================================
typedef class KDTree{
private:
	int size;
	Node* root;
	// std::vector<Node*> goal_nodes;
	std::vector<Node*> not_target;
	std::priority_queue<Node*, std::vector<Node*>, GainCompareNode> goal_nodes;
	std::priority_queue<Node*, std::vector<Node*>, GainCompareNode> entry_goals;
	//////narrow ////
	//std::vector<Node*>entry_goals;


	std::vector<Node*> record;

	/////narrow///
	std::vector<Node*> entries;
	int total_num_unknown;
	int max_unknown;
public:
	KDTree();
	~KDTree(); // destructor for free memory;
	Node* getRoot();
	int getSize();

	/// narrow space entry ////
	bool closed_search;     ////////////////a flag to check if new entries detected .....
	
	/// //////////////////////////////
	
	void insert(Node* n);
	Node* nearestNeighbor(Node* n, 
					      Node* root_node,
					      Node* best_node,
						  double least_distance,
						  int depth); // return pointer to the nearest neighbor
	std::vector<Node*> kNearestNeighbor(Node* n, int num);
	void addRecord(Node* n);
	void addEntry(Node* n);
	void addGoalPQ(Node* n);
	////narrow
	void addEntryGoal(Node* n );
	
	/////
	std::priority_queue<Node*, std::vector<Node*>, GainCompareNode> getGoalNodes();

	///narrow
	//std::vector<Node*> getEntryGoals();
	void removeEntry(Node* n);
	std::priority_queue<Node*, std::vector<Node*>, GainCompareNode> getEntryGoals();
	/////
	void clearEntryGoals();
	void removeTopGoalNode();
	void clearGoalPQ();
	std::vector<Node*>& getRecord();
	std::vector<Node*>& getEntry();
	void setTotalUnknown(int _total_num_unknown);
	int getTotalUnknown();
	void setMaxUnknown(int _max_unknown);
	int getMaxUnknown();
	void clear();   // empty tree
} PRM;

//======================================================================


// ===================Implementation====================================
KDTree::KDTree(){
	// root = NULL;
	size = 0;
}

KDTree::~KDTree(){
	entries.clear();
	for (Node* n: record){
		delete n;
	}
	
	////for edges///
	/*if(!entries.empty()){
	for (Node* n: entries){
		delete n;
	}}*/
	
}

Node* KDTree::getRoot(){
	return root;
}

int KDTree::getSize(){
	return size;
}

void KDTree::insert(Node* n){
	n->left = NULL;
	n->right = NULL;
	
	// If tree is emtpy, we add root
	if (size == 0){
		std::cout<<"in kd tree before adding 2 "<<n<<std::endl;
		root = n;
		++size;
		std::cout<<"in kd tree after adding 2 "<<n<<std::endl;
		return;
	}
	else{
		Node* ptr = root;
		int depth = 0;
		double value, insert_value;
		while (true){
			if (depth % 3 == 0){
				value = ptr->p.x();
				insert_value = n->p.x();
			}
			else if (depth % 3 == 1){
				value = ptr->p.y();
				insert_value = n->p.y();

			}
			else if (depth % 3 == 2){
				value = ptr->p.z();
				insert_value = n->p.z();

			}
			// Compare the value:
			// >=: right, < left
			if (insert_value >= value){
				if (ptr->right == NULL){
					ptr->right = n;
					n->tree_parent = ptr;
					++size;
					return;
				}
				ptr = ptr->right;
			}
			else{
				if (ptr->left == NULL){
					ptr->left = n;
					n->tree_parent = ptr;
					++size;
					return;
				}
				ptr = ptr->left;
			}
			++depth;
		}
	}
	return;
}

static double least_distance_nn = 10000000;
Node* KDTree::nearestNeighbor(Node* n, 
							  Node* root_node=NULL,
							  Node* best_node = NULL,
							  double least_distance=1000000,
							  int depth=0){
	point3d p = n->p; // get position/coordinate of the node
	Node* ptr;
	if (root_node == NULL){
		ptr = root;	
	}
	else{
		ptr = root_node;
	}
	// Search Good side
	// Store Bad side
	std::vector<Node*> bad_side;
	while (ptr != NULL){
		
		// cout << "depth: " << depth << endl;
	 //    cout << "point: "<< ptr->p.x() <<", " <<  ptr->p.y()<<", " << ptr->p.z() << endl;

		// Check current node againt the best node
		double distance_current = p.distance(ptr->p);

		//==============Nodes Which are not our target================
 		if (ptr == n){
			distance_current = 10000000;
		}
		for (Node* nt: this->not_target){
			if (ptr == nt){
				distance_current = 10000000;
				break;
			}
		}

		//============================================================
		if (distance_current < least_distance_nn){	
			best_node = ptr;
			least_distance_nn = distance_current;
		}


		// Determine which side is better:
		double value, query_value;
		if (depth % 3 == 0){
			value = ptr->p.x();
			query_value = p.x();
		}
		else if (depth % 3 == 1){
			value = ptr->p.y();
			query_value = p.y();
		}
		else if (depth % 3 == 2){
			value = ptr->p.z();
			query_value = p.z();
		}

		// if < then search left, if >= search right
		if (query_value < value){
			bad_side.push_back(ptr->right);	
			ptr = ptr->left;
			// cout << "left" << endl;
		}
		else{
			bad_side.push_back(ptr->left);	
			ptr = ptr->right;
			// cout << "right" << endl;
		}
		++depth;
	}

	// Search bad side:
	// cout << depth << endl;
	std::reverse(bad_side.begin(), bad_side.end());
	// cout << "bad side size :" << bad_side.size() << endl;
	// cout << "bad side size: " << bad_side.size() << endl;
	// cout << "least distance after good search: " << least_distance << endl; 
	int count = 0;
	for (std::vector<Node*>::iterator itr = bad_side.begin(); 
		 itr != bad_side.end(); ++itr){
		// cout << "count: " << count << endl;
		++count;
		// If there is no node in bad side
		  if (*itr == NULL) {
            --depth;
            continue;
        }
		else{
			// cout << "have branch" << endl;
			double value, query_value;
			if ((depth-1) % 3 == 0){
 				value = (*itr)->tree_parent->p.x();
				query_value = p.x();
			}
			else if ((depth-1) % 3 == 1){
				value = (*itr)->tree_parent->p.y();
				query_value = p.y();
			}
			else if ((depth-1) % 3 == 2){
				value = (*itr)->tree_parent->p.z();
				query_value = p.z();
			}
			double best_bad_side_distance = std::abs(value - query_value);
			// cout << "value, query_value: " << value << " " << query_value << endl;
			// cout << "depth: " << depth << "  best bad: " << best_bad_side_distance << endl;
			// if (best_node != NULL){
			// 	cout << "current best: "<< best_node->p.x() <<", " <<  best_node->p.y()<<", " << best_node->p.z() << endl;
			// }
			// else{
			// 	cout << "current best is null" << endl;
			// }
			// cout << "least_distance: " << least_distance << endl;
			if (best_bad_side_distance >= least_distance){

				// cout << "best distance is not good enough" << endl;
				// cout << "best bad side distance: " << best_bad_side_distance << endl;
				// cout << "least distance: " << least_distance << endl;
				--depth;
				continue;
			}
			else{
				// cout << "recursive call at depth : " << depth << endl;
				// cout << "root point: "<< (*itr)->p.x() <<", " <<  (*itr)->p.y()<<", " << (*itr)->p.z() << endl;
				// cout << "least_distance: " << least_distance << endl;
				best_node = nearestNeighbor(n, *itr, best_node, least_distance, depth);
				--depth;	
			}
		}	
	}

	if (root_node == NULL){
		least_distance_nn = 1000000;
	}
	return best_node;
}

// Returns the k-nearest neighbor in ascending order
std::vector<Node*> KDTree::kNearestNeighbor(Node* n, int num){
	std::vector<Node*> knn;
	// cout << "================start================" <<endl;
	for (int i=0; i<num; ++i){
		// cout << "====================" << i << "=========================" << endl;
		Node* nearest_neighbor = nearestNeighbor(n);
		knn.push_back(nearest_neighbor);
		this->not_target.push_back(nearest_neighbor);
		// cout << n->p.distance(nearest_neighbor->p) << endl;
		// cout << "================================================" << endl;
	}
	// cout << "================end================" <<endl;
	// std::reverse(knn.begin(), knn.end());
	this->not_target.clear();
	return knn;
}

void KDTree::addRecord(Node* n){
	// this->goal_nodes.push_back(n);
	this->record.push_back(n);
}

void KDTree::addEntry(Node* n){
	// this->goal_nodes.push_back(n);
	this->entries.push_back(n);
}

void KDTree::addGoalPQ(Node* n){
	this->goal_nodes.push(n);
}

////narrow ///
void KDTree::addEntryGoal(Node* n){
	this->entry_goals.push(n);
}

/// 
std::priority_queue<Node*, std::vector<Node*>, GainCompareNode> KDTree::getGoalNodes(){
	return this->goal_nodes;
}

////narrow
/*td::vector<Node*> KDTree::getEntryGoals(){
	return this->entry_goals;
}*/
std::priority_queue<Node*, std::vector<Node*>, GainCompareNode> KDTree::getEntryGoals(){
	return this->entry_goals;
}




void KDTree::clearEntryGoals(){
    this->entry_goals = std::priority_queue<Node*, std::vector<Node*>, GainCompareNode>();
}

void KDTree::removeTopGoalNode(){
	this->goal_nodes.pop();
}



void KDTree::clearGoalPQ(){
	this->goal_nodes = std::priority_queue<Node*, std::vector<Node*>, GainCompareNode> ();
}

std::vector<Node*>& KDTree::getRecord(){
	return this->record;
}


void  KDTree::removeEntry(Node* n ){
	this->entries.erase(std::remove(entries.begin(), entries.end(), n), entries.end());
}
std::vector<Node*>& KDTree::getEntry(){
	return this->entries;
}
void KDTree::setTotalUnknown(int _total_num_unknown){
	this->total_num_unknown = _total_num_unknown;
}

int KDTree::getTotalUnknown(){
	return this->total_num_unknown;
}

void KDTree::setMaxUnknown(int _max_unknown){
	this->max_unknown = _max_unknown;
}

int KDTree::getMaxUnknown(){
	return this->max_unknown;
}

void KDTree::clear(){
	root = NULL;
	size = 0;
	return;
}

