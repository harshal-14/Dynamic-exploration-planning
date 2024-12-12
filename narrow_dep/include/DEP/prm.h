#include <DEP/kdtree.h>
#include <DEP/utils.h>
#include <DEP/env.h>
#include <random>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <voxblox_ros/esdf_server.h>
#include <cmath>


// Depth CAMERA
double FOV = 1.8;
double dmin = 0;
double dmax = 1.0;
/// new addition spars
/*double zone1=0.5;
double zone2=1.5;
double zone3=2.5;



void clearance_calculator(const OcTree& tree,Node* n) {
    point3d p=n->p;
	
    double ray_yaw;
    double min_clearance = 1000;  // Set a large value as initial minimum clearance
    for (ray_yaw = 0; ray_yaw < 2 * M_PI; ray_yaw += M_PI / 10) {
        point3d direction(cos(ray_yaw), sin(ray_yaw), 0.0);  // Direction in 2D plane
        direction = direction.normalized();  // Normalize the direction

        point3d end;  // Endpoint of the ray 
        bool ignoreUnknownCells = true;

        // Cast a ray in the current direction
        bool hit_surface = tree.castRay(p, direction, end, ignoreUnknownCells, 10.0);
        if (hit_surface) {
            double ray_distance = n->p.distance(end);
            min_clearance = std::min(min_clearance, ray_distance);
        }
    }

    // Assign threshold values based on min_clearance
    if (min_clearance < zone1) {
        n->thresh_low = 0.8;
        n->thresh_high = 1.2;
    } else if (min_clearance < zone2) {
        n->thresh_low = 1.2;
        n->thresh_high = 1.8;
    } else if (min_clearance < zone3) {
        n->thresh_low = 1.8;
        n->thresh_high = 2.4;
    } else {
        n->thresh_low = 2.4;
        n->thresh_high = 3.0;
    }
}
*/

// Visualize Map
bool VISUALIZE_MAP = true;
// static std::vector<geometry_msgs::Point> DEFAULT_VECTOR;
static std::vector<visualization_msgs::Marker> DEFAULT_VECTOR;

///static vector of edges //
static std::vector<visualization_msgs::Marker> DEFAULT_EDGEVECTOR;

static std::vector<visualization_msgs::Marker> DEFAULT_INNERVECTOR;

std::vector<double> generate_yaws(int n){
	std::vector<double> yaws;
	for (int i=0; i<n; ++i){
		yaws.push_back(i*2*PI_const/n);
	}
	return yaws;
}
// std::vector<double> yaws {0, PI_const/4, PI_const/2, 3*PI_const/4, PI_const, 5*PI_const/4, 3*PI_const/2, 7*PI_const/4};
// std::vector<double> yaws {0, PI_const/8 ,PI_const/4, PI_const*3/8, PI_const/2, PI_const*5/8, 3*PI_const/4, PI_const*7/8, PI_const, PI_const*9/8,5*PI_const/4, PI_const*11/8,3*PI_const/2, PI_const*13/8,7*PI_const/4, PI_const*15/8};
std::vector<double> yaws = generate_yaws(32);
// Random Generator
std::random_device rd;
std::mt19937 mt(rd());
// Helper Function: Random Number
double randomNumber(double min, double max){
	std::uniform_real_distribution<double> distribution(min, max);
	return distribution(mt);
}


// Hepler Function: Check Position Validation
bool isValid(const OcTree& tree, point3d p, bool robot_size=false){
	if (robot_size == false){
		OcTreeNode* nptr = tree.search(p);
		if (nptr == NULL){return false;}
		return !tree.isNodeOccupied(nptr);
	}
	else{// we should consider the robot size in this case
		// calculate x, y, z range
		double x_min, x_max, y_min, y_max, z_min, z_max;
		x_min = p.x() - DRONE_X/2;
		x_max = p.x() + DRONE_X/2;
		y_min = p.y() - DRONE_Y/2;
		y_max = p.y() + DRONE_Y/2;
		z_min = p.z() - DRONE_Z/2;
		z_max = p.z() + DRONE_Z/2;

		for (double x=x_min; x<=x_max+DRONE_X/2; x+=RES){
			for (double y=y_min; y<=y_max+DRONE_X/2; y+=RES){
				for (double z=z_min; z<z_max; z+=RES){
					if (isValid(tree, point3d(x, y, z))){
						continue;
					}
					else{
						return false;
					}
				}
			}
		}
		return true;
	}
}

// Random Sample
// allow_not_valid = true indicates the configuration could be invalid by this generator
Node* randomConfig(const OcTree& tree, bool allow_not_valid=false){ 
	double min_x, max_x, min_y, max_y, min_z, max_z;
	tree.getMetricMax(max_x, max_y, max_z);
	tree.getMetricMin(min_x, min_y, min_z);
	min_x = std::max(min_x, env_x_min);
	max_x = std::min(max_x, env_x_max);
	min_y = std::max(min_y, env_y_min);
	max_y = std::min(max_y, env_y_max);
	min_z = std::max(min_z, env_z_min);
	max_z = std::min(max_z, env_z_max);
	bool valid = false;
	double x, y, z;
	point3d p;

	while (valid == false){
		p.x() = randomNumber(min_x, max_x);
		p.y() = randomNumber(min_y, max_y);
		p.z() = randomNumber(min_z, max_z);
		if (allow_not_valid==true){ 
			break;
		}
		valid = isValid(tree, p, true);
		// valid = isValid(tree, p);
	}

	Node* nptr = new Node(p);
	////new addition for spars
    //clearance_calculator(tree,nptr);
	nptr->iam_entry=false;
	return nptr;
}


// BBX: (xmin, xmax, ymin, ymax, zmin, zmax)
Node* randomConfigBBX(const OcTree& tree, std::vector<double> &bbx){ 
	double min_x, max_x, min_y, max_y, min_z, max_z;
	tree.getMetricMax(max_x, max_y, max_z);
	tree.getMetricMin(min_x, min_y, min_z);
	min_x = std::max(min_x, bbx[0]);
	max_x = std::min(max_x, bbx[1]);
	min_y = std::max(min_y, bbx[2]);
	max_y = std::min(max_y, bbx[3]);
	min_z = std::max(min_z, bbx[4]);
	max_z = std::min(max_z, bbx[5]);
	min_x = std::max(min_x, env_x_min);
	max_x = std::min(max_x, env_x_max);
	min_y = std::max(min_y, env_y_min);
	max_y = std::min(max_y, env_y_max);
	min_z = std::max(min_z, env_z_min);
	max_z = std::min(max_z, env_z_max);
	bool valid = false;
	double x, y, z;
	point3d p;

	while (valid == false){
		
		p.x() = randomNumber(min_x, max_x);
		// Y>0
		// p.y() = randomNumber(0, max_y);
		p.y() = randomNumber(min_y, max_y);
		p.z() = randomNumber(min_z, max_z);
		// p.z() = randomNumber(min_z, 2.5); // garage and cafe
		valid = isValid(tree, p, true);
		// valid = isValid(tree, p);
	}

	Node* nptr = new Node(p);
	///new additon for spars
	//clearance_calculator(tree,nptr);
	nptr->iam_entry=false;
	return nptr;
}



// Function: Do Collision Checking
bool checkCollision(OcTree& tree, Node* n1, Node* n2){
	//std::cout<<"Never came here"<<std::endl;
	point3d p1 = n1->p;
	point3d p2 = n2->p;
	std::vector<point3d> ray;
	//std::cout<<"this is line 202"<<std::endl;
	tree.computeRay(p1, p2, ray);
	for (std::vector<point3d>::iterator itr=ray.begin(); itr!=ray.end(); ++itr){
		if (isValid(tree, *itr ,true)){
		// if (isValid(tree, *itr)){
			continue;
		}
		else{
			return true;
		}
	}
	return false;
}

bool checklinecross(point3d p1, point3d p2, point3d p3, point3d p4) {
    auto crossProduct = [](double ax, double ay, double bx, double by) {
        return ax * by - ay * bx;
    };

    // Compute cross products
    double d1 = crossProduct(p4.x() - p3.x(), p4.y() - p3.y(), p1.x() - p3.x(), p1.y() - p3.y());
    double d2 = crossProduct(p4.x() - p3.x(), p4.y() - p3.y(), p2.x() - p3.x(), p2.y() - p3.y());
    double d3 = crossProduct(p2.x() - p1.x(), p2.y() - p1.y(), p3.x() - p1.x(), p3.y() - p1.y());
    double d4 = crossProduct(p2.x() - p1.x(), p2.y() - p1.y(), p4.x() - p1.x(), p4.y() - p1.y());

    // Check if the segments straddle each other
    if (d1 * d2 < 0 && d3 * d4 < 0) {
        return true;
    }

  
    return false;
}






// This function is to find the neighbor of a given octree node center
// p is octree node center, we can directly use them
std::vector<point3d> getNeighborNode(point3d p){
	std::vector<point3d> neighbors;
	std::vector<double> DX {-RES, 0, +RES}; //DX
	std::vector<double> DY {-RES, 0, +RES}; //DY
	std::vector<double> DZ {-RES, 0, +RES}; //DZ
	for (double dx: DX){
		for (double dy: DY){
			for (double dz: DZ){
				if (dx != 0 or dy != 0 or dz != 0){
					neighbors.push_back(point3d (p.x()+dx, p.y()+dy, p.z()+dz));
				}
			}
		}
	}
	return neighbors;
}

// P has to be the free cell
bool isFrontier(point3d p_request, const OcTree& tree){
	for (point3d p: getNeighborNode(p_request)){
		bool not_in_scope = p.x() > env_x_max or p.x() < env_x_min or p.y() > env_y_max or p.y() < env_y_min or p.z() > env_z_max or p.z() < env_z_min;
		if (not_in_scope){
			continue;
		}
		OcTreeNode* nptr = tree.search(p);
		if (nptr != NULL and tree.isNodeOccupied(nptr) == false){

			return true;
		}
	}
	return false;
}

// P has to be a frontier in this function
bool isSurfaceFrontier(point3d p_request, const OcTree& tree){
	for (point3d p: getNeighborNode(p_request)){
		bool not_in_scope = p.x() > env_x_max or p.x() < env_x_min or p.y() > env_y_max or p.y() < env_y_min or p.z() > env_z_max or p.z() < env_z_min;
		if (not_in_scope){
			continue;
		}
		OcTreeNode* nptr = tree.search(p);
		if (nptr != NULL and tree.isNodeOccupied(nptr)){
			return true;
		}
	}
	
	return false;
}



point3d entry_point(point3d p, point3d end, point3d end2) {
    double x1 = end.x();    // Coordinates of A (point 'end')
    double y1 = end.y();

    double x2 = end2.x();   // Coordinates of B (point 'end2')
    double y2 = end2.y();

    double x3 = p.x();      // Coordinates of P (point 'p')
    double y3 = p.y();

    // Calculate the projection scalar 't'
    double ac_p = (x3 - x1) * (x2 - x1) + (y3 - y1) * (y2 - y1);
    double ab = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);
    double t = ac_p / ab;

    // Compute the projection point P'
    double x4 = x1 + t * (x2 - x1);
    double y4 = y1 + t * (y2 - y1);

    // Return the new point with the original z-coordinate of P
    point3d p_new(x4, y4, p.z());
    return p_new;
}



/////////////Things I am adding //////////////
//////////////////


// Assuming 'point3d' has a valid constructor
point3d isNarrowPassage(const point3d &p,std::vector<point3d> &wallpoints, const OcTree &tree) {
    double maxRange = 1.0;
	double min_distance=100;
	point3d ep;
    for (double i = 0; i < M_PI; i += M_PI / 10) {

        point3d direction(cos(i), sin(i), 0.0);
        direction = direction.normalized();

        point3d end;
        bool ignoreUnknownCells = true;
        bool hit = tree.castRay(p, direction, end, ignoreUnknownCells, maxRange);
        OcTreeNode* nptr = tree.search(end);
		bool iswall1=false;
		if(nptr!=NULL){
			iswall1=tree.isNodeOccupied(nptr);	
		}
        if (hit && iswall1) {
            for (double opp = i + 0.9 * M_PI; opp <= i + 1.1 * M_PI; opp += 0.1 * M_PI) {
                point3d indirection(cos(opp), sin(opp), 0.0);
                indirection = indirection.normalized();

                point3d end2;
                bool hit2 = tree.castRay(p, indirection, end2, ignoreUnknownCells, maxRange);
				OcTreeNode* nptr = tree.search(end2);
				bool iswall2=false;
				if(nptr!=NULL){
					iswall2=tree.isNodeOccupied(nptr);	
				}
                if (hit2 && iswall2) {
					///////checking if it is a corner
					//double clearance_range=std::max((p-end).norm(),(p-end2).norm());
					bool inward_condition;
					
					for(double vi=i+M_PI/20;vi<0.9*M_PI;vi+=M_PI/20){
                    point3d mid_direction(cos(vi), sin(vi), 0.0);
                    mid_direction = mid_direction.normalized();
					point3d mid;
					inward_condition=!(tree.castRay(p, mid_direction, mid, ignoreUnknownCells, 1.2*maxRange));
					if(inward_condition) {break;}
					}

					bool outward_condition;
                    point3d midopp_direction;
				
					for(double vi_opp=opp+M_PI/20;vi_opp<0.9*M_PI+opp;vi_opp+=M_PI/20){
					point3d midopp_direction(cos(vi_opp), sin(vi_opp), 0.0);
                    midopp_direction = midopp_direction.normalized();
					point3d midopp;
					outward_condition=!(tree.castRay(p, midopp_direction, midopp, ignoreUnknownCells, 1.2*maxRange));
					if(outward_condition){break;}}

                    

                    if (inward_condition && outward_condition) {
						double distance=(end2-end).norm();
						if(distance<min_distance){
                        ep=entry_point(p, end, end2);
						if(isValid(tree,ep,false)){

						 min_distance=distance;
						 wallpoints.clear();
						 wallpoints.push_back(end);
						 wallpoints.push_back(end2);
						}
						} // Return by value
                    }
                }
            }
        }
    }
	if(min_distance<100){
		return ep;

	}
    // No narrow passage detected, return a sentinel value
    return point3d(std::numeric_limits<double>::quiet_NaN(),
                   std::numeric_limits<double>::quiet_NaN(),
                   std::numeric_limits<double>::quiet_NaN());
}
/*point3d isNarrowPassage(const point3d &p,std::vector<point3d> &wallpoints, const OcTree &tree) {
    double maxRange = 1.0;
	double min_distance=100;
	point3d ep;
    for (double i = 0; i < M_PI; i += M_PI / 10) {

        point3d direction(cos(i), sin(i), 0.0);
        direction = direction.normalized();

        point3d end;
        bool ignoreUnknownCells = false;
        bool hit = tree.castRay(p, direction, end, ignoreUnknownCells, maxRange);
        
        if (hit) {
            for (double opp = i + 0.9 * M_PI; opp <= i + 1.1 * M_PI; opp += 0.1 * M_PI) {
                point3d indirection(cos(opp), sin(opp), 0.0);
                indirection = indirection.normalized();

                point3d end2;
                bool hit2 = tree.castRay(p, indirection, end2, ignoreUnknownCells, maxRange);
				
                if (hit2) {
					///////checking if it is a corner
					//double clearance_range=std::max((p-end).norm(),(p-end2).norm());
					bool inward_condition;
					
					for(double vi=i+M_PI/20;vi<0.9*M_PI;vi+=M_PI/20){
                    point3d mid_direction(cos(vi), sin(vi), 0.0);
                    mid_direction = mid_direction.normalized();
					point3d mid;
					inward_condition=!(tree.castRay(p, mid_direction, mid, ignoreUnknownCells, 1.2*maxRange));
					if(inward_condition) {break;}
					}

					bool outward_condition;
                    point3d midopp_direction;
				
					for(double vi_opp=opp+M_PI/20;vi_opp<0.9*M_PI+opp;vi_opp+=M_PI/20){
					point3d midopp_direction(cos(vi_opp), sin(vi_opp), 0.0);
                    midopp_direction = midopp_direction.normalized();
					point3d midopp;
					outward_condition=!(tree.castRay(p, midopp_direction, midopp, ignoreUnknownCells, 1.2*maxRange));
					if(outward_condition){break;}}

                    

                    if (inward_condition && outward_condition) {
						double distance=(end2-end).norm();
						if(distance<min_distance){
                        ep=entry_point(p, end, end2);
						if(isValid(tree,ep,false)){

						 min_distance=distance;
						 wallpoints.clear();
						 wallpoints.push_back(end);
						 wallpoints.push_back(end2);
						}
						} // Return by value
                    }
                }
            }
        }
    }
	if(min_distance<100){
		return ep;

	}
    // No narrow passage detected, return a sentinel value
    return point3d(std::numeric_limits<double>::quiet_NaN(),
                   std::numeric_limits<double>::quiet_NaN(),
                   std::numeric_limits<double>::quiet_NaN());
}*/




// This only count for vertical FOV not horizontal
bool isInFOV(const OcTree& tree, point3d p, point3d u, double dmax){
	double distance = p.distance(u);
	if (distance >= dmin and distance <= dmax){
		point3d direction = u - p;
		point3d face (direction.x(), direction.y(), 0);
		point3d v_direction = direction;
		double vertical_angle = face.angleTo(v_direction);
		if (vertical_angle <= FOV/2){
			point3d end;
			bool cast = tree.castRay(p, direction, end, true, distance);
			if (cast == false){
				return true;
			}
		}
	}
	return false;
}

double calculateUnknown(const OcTree& tree, Node* n, double dmax){
	// Position:
	point3d p = n->p;
	// Possible range
	double xmin, xmax, ymin, ymax, zmin, zmax;
	xmin = p.x() - dmax;
	xmax = p.x() + dmax;
	ymin = p.y() - dmax;
	ymax = p.y() + dmax;
	zmin = p.z() - dmax;
	zmax = p.z() + dmax;
	

	point3d pmin (xmin, ymin, zmin);
	point3d pmax (xmax, ymax, zmax);
	point3d_list node_centers;
	tree.getUnknownLeafCenters(node_centers, pmin, pmax);

	// Yaw candicates array;
	
	std::map<double, int> yaw_num_voxels;
	for (double yaw: yaws){
		yaw_num_voxels[yaw] = 0;
	}

	int count_total_unknown = 0;
	int count_total_frontier = 0;
	int count_total_surface_frontier = 0;
	for (std::list<point3d>::iterator itr=node_centers.begin(); 
		itr!=node_centers.end(); ++itr){
		point3d u = *itr;
		bool not_in_scope = u.x() > env_x_max or u.x() < env_x_min or u.y() > env_y_max or u.y() < env_y_min or u.z() > env_z_max or u.z() < env_z_min;
		if (not_in_scope){
			continue;
		}
		OcTreeNode* nptr = tree.search(u);
		point3d direction = u - p;
		point3d face (direction.x(), direction.y(), 0);
		if (nptr == NULL){ // Unknown
			if (isInFOV(tree, p, u, dmax)){
				bool isNodeFrontier=false, isNodeSurfaceFrontier=false; 
				isNodeFrontier= isFrontier(u, tree);
				if (isNodeFrontier){
					isNodeSurfaceFrontier = isSurfaceFrontier(u, tree);
				}
				
				if (isNodeFrontier == false and isNodeSurfaceFrontier == false){
					count_total_unknown += 1;
				}
				else if (isNodeFrontier == true and isNodeSurfaceFrontier == false){
					count_total_unknown += 2;
				}
				else if (isNodeFrontier == true and isNodeSurfaceFrontier == true){
					count_total_unknown += 4;
				}


				// iterate through yaw angles
				for (double yaw: yaws){
					point3d yaw_direction (cos(yaw), sin(yaw), 0);
					double angle_to_yaw = face.angleTo(yaw_direction);
					if (angle_to_yaw <= FOV/2){
						// Give credits to some good unknown
						// case 1: it is a frontier unknown
						if (isNodeFrontier == false and isNodeSurfaceFrontier == false){
							yaw_num_voxels[yaw] += 1;
						}
						else if (isNodeFrontier == true and isNodeSurfaceFrontier == false){
							yaw_num_voxels[yaw] += 2;
						}
						else if (isNodeFrontier == true and isNodeSurfaceFrontier == true){
							yaw_num_voxels[yaw] += 4;
						}						
					}
				}
			}
		}
	}
	// cout << "+----------------------------+" << endl;
	// cout << "Total Unknown: "<< count_total_unknown << endl;
	// cout << "Total Frontier: " << count_total_frontier << endl;
	// cout << "Total Surface Frontier: " << count_total_surface_frontier << endl;
	// cout << "+----------------------------+" << endl;
	n->yaw_num_voxels = yaw_num_voxels;
	return count_total_unknown;
}


bool isNodeRequireUpdate(Node* n, std::vector<Node*> path, double& least_distance){
	double distance_thresh = 2.0;//1.2*n->thresh_high; ////new additon in place of 2.0
	least_distance = 1000000;
	for (Node* waypoint: path){
		double current_distance = n->p.distance(waypoint->p);
		if (current_distance < least_distance){
			least_distance = current_distance;
		}
	}
	if (least_distance <= distance_thresh){
		return true;
	}
	else{
		return false;	
	}
	
}


// Ensure the vertical sensor range condition for node connection
bool sensorRangeCondition(Node* n1, Node* n2){
	point3d direction = n2->p - n1->p;
	point3d projection (direction.x(), direction.y(), 0);
	double vertical_angle = projection.angleTo(direction);
	if (vertical_angle < FOV/2){
		return true;
	}
	else{
		return false;
	}
}


/*PRM* buildRoadMap(OcTree &tree, 
				  PRM* map,
				  std::vector<Node*> path,
				  Node* start = NULL,  
				  std::vector<visualization_msgs::Marker> &map_vis_array = DEFAULT_VECTOR)*/
PRM* buildRoadMap(OcTree &tree, 
				  PRM* map,
				  std::vector<Node*> path,
				  Node* start = NULL,  
				  std::vector<visualization_msgs::Marker> &map_vis_array = DEFAULT_VECTOR,std::vector<visualization_msgs::Marker> &edge_vis_array = DEFAULT_EDGEVECTOR,
				  std::vector<visualization_msgs::Marker> &inner_map_vis_array = DEFAULT_INNERVECTOR,bool closed_search=false,PRM* outermap = NULL,Node* current_return_point = NULL,Node* prior_entry_node = NULL)
{
	// ==================================Sampling===========================================================
	// PRM* map = new PRM();
	map->clearGoalPQ();
	map->clearEntryGoals();
	std::vector<Node*> new_nodes;
	std::vector<Node*> edge_nodes;
	int small_box_entries=0;
	//map->closed_search=false;
	// double threshold = 500; // HardCode threshold
	bool saturate = false;
	bool region_saturate = false;
	int count_sample = 0;
	int sample_thresh = 50;
	double distance_thresh = 0.8;
	double entry_distance_thresh=0.4;
	double narrow_search_count=0;
	while (not saturate){
		Node* n;
		double distance_to_nn = 0;
		// int r = (int) randomNumber(1,10);
		// cout << r << endl;
		if(closed_search==false){
		while(narrow_search_count<31){
			n = randomConfig(tree);
			if (map->getSize() == 0){
					n->new_node = true;
					map->insert(n);
					new_nodes.push_back(n);
					++count_sample;
					continue;
				}
			
			Node* nn = map->nearestNeighbor(n);
			

			point3d p=n->p;
			std::vector<point3d> wallpoints;
			point3d p_entry=isNarrowPassage(p,wallpoints,tree);
						
			bool valid_point=(!(std::isnan(p_entry.x()) || std::isnan(p_entry.y()) || std::isnan(p_entry.z())));
			if (valid_point) {
			Node* entry_n = new Node(p_entry);
			double distance_to_nn = entry_n->p.distance(nn->p);
				
				if ( nn->entry_explored && (distance_to_nn < distance_thresh)){
				delete entry_n;
				delete n;
	
				}
				
				else if ((nn->iam_entry)  && (distance_to_nn < entry_distance_thresh)){
                delete entry_n;
				delete n;
				}
			

				else{
					entry_n->iam_entry=true;
					for(point3d p_:wallpoints){
					entry_n->wallpoints.push_back(p_);
					}
					entry_n->new_node=true;
					map->insert(entry_n);
					map->addEntry(entry_n);
					edge_nodes.push_back(entry_n);
					new_nodes.push_back(entry_n);
					++count_sample;
                    delete n;

				}

			}
			narrow_search_count++;	
		}
		}

		if (region_saturate or start == NULL ){
		
			int count_failure = 0;
			while (true){
				std::cout<<"value of count_failure"<<count_failure<<std::endl;
				if (count_failure > 2*sample_thresh){
					saturate = true;
					break;
				}
				if(closed_search){
					saturate=true;
					break;
				}
				n = randomConfig(tree);
				if (map->getSize() == 0){
					n->new_node = true;
					map->insert(n);
					new_nodes.push_back(n);
					++count_sample;
					break;
				}
				Node* nn = map->nearestNeighbor(n);
				distance_to_nn = n->p.distance(nn->p);
				cout << "least distance" <<distance_to_nn << endl;
				bool close_search_break=false;
				bool open_search_break=false;

				//if (distance_to_nn < n->thresh_low){
				if(!closed_search){
						
						if ((!nn->iam_entry) && (distance_to_nn < distance_thresh)){
								count_failure=count_failure+2;
								delete n;
								//break;
			                    std::cout<<"stuck at 778"<<std::endl;

						}
						else if ((nn->entry_explored) && (distance_to_nn < distance_thresh)){
								count_failure=count_failure+2;
								delete n;
								//break;
			                    std::cout<<"stuck at 778"<<std::endl;

						}
						else if ((nn->iam_entry) && (distance_to_nn < entry_distance_thresh)){
								count_failure=count_failure+2;
								delete n;
								std::cout<<"stuck at 784"<<std::endl;
						}
						

						else{
					        n->new_node = true;
					        map->insert(n);
					        new_nodes.push_back(n);
					        ++count_sample;
							open_search_break=true;
							std::cout<<"came here at 794"<<std::endl;
				        }
                    

					}




				

				/////to delete the closed search things

				/*else if(closed_search){
					//Node* nn_o= outermap->nearestNeighbor(n);
					bool innermap_collision=checkCollision(tree,n,nn);
					//bool outermap_collision=checkCollision(tree, n, nn_o);
					bool line_condition=checklinecross(n->p,prior_entry_node->p,current_return_point->wallpoints[0],current_return_point->wallpoints[1]);

	
					//distance_to_nn_out = n->p.distance(nn_o->p);
					////checking all conditions

					if ((!nn->iam_entry) && (distance_to_nn < distance_thresh)){
								count_failure=count_failure+2;
								delete n;
								std::cout<<"stuck at 826"<<std::endl;
								//break;
						}
					else if ((nn->iam_entry) && (distance_to_nn < entry_distance_thresh)){
								count_failure=count_failure+2;
								delete n;
								//break;
								std::cout<<"stuck at 833"<<std::endl;
						}
					


					else if(innermap_collision && not outermap_collision){
							count_failure++;
							delete n;
							std::cout<<"stuck at 842"<<std::endl;
							//break;	
				    }

					else if(not innermap_collision && not outermap_collision){
						
						bool nno_line_condition=checklinecross(nn_o->p,prior_entry_node->p,current_return_point->wallpoints[0],current_return_point->wallpoints[1]);
						bool line_condition=checklinecross(n->p,prior_entry_node->p,current_return_point->wallpoints[0],current_return_point->wallpoints[1]);

							if(nno_line_condition){

								n->new_node = true;
								map->insert(n);
								new_nodes.push_back(n);
								++count_sample;
								close_search_break=true;
								std::cout<<"stuck at 858"<<std::endl;
							}
							else if(line_condition){
								n->new_node = true;
								map->insert(n);
								new_nodes.push_back(n);
								++count_sample;
								close_search_break=true;
								std::cout<<"stuck at 866"<<std::endl;
							}
							else{
								++count_failure;
								delete n;
								
							}
				    }
					else {
						 n->new_node = true;
					    map->insert(n);
					    new_nodes.push_back(n);
					    ++count_sample;
						close_search_break=true;

					}
					if(/*(nn==current_return_point) && line_condition && (distance_to_nn>entry_distance_thresh)){
						 n->new_node = true;
					    map->insert(n);
					    new_nodes.push_back(n);
					    ++count_sample;
						close_search_break=true;

					}
	                else if((nn!=current_return_point) &&(not innermap_collision) && (distance_to_nn>distance_thresh)){
						 n->new_node = true;
					    map->insert(n);
					    new_nodes.push_back(n);
					    ++count_sample;
						close_search_break=true;
					}
					else{
								count_failure=count_failure+2;
								delete n;
								
					}

					
					
				}*/
				if( open_search_break){break;}
				std::cout<<"stuck at 889"<<std::endl;
				
				
			}}
		else{
			int count_failure2 = 0;
			//int small_box_entries=0;
			if (start != NULL){
				while (true){
					if (count_failure2 > 2*sample_thresh){
						region_saturate = true;
						break;
					}
					// Bounding Box
					double start_yaw = start->yaw;
					double xmin = start->p.x() - 5;
					double xmax = start->p.x() + 5;
					double ymin = start->p.y() - 5;
					double ymax = start->p.y() + 5;
					double zmin = env_z_min;
					double zmax = env_z_max;
					if( not closed_search){
					if (start_yaw == 0){
						xmin = start->p.x()-2;
						ymax -= 2;
						ymin += 2;
					}
					else if (start_yaw == PI_const/4){
						xmin = start->p.x()-1;
						ymin = start->p.y()-1;
					}
					else if (start_yaw == PI_const/2){
						ymin = start->p.y()-2;
						xmax -= 2;
						xmin += 2;
					}
					else if (start_yaw == 3*PI_const/4){
						xmax = start->p.x()+1;
						ymin = start->p.y()-1;
					}
					else if (start_yaw == PI_const){
						xmax = start->p.x()+2;
						ymax -= 2;
						ymin += 2;
					}
					else if (start_yaw == 5*PI_const/4){
						xmax = start->p.x()+1;
						ymax = start->p.y()+1;
					}
					else if (start_yaw == 3*PI_const/2){
						ymax = start->p.y()+2;
						xmax -= 2;
						xmin += 2;
					}
					else if (start_yaw == 7*PI_const/4){
						xmin = start->p.x()-1;
						ymax = start->p.y()+1;
					}}

					else{
						xmax=start->p.x()+1.0;
						ymax = start->p.y()+1.0;
						xmin=start->p.x()-1.0;
						ymin = start->p.y()-1.0;
					}
					

					std::vector<double> bbx {xmin, xmax, ymin, ymax, zmin, zmax};
					n = randomConfigBBX(tree, bbx);
					Node* nn = map->nearestNeighbor(n);
					distance_to_nn = n->p.distance(nn->p);
					 cout << "least distance: " <<distance_to_nn << endl;
					//if (distance_to_nn < n->thresh_low){
					///introduce it at entry//
					///////////further improvements
					
				bool close_search_break=false;
				bool open_search_break=false;

				//if (distance_to_nn < n->thresh_low){
				if(!closed_search){
						

						if ((!nn->iam_entry) && (distance_to_nn < distance_thresh)){
								count_failure2=count_failure2+2;
								delete n;
								//break;
								//std::cout<<"stuck at 1099"<<std::endl;
						}
						else if ((nn->entry_explored) && (distance_to_nn < distance_thresh)){
								count_failure2=count_failure2+2;
								delete n;
								//break;
			                    //std::cout<<"stuck at 778"<<std::endl;

						}
						else if ((nn->iam_entry) && (distance_to_nn < entry_distance_thresh)){
								count_failure2=count_failure2+2;
								delete n;
								//std::cout<<"stuck at 1104"<<std::endl;
								
						}
						

						else{
					        n->new_node = true;
					        map->insert(n);
					        new_nodes.push_back(n);
					        ++count_sample;
							open_search_break=true;
				        }
                    
						//std::cout<<"stuck at 1116"<<std::endl;
					}






				/////to delete the closed search things

				else if(closed_search){
					
					std::cout<<"came  at 997"<<std::endl;
				    Node* nn_o= outermap->nearestNeighbor(n);
					std::cout<<"came  at 998"<<std::endl;
					bool innermap_collision=checkCollision(tree,n,nn);
					std::cout<<"came  at 1000"<<std::endl;
					//bool outermap_collision=checkCollision(tree, n, nn_o);
					double distance_to_nn_out = n->p.distance(nn_o->p);
					std::cout<<"came  at 1003"<<std::endl;
					bool line_condition=checklinecross(n->p,prior_entry_node->p,current_return_point->wallpoints[0],current_return_point->wallpoints[1]);
					std::cout<<"came  at 1005"<<std::endl;
					if(/*(nn==current_return_point) &&*/ line_condition && (distance_to_nn>entry_distance_thresh) && (distance_to_nn_out>entry_distance_thresh)){
						 n->new_node = true;
					    map->insert(n);
					    new_nodes.push_back(n);
					    ++count_sample;
						close_search_break=true;

					}
	                /*else if((nn!=current_return_point) &&(not innermap_collision) && (distance_to_nn>distance_thresh)){
						 n->new_node = true;
					    map->insert(n);
					    new_nodes.push_back(n);
					    ++count_sample;
						close_search_break=true;
					}*/
					else{
								count_failure2=count_failure2+2;
								delete n;
								
					}

					
					
				}
				if(close_search_break || open_search_break){
					//std::cout<<"came  at 1207"<<std::endl;
					break;}


				}
			}
		}
	}
	cout << "newly added: " << count_sample << " samples" << endl;


	// ========================Connect and Evaluate for new===================================================


	// Check neighbor and add edges///////////////////////////////...................................
	/*if(!closed_search){
	
	
	for (Node* n: new_nodes){
	point3d p=n->p;
	std::vector<point3d> wallpoints;
	point3d p_entry=isNarrowPassage(p,wallpoints,tree);

		if (!(std::isnan(p_entry.x()) || std::isnan(p_entry.y()) || std::isnan(p_entry.z()))) {

			if(isValid(tree,p_entry,false)){
			Node* entry_n = new Node(p_entry);
			entry_n->iam_entry=true;
			entry_n->new_node=true;
			for(point3d p:wallpoints){
			entry_n->wallpoints.push_back(p);
			}
			map->insert(entry_n);
			map->addEntry(entry_n);
			edge_nodes.push_back(entry_n);
			new_nodes.push_back(entry_n);
		    }

		}
	else{continue;}
	}}*/
	/*for(Node* n:new_nodes){
		double x=n->p.x();
		double y=n->p.y();
		double z=n->p.z();
		std::cout<<" x :"<<x<<" y: "<<y<<" z :"<<z<<std::endl;
	}*/
	
	/////////////////////////////////////////////////////////////////////////////,,,,,,,,,,,,,
	for (Node* n: new_nodes){
		// Node* nearest_neighbor = map->nearestNeighbor(n);
		std::vector<Node*> knn = map->kNearestNeighbor(n, 15);
		//std::cout<<"this is line 1075"<<std::endl;
		for (Node* nearest_neighbor: knn){
			//std::cout<<"this is line 1077"<<std::endl;
			if(nearest_neighbor==NULL){
				continue;
			}
			/*std::cout<<"this is line 1094"<<std::endl;
			double x=n->p.x();
		    double y=n->p.y();
		    double z=n->p.z();
		    std::cout<<" x :"<<x<<" y: "<<y<<" z :"<<z<<std::endl;

			double x1=nearest_neighbor->p.x();
		    double y1=nearest_neighbor->p.y();
		    double z1=nearest_neighbor->p.z();
		    std::cout<<" x1 :"<<x1<<" y1: "<<y1<<" z1 :"<<z1<<std::endl;*/
            
		

			bool has_collision = checkCollision(tree, n, nearest_neighbor);
			//std::cout<<"I got debugged here"<<std::endl;

			double distance_to_knn = n->p.distance(nearest_neighbor->p);
			//std::cout<<"never got debugged here"<<std::endl;
			////new addition spars
			/*double high ;
			double low ;
			if(n->thresh_high >=nearest_neighbor->thresh_high){
				high=n->thresh_high;
				low=n->thresh_low;
			}
			else{
				high=nearest_neighbor->thresh_high;
				low=nearest_neighbor->thresh_low;

			}*/
			bool range_condition = sensorRangeCondition(n, nearest_neighbor) and sensorRangeCondition(nearest_neighbor, n);
			//std::cout<<"I died here got debugged here"<<std::endl;
			// if (distance_to_knn < 0.8){
			// 	cout << "bad node" << endl;
			// }
			//if (has_collision == false and distance_to_knn < high and distance_to_knn >low and range_condition == true){
			if (has_collision == false and distance_to_knn < 1.5 and range_condition == true){
				//std::cout<<"this is line 1"<<std::endl;
				n->adjNodes.insert(nearest_neighbor);
				//std::cout<<"this is line 2"<<std::endl;
				nearest_neighbor->adjNodes.insert(n); 
		            //d::cout<<"this is line 3"<<std::endl;
			}
		}


		if (n->adjNodes.size() != 0){
			map->addRecord(n);
			double num_voxels = calculateUnknown(tree, n, dmax);
			//if(n->entry_explored){
			//n->num_voxels =0;}
			//else{
			n->num_voxels=num_voxels;
			//}

		}


		////adding edges between outer & inner map for new nodes generated in closed search ///3 newly added thing
		if(closed_search){
		std::vector<Node*> knn_ = outermap->kNearestNeighbor(n, 15);
		//std::cout<<"this is line 1075"<<std::endl;
		for (Node* nearest_neighbor: knn_){
			//std::cout<<"this is line 1077"<<std::endl;
			if(nearest_neighbor==NULL){
				continue;
			}
		
            
		

			bool has_collision = checkCollision(tree, n, nearest_neighbor);
			

			double distance_to_knn = n->p.distance(nearest_neighbor->p);
			

			
			bool range_condition = sensorRangeCondition(n, nearest_neighbor) and sensorRangeCondition(nearest_neighbor, n);
			
			if (has_collision == false and distance_to_knn < 1.0 and range_condition == true){
				//std::cout<<"this is line 1"<<std::endl;
				n->adjNodes.insert(nearest_neighbor);
				//std::cout<<"this is line 2"<<std::endl;
				nearest_neighbor->adjNodes.insert(n); 
				if(n->entry_explored==true){
					nearest_neighbor->entry_explored=true;
				}
		          
			}
		}
		}

	}

	/////neighbours for entry points 	
	/*for (Node* n: edge_nodes){
		// Node* nearest_neighbor = map->nearestNeighbor(n);
		std::vector<Node*> knn = map->kNearestNeighbor(n, 15);

		for (Node* nearest_neighbor: knn){
			bool has_collision = checkCollision(tree, n, nearest_neighbor);
			double distance_to_knn = n->p.distance(nearest_neighbor->p);
			////new addition spars
			/*double high ;
			double low ;
			if(n->thresh_high >=nearest_neighbor->thresh_high){
				high=n->thresh_high;
				low=n->thresh_low;
			}
			else{
				high=nearest_neighbor->thresh_high;
				low=nearest_neighbor->thresh_low;

			}
			bool range_condition = sensorRangeCondition(n, nearest_neighbor) and sensorRangeCondition(nearest_neighbor, n);
			// if (distance_to_knn < 0.8){
			// 	cout << "bad node" << endl;
			// }
			//if (has_collision == false and distance_to_knn < high and distance_to_knn >low and range_condition == true){
			if (has_collision == false and distance_to_knn < 1.5 and range_condition == true){
				n->adjNodes.insert(nearest_neighbor);
				nearest_neighbor->adjNodes.insert(n); 
			}
		}


		/*if (n->adjNodes.size() != 0){
		
			double num_voxels = calculateUnknown(tree, n, dmax);
			n->num_voxels = num_voxels;
		}

	}*/

	// ==========================Update Old node===============================================================
	// Set cost and heuristics to inf
	int count_update_node = 0;
	int count_actual_update = 0;
	int total_unknown = 0;
	int max_unknown = 0;
	for (Node* n: map->getRecord()){
		n->g = 1000;
		n->f = 1000;
		n->parent = NULL;
		// Check whether the nodes need to update:
		double least_distance;
		bool update = isNodeRequireUpdate(n, path, least_distance);
		if (update and n->new_node==false){
			n->update = true;

			++count_update_node;
			// check the update condition: 
			// 1. if node is very close to trajetory: set it to zero
			// 2. if it is already 0, leave it
			// 3. if it is less than threshold (e.g 100), set it to zero
			// 4. if non of those applies, recalculate it
			double cut_off_value = 5;
			double cut_off_distance = 0.5;
			if (n->num_voxels <= cut_off_value or least_distance <= cut_off_distance){
				n->num_voxels = 0;
				for (double yaw:yaws){
					n->yaw_num_voxels[yaw] = 0;
				}
			}
			else{
				++count_actual_update;
				double num_voxels = calculateUnknown(tree, n, dmax);
				////new added ////
				if(n->entry_explored==false){
				n->num_voxels = num_voxels;}
				else{
					n->num_voxels = 0; 
				}
			}
		}
		else{
			n->update = false;
		}
		n->new_node = false;

		if (n->num_voxels>max_unknown){
			max_unknown = n->num_voxels;
		}
		if(n->iam_entry==false && n->entry_explored==false){
		map->addGoalPQ(n);}
		else if(n->iam_entry && n->entry_explored==false){
		map->addEntryGoal(n);}
		if(closed_search){
			for (Node* adj:n->adjNodes){
				if(adj==map->getRoot()){
					n->entry_explored=true;}
				else{continue;}
				
			}
		}

		total_unknown += n->num_voxels;
	}
	map->setTotalUnknown(total_unknown);
	map->setMaxUnknown(max_unknown);
	std::cout << "Total Number of Unknown Voxels is: " << map->getTotalUnknown() << endl;
	std::cout << "Max Unknown Voxels is: " << map->getMaxUnknown() << endl;
	std::cout << "Number of nodes needed updated is: " << count_update_node << endl;
	std::cout << "Number of actual nodes needed updated is: " << count_actual_update << endl;

	// ====================================VISUALIZATION===============================================

	int node_point_id = 1;
	int entry_point_id = 10000;
	int unknown_voxel_id = 1000;
	std::vector<geometry_msgs::Point> node_vis_array;
	std::vector<geometry_msgs::Point> innernode_vis_array;
	if (VISUALIZE_MAP){
		map_vis_array.clear();
		edge_vis_array.clear();
		inner_map_vis_array.clear();
	}

	if (VISUALIZE_MAP){
		for (Node* n: map->getRecord()){
			
			geometry_msgs::Point p;
			p.x = n->p.x();
			p.y = n->p.y();
			p.z = n->p.z();
			
			visualization_msgs::Marker node_point_vis_marker;

			if( (closed_search==false) && (n->came_from_innermap==false)){
			
			node_point_vis_marker.header.frame_id = "world";
			node_point_vis_marker.id = node_point_id;
			node_point_vis_marker.header.stamp = ros::Time();
			node_point_vis_marker.type = visualization_msgs::Marker::CUBE;
			node_point_vis_marker.action = visualization_msgs::Marker::ADD;
			node_point_vis_marker.pose.position.x = p.x;
			node_point_vis_marker.pose.position.y = p.y;
			node_point_vis_marker.pose.position.z = p.z;
			node_point_vis_marker.scale.x = 0.1;
			node_point_vis_marker.scale.y = 0.1;
			node_point_vis_marker.scale.z = 0.1;
			node_point_vis_marker.color.a = 1.0; 
			node_point_vis_marker.color.r = 1.0;
			node_point_vis_marker.color.g = 0.0;
			node_point_vis_marker.color.b = 0.0;
			if (n->update == true){
				node_point_vis_marker.color.r = 0.0;
				node_point_vis_marker.color.g = 0.0;
				node_point_vis_marker.color.b = 1.0;
			}
			
			


			++node_point_id;
			
			visualization_msgs::Marker unknown_voxel_vis_marker;
			unknown_voxel_vis_marker.header.frame_id = "world";
			unknown_voxel_vis_marker.id = unknown_voxel_id;
			unknown_voxel_vis_marker.header.stamp = ros::Time();
			unknown_voxel_vis_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
			unknown_voxel_vis_marker.action = visualization_msgs::Marker::ADD;
			unknown_voxel_vis_marker.pose.position.x = p.x;
			unknown_voxel_vis_marker.pose.position.y = p.y;
			unknown_voxel_vis_marker.pose.position.z = p.z+0.1;
			unknown_voxel_vis_marker.scale.x = 0.1;
			unknown_voxel_vis_marker.scale.y = 0.1;
			unknown_voxel_vis_marker.scale.z = 0.1;
			unknown_voxel_vis_marker.color.a = 1.0; 
			
			// unknown_voxel_vis_marker.color.r = 1.0;
			// unknown_voxel_vis_marker.color.g = 0.0;
			// unknown_voxel_vis_marker.color.b = 0.0;
			int num = (int) n->num_voxels;
			// int vis_angle =  n->yaw * 180/PI_const;
			// std::string sep = ", ";
			unknown_voxel_vis_marker.text = std::to_string(num); //+ sep + std::to_string(vis_angle);
			++unknown_voxel_id;
			map_vis_array.push_back(node_point_vis_marker);
			map_vis_array.push_back(unknown_voxel_vis_marker);
			
			}

			else{
			node_point_vis_marker.header.frame_id = "world";
			node_point_vis_marker.id = node_point_id;
			node_point_vis_marker.header.stamp = ros::Time();
			node_point_vis_marker.type = visualization_msgs::Marker::SPHERE;
			node_point_vis_marker.action = visualization_msgs::Marker::ADD;
			node_point_vis_marker.pose.position.x = p.x;
			node_point_vis_marker.pose.position.y = p.y;
			node_point_vis_marker.pose.position.z = p.z;
			node_point_vis_marker.scale.x = 0.1;
			node_point_vis_marker.scale.y = 0.1;
			node_point_vis_marker.scale.z = 0.1;
			node_point_vis_marker.color.a = 1.0; 
			node_point_vis_marker.color.r = 0.0;
			node_point_vis_marker.color.g = 0.0;
			node_point_vis_marker.color.b = 0.0;
			node_point_vis_marker.color.b = 0.0;
		

			++node_point_id;
			
			
			// unknown_voxel_vis_marker.color.r = 1.0;
			// unknown_voxel_vis_marker.color.g = 0.0;
			// unknown_voxel_vis_marker.color.b = 0.0;
			//int num = (int) n->num_voxels;
			// int vis_angle =  n->yaw * 180/PI_const;
			// std::string sep = ", ";
			//unknown_voxel_vis_marker.text = std::to_string(num); //+ sep + std::to_string(vis_angle);
			//++unknown_voxel_id;
			inner_map_vis_array.push_back(node_point_vis_marker);
			//map_vis_array.push_back(unknown_voxel_vis_marker)

		}
		for (Node* adj: n->adjNodes){
				
				geometry_msgs::Point p_adj;
				p_adj.x = adj->p.x();
				p_adj.y = adj->p.y();
				p_adj.z = adj->p.z();
				if(closed_search || (n->came_from_innermap && adj->came_from_innermap)){
				innernode_vis_array.push_back(p);
				innernode_vis_array.push_back(p_adj);}
				else{
				node_vis_array.push_back(p);
				node_vis_array.push_back(p_adj);	
				}
		
		
		}}

		/////to add edge nodes ---
		for (Node* n: map->getEntry()){
			geometry_msgs::Point p;
			p.x = n->p.x();
			p.y = n->p.y();
			p.z = n->p.z();
			visualization_msgs::Marker edge_point_vis_marker;
			edge_point_vis_marker.header.frame_id = "world";
			edge_point_vis_marker.id = entry_point_id;
			edge_point_vis_marker.header.stamp = ros::Time();
			edge_point_vis_marker.type = visualization_msgs::Marker::CUBE;
			edge_point_vis_marker.action = visualization_msgs::Marker::ADD;
			edge_point_vis_marker.pose.position.x = p.x;
			edge_point_vis_marker.pose.position.y = p.y;
			edge_point_vis_marker.pose.position.z = p.z;
			edge_point_vis_marker.scale.x = 0.3;
			edge_point_vis_marker.scale.y = 0.3;
			edge_point_vis_marker.scale.z = 0.3;
			edge_point_vis_marker.color.a = 1.0; 
			edge_point_vis_marker.color.r = 1.0;
			edge_point_vis_marker.color.g = 1.0;
			edge_point_vis_marker.color.b = 1.0;
			

			++entry_point_id;

			edge_vis_array.push_back(edge_point_vis_marker);
			
			for (Node* adj: n->adjNodes){
				geometry_msgs::Point p_adj;
				p_adj.x = adj->p.x();
				p_adj.y = adj->p.y();
				p_adj.z = adj->p.z();
				node_vis_array.push_back(p);
				node_vis_array.push_back(p_adj);
			}
		}

		////visualization of edges
		if(!closed_search){
		visualization_msgs::Marker node_vis_marker;
		node_vis_marker.header.frame_id = "world";
		node_vis_marker.points = node_vis_array;
		node_vis_marker.id = 0;
		node_vis_marker.type = visualization_msgs::Marker::LINE_LIST;
		node_vis_marker.scale.x = 0.05;
		node_vis_marker.scale.y = 0.05;
		node_vis_marker.scale.z = 0.05;
		node_vis_marker.color.a = 1.0;
		node_vis_marker.color.r = 0.0;
		node_vis_marker.color.g = 1.0;
		node_vis_marker.color.b = 0.0;
		// cout << "NODE VIS ARRAY SIZE: "<< node_vis_array.size() << endl;
		map_vis_array.push_back(node_vis_marker);}
        else{
		visualization_msgs::Marker innernode_vis_marker;
		innernode_vis_marker.header.frame_id = "world";
		innernode_vis_marker.points = innernode_vis_array;
		innernode_vis_marker.id = 0;
		innernode_vis_marker.type = visualization_msgs::Marker::LINE_LIST;
		innernode_vis_marker.scale.x = 0.05;
		innernode_vis_marker.scale.y = 0.05;
		innernode_vis_marker.scale.z = 0.05;
		innernode_vis_marker.color.a = 1.0;
		innernode_vis_marker.color.r = 1.0;
		innernode_vis_marker.color.g = 1.0;
		innernode_vis_marker.color.b = 1.0;
		// cout << "NODE VIS ARRAY SIZE: "<< node_vis_array.size() << endl;
		inner_map_vis_array.push_back(innernode_vis_marker);

		}



	}

	return map;
}
PRM* buildinnermap(PRM* map,Node* n){
	map->insert(n);
	return map;
}


// ===================Visualization============================


///////////four cases ---map /