#include <ros/ros.h>
#include <DEP/prm.h>
#include <DEP/multi_astar.h>
#include "gurobi_c++.h"


void test_tsdf(OcTree &tree, voxblox::EsdfServer &voxblox_server);
double interpolateNumVoxels(Node* n, double yaw);
std::vector<Node*> findBestPath(PRM* roadmap,
                                Node* start,
                                std::vector<Node*> goal_candidates,
                                OcTree& tree,
                                bool replan);
std::vector<Node*> getGoalCandidates(PRM* roadmap);
Node* findStartNode(PRM* map, Node* current_node, OcTree& tree);
double calculatePathLength(std::vector<Node*> path);
double calculatePathRotation(std::vector<Node*> path, double current_yaw);
double findDistanceToWall(double x, double y, double z, voxblox::EsdfServer &voxblox_server);

int main(int argc, char** argv){
	ros::init(argc, argv, "test_tsdf");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	// tsdf map
	voxblox::EsdfServer voxblox_server(nh, nh_private);
	voxblox_server.loadMap("/home/zhefan/catkin_ws/src/DEP/src/test/cafe_voxblox.vxblx");
	// octomap
	OcTree tree (0.2);
	tree.readBinary("/home/zhefan/catkin_ws/src/DEP/src/test/cafe_octree.bt");
	test_tsdf(tree, voxblox_server);
	return 0;
}

void test_tsdf(OcTree &tree, voxblox::EsdfServer &voxblox_server){
	cout << "tree resolution: " << tree.getResolution() << endl;
	Eigen::Vector3d p_test (0.3, 6, 1);
	double distance = 0;
	// bool success = voxblox_server.getEsdfMapPtr()->getDistanceAtPosition(p_test, &distance);
	// if (success){
	// 	cout << "distance to wall: " << distance << endl;
	// }
	Node current_pose (point3d (0.3, 6, 1));
	PRM* roadmap = new PRM ();
	std::vector<Node*> last_path;
	roadmap = buildRoadMap(tree, roadmap, last_path);
	Node* start = findStartNode(roadmap, &current_pose, tree);
	std::vector<Node*> goal_candidates = getGoalCandidates(roadmap);
	bool replan = false;
	std::vector<Node*> best_path = findBestPath(roadmap, start, goal_candidates, tree, replan);
	print_node_vector(best_path);
}

double findDistanceToWall(double x, double y, double z, voxblox::EsdfServer &voxblox_server){
	Eigen::Vector3d p (x, y, z);
	double distance = 0;
	bool success = voxblox_server.getEsdfMapPtr()->getDistanceAtPosition(p, &distance);
	if (success){
		return distance;
	}
	else{
		return 1000000;
	}
}

double calculatePathLength(std::vector<Node*> path){
	int idx1 = 0;
	double length = 0;
	for (int idx2=1; idx2<=path.size()-1; ++idx2){
		length += path[idx2]->p.distance(path[idx1]->p);
		++idx1;
	}
	return length;
}

double calculatePathRotation(std::vector<Node*> path, double current_yaw){
	int idx1 = 0;
	double rotation = 0;
	double start_yaw = path[idx1]->yaw;
	double dyaw_start = start_yaw - current_yaw;
	if (std::abs(dyaw_start) < PI_const){
		rotation += std::abs(dyaw_start);
	}
	else{
		rotation += 2*PI_const - std::abs(dyaw_start);
	}

	for (int idx2=1; idx2<=path.size()-1; ++idx2){
		double this_yaw = path[idx1]->yaw;
		double next_yaw = path[idx2]->yaw;
		double dyaw = next_yaw - this_yaw;
		if (std::abs(dyaw) < PI_const){
			rotation += std::abs(dyaw);
		}
		else{
			rotation += 2*PI_const - std::abs(dyaw);
		}
		++idx1;
	}
	return rotation;
}

Node* findStartNode(PRM* map, Node* current_node, OcTree& tree){
	std::vector<Node*> knn = map->kNearestNeighbor(current_node, 5);
	for (Node* n: knn){
		bool has_collision = checkCollision(tree, n, current_node);
		if (not has_collision){
			return n;
		}
	}
	cout << "No Valid Node for Start" << endl;
	return NULL;
}

std::vector<Node*> getGoalCandidates(PRM* roadmap){
	std::vector<Node*> goal_candidates;
	double thresh = 0.1;
	double min_number = 10;
	double max_num_voxels = 0;
	// Go over node which has number of voxels larger than 0.5 maximum
	bool first_node = true;
	std::priority_queue<Node*, std::vector<Node*>, GainCompareNode> goal_nodes = roadmap->getGoalNodes();
	while (true){
		Node* n = goal_nodes.top();
		goal_nodes.pop();

		if (n->num_voxels < max_num_voxels * thresh){
			break;
		}

		if (first_node){
			max_num_voxels = n->num_voxels;
			first_node = false;
		}
		goal_candidates.push_back(n);
	}

	if (goal_candidates.size() < min_number){
		Node* n = goal_nodes.top();
		goal_nodes.pop();
		goal_candidates.push_back(n);
	}

	return goal_candidates;
}

// Helper Function:
double interpolateNumVoxels(Node* n, double yaw){
	// Find the interval then interpolate
	double min_yaw, max_yaw;
	for (int i=0; i<yaws.size()-1; ++i){
		if (yaw > yaws[i] and yaw < yaws[i+1]){
			min_yaw = yaws[i];
			max_yaw = yaws[i+1];
			break;
		}
	}

	// Interpolate
	std::map<double, int> yaw_num_voxels = n->yaw_num_voxels;
	double num_voxels = yaw_num_voxels[min_yaw] + (yaw - min_yaw) * (yaw_num_voxels[max_yaw]-yaw_num_voxels[min_yaw])/(max_yaw - min_yaw);
	// cout << "max angle: " << max_yaw << endl;
	// cout << "min_angle: " << min_yaw << endl;
	// cout << "yaw: " << yaw << endl;
	// cout << "max: " << yaw_num_voxels[max_yaw] << endl;
	// cout << "min: " << yaw_num_voxels[min_yaw] << endl;
	return num_voxels;
}

std::vector<Node*> findBestPath(PRM* roadmap,
                                Node* start,
                                std::vector<Node*> goal_candidates,
                                OcTree& tree,
                                bool replan=false){
	double linear_velocity = 0.3;
	double angular_velocity = 0.8;
	double current_yaw = start->yaw;
	double score_thresh = 0.5;

	std::vector<Node*> final_path;
	// 1. Generate all the path
	std::vector<std::vector<Node*>> path_vector;
	std::vector<double> path_score; // Score = num_voxels/time
	std::vector<Node*> candidate_path;
	int count_path_id = 0;
	for (Node* goal: goal_candidates){
		candidate_path = AStar(roadmap, start, goal, tree, replan);	

		
		// find the appropriate yaw for nodes:
	  		// 1. sensor range from node in this yaw can see next node
		double score = 0;
		double total_num_voxels = 0;
		double total_unknwon;
		for (int i=0; i<candidate_path.size(); ++i){
			// last one
			if (i == candidate_path.size()-1){
				double max_yaw = 0;
				double max_voxels = 0;
				total_unknwon = calculateUnknown(tree, candidate_path[i], dmax);
				for (double yaw: yaws){
					if (candidate_path[i]->yaw_num_voxels[yaw] > max_voxels){
						max_voxels = candidate_path[i]->yaw_num_voxels[yaw];
						max_yaw = yaw;
					}
				}
				candidate_path[i]->yaw = max_yaw;
				total_num_voxels += max_voxels;
			}
			else{

				Node* this_node = candidate_path[i];
				Node* next_node = candidate_path[i+1];
				point3d direction = next_node->p - this_node->p;
				double node_yaw = atan2(direction.y(), direction.x());
				if (node_yaw < 0){
					node_yaw = 2*PI_const - (-node_yaw);
				}
				total_num_voxels += interpolateNumVoxels(this_node, node_yaw);
				// cout << "interpolate result: " << interpolateNumVoxels(this_node, node_yaw) << endl;
			}
		}

		if (candidate_path.size() != 0){
			for (int i=0; i<candidate_path.size()-1; ++i){
				Node* this_node = candidate_path[i];
				Node* next_node = candidate_path[i+1];
				point3d direction = next_node->p - this_node->p;
				double node_yaw = atan2(direction.y(), direction.x());
				if (node_yaw < 0){
					node_yaw = 2*PI_const - (-node_yaw);
				}
				candidate_path[i]->yaw = node_yaw;
			}
			double total_path_length = calculatePathLength(candidate_path);
			double total_path_rotation = calculatePathRotation(candidate_path, current_yaw);
			double total_time = total_path_length/linear_velocity + total_path_rotation/angular_velocity;
			score = total_num_voxels/total_time;
			// cout << "Path " << count_path_id << " score: " << score << " length: " << total_path_length << " Rotation: " << total_path_rotation << " Time: " << total_time <<  " Voxels: " << total_num_voxels << endl;
		}
		path_score.push_back(score);
		path_vector.push_back(candidate_path);
		++count_path_id;
	}






	// Find best score
	double best_score = 0;
	double best_idx = 0;
	for (int i=0; i<path_vector.size(); ++i){
		if (path_score[i] > best_score){
			best_score = path_score[i];
			best_idx = i;
		}
	}
	// cout << "initial_best_idx: " << best_idx << endl;

	// // Filter the score which is larger than threshold
	// std::vector<std::vector<Node*>> final_candidate_path;
	// std::vector<double> final_candidate_score;
	// for (int i=0; i<path_vector.size(); ++i){
	// 	if (path_score[i] > score_thresh * best_score){
	// 		final_candidate_path.push_back(path_vector[i]);
	// 		final_candidate_score.push_back(path_score[i]);
	// 	}
	// }


	// // for each of them calculate the total distance to reach other
	// std::vector<double> total_distance_to_other;
	// double shortest_distance = 100000;
	// for (int i=0; i<final_candidate_path.size(); ++i){
	// 	// calculate the total distance to other path
	// 	double total_distance = 0;
	// 	for (int j=0; j<final_candidate_path.size(); ++j){
	// 		if (j != i){
	// 			std::vector<Node*> path_to_other = AStar(roadmap, *(final_candidate_path[i].end()-1), *(final_candidate_path[j].end()-1), tree, false);
	// 			total_distance += calculatePathLength(path_to_other);
	// 		}
	// 	}
	// 	if (total_distance < shortest_distance){
	// 		shortest_distance = total_distance;
	// 	}
	// 	cout << "path: " << i <<" raw score: " << final_candidate_score[i] << endl;
	// 	cout << "path: " << i << " total distance: " << total_distance <<endl;
	// 	total_distance_to_other.push_back(total_distance);
	// }	


	// // Update final score
	// for (int i=0; i<final_candidate_path.size(); ++i){
	// 	double discount_factor = total_distance_to_other[i]/shortest_distance;
	// 	final_candidate_score[i] /= discount_factor;
	// 	cout << "path: " << i << " discount factor: " << discount_factor << endl;
	// 	cout << "path: " << i << " updated score: " << final_candidate_score[i] << endl;
	// }

	// // Find the best idx
	// double final_best_score = 0;
	// best_idx = 0;
	// for (int i=0; i<final_candidate_path.size(); ++i){
	// 	if (final_candidate_score[i] > final_best_score){
	// 		final_best_score = final_candidate_score[i];
	// 		best_idx = i;
	// 	}
	// }

	// cout << "best_idx: " << best_idx << endl;
	// Get the best path
	final_path = path_vector[best_idx];
	for (int i=0; i<final_path.size()-1; ++i){
		Node* this_node = final_path[i];
		Node* next_node = final_path[i+1];
		point3d direction = next_node->p - this_node->p;
		double node_yaw = atan2(direction.y(), direction.x());
		if (node_yaw < 0){
			node_yaw = 2*PI_const - (-node_yaw);
		}
		final_path[i]->yaw = node_yaw;
	}
	// cout << "final id: " << best_idx << endl;
	// cout << "final path size: " << final_path.size() << endl;
	return final_path;
}