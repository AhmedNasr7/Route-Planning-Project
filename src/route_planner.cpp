#include "route_planner.h"
#include <algorithm>

bool compare_node(RouteModel::Node* node1, RouteModel::Node* node2) // helper function for sort method
{
  auto f1 = node1->g_value + node1->h_value;
  auto f2 = node2->g_value + node2->h_value;
  return f1 > f2;
}


RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

  

    this->start_node = &m_Model.FindClosestNode(start_x, start_y);
    this->end_node = &m_Model.FindClosestNode(end_x, end_y);

 
}


float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {

	return node->distance(*end_node);

}


void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {

	current_node->FindNeighbors();
	for(auto node : current_node->neighbors)
	{
		node->parent = current_node;
		node->g_value = current_node->g_value + node->distance(*current_node);
		node->h_value = CalculateHValue(node);
      	node->visited = true;
		open_list.push_back(node);
    }
}



RouteModel::Node *RoutePlanner::NextNode() {

  
  std::sort(open_list.begin(), open_list.end(), compare_node);


	RouteModel::Node *next_node = open_list.back();
	open_list.pop_back();


	return next_node;

}



std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // - looping through the nodes and adding them to the path_found vector

    while(current_node->parent != nullptr)
    {
    	path_found.push_back(*current_node);
    	distance += current_node->distance(*(current_node->parent));
    	current_node = current_node->parent;
    }

    

    path_found.push_back(*current_node); // push start node

    std::reverse(path_found.begin(), path_found.end()); 


    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.


    return path_found;

}




void RoutePlanner::AStarSearch() {

  RouteModel::Node *current_node = nullptr;

  
  current_node = start_node;
  current_node->visited = true;
  open_list.emplace_back(current_node);
   

    while(open_list.size() > 0)
    {

    	
      current_node = NextNode();

    	if (current_node->distance(*end_node) == 0)
    	{
    		m_Model.path =  ConstructFinalPath(end_node);
          	return;
    	}

    	AddNeighbors(current_node);


    }
  

}