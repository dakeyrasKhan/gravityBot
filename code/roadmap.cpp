#include "roadmap.hpp"

Roadmap::Roadmap(Scene* scene){
	int idCount=0;
	while(waypoints.size()<NB_WAYPOINTS){
		
		Node node;
		node.pos=Position::Random(scene->size);
		node.id=idCount++;

		if(scene->collision(node.pos))
			continue;
		
		addPos(node,&tree);
		
		vector<Node> neighbours;
		findNeighbours(node.pos.ToPoint(),scene->maxSize(),&tree,&neighbours);
		vector<int> newAdjacent;
		double fail=0;
		for(auto neighbour : neighbours){
			if(scene->validMove(neighbour.pos,node.pos)){
				newAdjacent.push_back(neighbour.id);
				adjacency[neighbour.id].push_back(node.id);
			}
			else
				fail++;
		}
		failRate.push_back(fail/double(neighbours.size()));
	}
}