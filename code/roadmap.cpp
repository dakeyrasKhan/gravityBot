#include "roadmap.hpp"

Roadmap::Roadmap(Scene* scene){
	while(waypoints.size()<NB_WAYPOINTS){
		Position nouveau=Position::Random(scene->size);
		if(scene->collision(nouveau))
			continue;
		addPos(nouveau,&tree);
		int idNouveau = tree.values.size()-1;
		vector<Position> neighbours;
		findNeighbours(nouveau.toPoint(),scene->maxSize(),&tree,&neighbours);
		vector<int> newAdjacent;
		double fail=0;
		for(auto neighbour : neighbours){
			if(scene->validMove(neighbour.pos,nouveau.pos)){
				newAdjacent.push_back(neighbour.id);
				adjacency[neighbour.id].push_back(idNouveau);
			}
			else
				fail++;
		}
		failRate.push_back(fail/double(neighbours.size()));
	}
}