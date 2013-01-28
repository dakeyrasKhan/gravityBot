#include "roadmap.hpp"


Path Roadmap::getPath(Position start, Position end){
	vector<Node> neighbours;
	findNeighbours(start.ToPoint(),scene->maxSize(),&tree,&neighbours);
	priority_queue<NodeComp> heap;

	for(auto neighbour : neighbours){
		if(scene->validMove(start,neighbour.pos))
			heap.push(NodeComp(neighbour,Position(neighbour.pos-start).Norm()));
	}
	Path path;
	
	bool *seen = new bool[waypoints.size()];
	double *distances = new double[waypoints.size()];
	int *father = new int[waypoints.size()];

	for(int i=0;i<waypoints.size();i++){
		seen[i]=false;
		distances[i]=INFINITY;
		father[i]=-1;
	}

	while(!heap.empty()){
		Node current = heap.top().node;
		heap.pop();
		if(seen[current.id])
			continue;
		for(auto neighbour : adjacency[current.id]){
			double dist = Position(current.pos-neighbour.pos).Norm();
			if(distances[current.id] + dist < distances[neighbour.id]){
				distances[neighbour.id] = distances[current.id] + dist;
				father[neighbour.id] = current.id;
				heap.push(NodeComp(neighbour,dist));
			}
		}
	}
	neighbours.clear();
	findNeighbours(end.ToPoint(),scene->maxSize(),&tree,&neighbours);
	int endPoint=-1;
	double distToEnd=INFINITY;
	for(auto neighbour : neighbours){
		//Si on peut l'atteindre depuis le début
		if(distances[neighbour.id]<INFINITY){
			//Si on peut atteindre la fin depuis
			if(scene->validMove(end,neighbour.pos)){
				//Si c'est mieux
				if(distances[neighbour.id]+Position(neighbour.pos-end).Norm()<distToEnd){
					distToEnd=distances[neighbour.id]+Position(neighbour.pos-end).Norm();
					endPoint=neighbour.id;
				}
			}
		}
	}
	if(endPoint==-1){
		delete[] father;
		delete[] distances;
		delete[] seen;
		return path;
	}

	std::stack<int> idPathStack;
	int current = endPoint;
	while(current!=-1){
		idPathStack.push(current);
		current=father[current];
	}
	path.add(start);
	while(!idPathStack.empty()){
		path.add(waypoints[idPathStack.top()]);
		idPathStack.pop();
	}
	path.add(end);
	delete[] father;
	delete[] distances;
	delete[] seen;
	return path;
}

void Roadmap::explore(int curNode, int curClasse){
	if(classes[curNode]!=-1)
		return;
	classes[curNode]=curClasse;
	for(auto neighbour : adjacency[curNode])
		explore(neighbour.id,curClasse);
}

Roadmap::Roadmap(Scene* scene){
	int idCount=0;
	while(waypoints.size()<NB_WAYPOINTS){
		
		Node node(Position::Random(scene->size),idCount++);

		if(scene->collision(node.pos))
			continue;
		
		addPos(node,&tree);
		waypoints.push_back(node.pos);
		classes.push_back(-1);	
		vector<Node> neighbours;
		findNeighbours(node.pos.ToPoint(),scene->maxSize(),&tree,&neighbours);
		vector<int> newAdjacent;
		double fail=0;
		for(auto neighbour : neighbours){
			if(scene->validMove(neighbour.pos,node.pos)){
				newAdjacent.push_back(neighbour.id);
				adjacency[neighbour.id].push_back(node);
			}
			else
				fail++;
		}
		failRate.push_back(fail/double(neighbours.size()));
	}
	int curClasse=0;
	for(int i=0;i<classes.size();i++){
		if(classes[i]!=-1)
			continue;
		explore(i,curClasse++);
	}
}