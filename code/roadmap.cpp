#include "roadmap.hpp"


Path Roadmap::getPath(Position start, Position end, bool with,Point *pos=NULL){
	vector<FullNode> neighbours;
	findNeighbours(start.ToPoint(),scene->maxSize(),with,&tree,&neighbours);
	priority_queue<NodeComp> heap;

	for(auto neighbour : neighbours){
		if(scene->validMove(start,neighbour.pos,with,pos))
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
		FullNode current = heap.top().node;
		heap.pop();
		if(seen[current.id])
			continue;
		for(auto neighbour : adjacency[current.id]){
			if(pos!=NULL){
				if(!scene->validMove(current.pos,neighbour.pos,with,pos))
					continue;
			}
			double dist = Position(current.pos-neighbour.pos).Norm();
			if(distances[current.id] + dist < distances[neighbour.id]){
				distances[neighbour.id] = distances[current.id] + dist;
				father[neighbour.id] = current.id;
				heap.push(NodeComp(neighbour,dist));
			}
		}
	}
	neighbours.clear();
	findNeighbours(end.ToPoint(),scene->maxSize(),with,&tree,&neighbours);
	int endPoint=-1;
	double distToEnd=INFINITY;
	for(auto neighbour : neighbours){
		//Si on peut l'atteindre depuis le début
		if(distances[neighbour.id]<INFINITY){
			//Si on peut atteindre la fin depuis
			if(scene->validMove(end,neighbour.pos,with,pos)){
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
		path.add(waypoints[idPathStack.top()].pos);
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

void Roadmap::addNode(FullNode node){
	
	if(scene->Collision(node.pos,node.with,NULL))
		return;

	addPos(node,&tree);	
	waypoints.push_back(node);
	adjacency.push_back(vector<FullNode>());		

	vector<FullNode> neighbours;
	findNeighbours(node.pos.ToPoint(),scene->maxSize(),node.with,&tree,&neighbours);
	double fail=0;
	for(auto neighbour : neighbours){
		if(neighbour.pos == node.pos && 
		   scene->validMove(neighbour.pos,node.pos,node.with,NULL)){
				adjacency[neighbour.id].push_back(node);
		}
		else
			fail++;
	}
	failRate.push_back(fail/double(neighbours.size()));
}


Roadmap::Roadmap(Scene* scene){
	int idCount=0;
	for(int i=0;i<2;i++){
		while(waypoints.size()<NB_WAYPOINTS){
			FullNode node(Position::Random(scene->size),idCount++,(i<1));
			addNode(node);
		}
		/*int curClasse=0;
		for(int i=0;i<classes.size();i++){
			if(classes[i]!=-1)
				continue;
			explore(i,curClasse++);
		}
		nbClasses = curClasse;*/
	}
	for(auto node : waypoints){
		if(!node.with)
			continue;
		Point pos=scene->Drop(node.pos);
		for(int i=0;i<NB_DROP;i++){
			Position r = randomCatch(pos);
			if(scene->Collision(r,false,NULL))
				continue;
			Path p = getPath(node.pos,r,false,&pos);
			if(!p.empty()){
				drop[node.id]=p;
				addNode(FullNode(r,waypoints.size(),true));
				adjacency[node.id].push_back(FullNode(r,waypoints.size()-1,true));
				break;
			}
		}

	}	
}