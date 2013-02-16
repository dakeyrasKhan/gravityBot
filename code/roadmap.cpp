#include "roadmap.hpp"
#include <iostream>
#include <exception>

Path Roadmap::getPath(Position start, Position end, bool with,Point *pos=NULL){
	vector<FullNode> neighbours;
	Path path;

	findNeighbours(start.ToPoint(),scene->MaxSize(), with,&tree,&neighbours);
	std::cout<<"Found "<<neighbours.size()<<" potential start neighbours"<<std::endl;
	
	priority_queue<NodeComp> heap;
	bool *seen = new bool[waypoints.size()];
	double *distances = new double[waypoints.size()];
	int *father = new int[waypoints.size()];

	for(int i=0;i<waypoints.size();i++){
		seen[i]=false;
		distances[i]= INFINITY;
		father[i]=-1;
	}
	

	for(auto neighbour : neighbours){

		if(scene->ValidMove(start,neighbour.pos, IGNORE_BALL_COLLISION)){
			distances[neighbour.id]=Position(neighbour.pos-start).Norm();
			heap.push(NodeComp(neighbour,Position(neighbour.pos-start).Norm()));
		}
		else{
			std::cout<<"Not Valid !"<<std::endl;
			std::cout<<"Start : "<<start[0]<<","<<start[1]<<","<<start[2]<<","<<start[3]<<","<<start[4]<<","<<start[5]<<","<<start[6]<<","<<start[7]<<std::endl;
			std::cout<<"End : "<<neighbour.pos[0]<<","<<neighbour.pos[1]<<","<<neighbour.pos[2]<<","<<neighbour.pos[3]<<","<<neighbour.pos[4]<<","<<neighbour.pos[5]<<","<<neighbour.pos[6]<<","<<neighbour.pos[7]<<std::endl;
		}
	}
	
	
	std::cout<<"starting dijkstra with "<<heap.size()<<" nodes"<<std::endl;

	while(!heap.empty()){
		FullNode current = heap.top().node;
		heap.pop();
		if(seen[current.id])
			continue;
		seen[current.id]=true;
		for(auto neighbour : adjacency[current.id]){

			if(pos!=NULL){
				if(!scene->ValidMove(current.pos,neighbour.pos))
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
	std::cout<<"dijkstra ended"<<std::endl;
	neighbours.clear();
	findNeighbours(end.ToPoint(),scene->MaxSize(),with,&tree,&neighbours);
	std::cout<<"found "<<neighbours.size()<<" possible end"<<std::endl;
	int endPoint=-1;
	double distToEnd=INFINITY;
	for(auto neighbour : neighbours){
		//Si on peut l'atteindre depuis le début
		if(distances[neighbour.id]<INFINITY){
			//Si on peut atteindre la fin depuis
			if(scene->ValidMove(end,neighbour.pos)){
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
	std::cout<<"found end"<<std::endl;
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

void Roadmap::addNode(FullNode node, int flags){
	
	if(scene->Collision(node.pos,flags)){
		//std::cout<<"Collision : "<<node.pos[0]<<","<<node.pos[1]<<","<<node.pos[2]<<","<<node.pos[3]<<","<<node.pos[4]<<","<<node.pos[5]<<","<<node.pos[6]<<","<<node.pos[7]<<std::endl;
		//std::cout<<"collision"<<std::endl;
		return;
	}
	//std::cout<<"adding pos in tree"<<std::endl;
	addPos(node,&tree);	
	//std::cout<< "pos added"<<std::endl;

	waypoints.push_back(node);
	adjacency.push_back(vector<FullNode>());		

	vector<FullNode> neighbours;
	//std::cout<<"finding neighbours"<<std::endl;
	findNeighbours(node.pos.ToPoint(),scene->MaxSize(),node.with,&tree,&neighbours);
	//std::cout<< "found "<<neighbours.size()<<" neighbours"<<std::endl;
	double fail=0;
	for(auto neighbour : neighbours){
		//std::cout<<"Cheking validMove"<<std::endl;
		if(neighbour.with == node.with && 
		   scene->ValidMove(neighbour.pos,node.pos)){
				adjacency[neighbour.id].push_back(node);
		}
		else
			fail++;
		//std::cout<<"done"<<std::endl;
	}
	failRate.push_back(fail/double(neighbours.size()));
}


Roadmap::Roadmap(Scene* scene):scene(scene),tree(scene->NegSize().ToPoint(),scene->PosSize().ToPoint()){
	int prec=0;
	// On fait 2 roadmap, une avec le robot tenant l'objet, et une sans
	for(int i=0;i<2;i++){
		std::cout<<"pass #"<<i<<std::endl;
		while(waypoints.size()<NB_WAYPOINTS/(2-i)){
			Position randompos=Random(scene->NegSize(),scene->PosSize(),i,scene->robot);
			FullNode node(randompos,waypoints.size(),(i==1));
			//std::cout<<"adding node"<<std::endl;
			// Lors de la deuxième passe, on ajoute le flag BALL_ON_ARM
			addNode(node,i*TRANSPORTING_BALL);
			//std::cout<<"node added\n"<<std::endl;
			if(waypoints.size()%100==0 && waypoints.size()!=prec){
				std::cout<<waypoints.size()<<std::endl;
				prec=waypoints.size();
			}
		}
	}

	// Pour chaque waypoints du robot tenant l'objet
	for(auto node : waypoints){
		if(!node.with)
			continue;
		//On lache l'objet
		Point pos;
		try{
			pos=scene->Drop(node.pos);
		}
		catch(NoDropPointException e){
			std::cout<<"no drop"<<std::endl;
			continue;
		}
		//si on peut l'attraper directement sans se prendre d'obstacle, c'est pas intéressant
		if(scene->ValidMove(node.pos,scene->robot.Catch(node.pos,pos), IGNORE_BALL_COLLISION)){
			std::cout<<"not interesting"<<std::endl;
			continue;
		}

		for(int i=0;i<NB_DROP;i++){
			//On essaie de l'attraper
			Position r = scene->robot.RandomCatch(pos);
			if(scene->Collision(r,IGNORE_BALL_COLLISION)){
				//std::cout<<"failed catch "<<std::endl;
				continue;
			}

			//// A VERIFIER ! C'est là qu'on local plan
			Path p = getPath(node.pos,r,false,&pos);
			
			if(!p.empty()){
				std::cout<<"CHOPPE"<<std::endl;
				drop[node.id]=p;
				addNode(FullNode(r,waypoints.size(),true), IGNORE_BALL_COLLISION);
				adjacency[node.id].push_back(FullNode(r,waypoints.size()-1,true));
				break;
			}
		}

	}	
}