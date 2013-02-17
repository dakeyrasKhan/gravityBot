#include "roadmap.hpp"
#include <iostream>
#include <exception>

Path Roadmap::getPath(FullNode _start, FullNode _end,Point *pos=NULL){

	bool ballMove = (_start.pos[BALL_X]!=_end.pos[BALL_X]) || (_start.pos[BALL_Y]!=_end.pos[BALL_Y]) || (_start.pos[BALL_Z]!=_end.pos[BALL_Z]);
	vector<FullNode> neighbours;
	Path path;

	//if start.ballPos != end.ballPos et start!= with, il faut chopper à la main
	FullNode start=_start;
	Path startPath,endPath;
	if(ballMove && !_start.with){
		Point ballpos;
		ballpos[X]=_start.pos[BALL_X];ballpos[Y]=_start.pos[BALL_Y];ballpos[Z]=_start.pos[BALL_Z];
		for(int _try=0;_try<NB_TRY;_try++){
			Position r = scene->robot.RandomCatch(ballpos);
			Path p=OkCatch(_start.pos,r,&ballpos);
			if(!p.empty()){
				start=FullNode(r,-1,true);
				startPath=p;
				break;
			}
		}
	}


	findNeighbours(start.pos.ToPoint(),scene->MaxSize(), start.with,&tree,&neighbours);
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

		if(scene->ValidMove(start.pos,neighbour.pos, IGNORE_BALL_COLLISION)){
			distances[neighbour.id]=Position(neighbour.pos-start.pos).Norm();
			heap.push(NodeComp(neighbour,Position(neighbour.pos-start.pos).Norm()));
		}
		else{
			/*
			std::cout<<"Not Valid !"<<std::endl;
			std::cout<<"Start : "<<start.pos[0]<<","<<start.pos[1]<<","<<start.pos[2]<<","<<start.pos[3]<<","<<start.pos[4]<<","<<start.pos[5]<<","<<start.pos[6]<<","<<start.pos[7]<<std::endl;
			std::cout<<"End : "<<neighbour.pos[0]<<","<<neighbour.pos[1]<<","<<neighbour.pos[2]<<","<<neighbour.pos[3]<<","<<neighbour.pos[4]<<","<<neighbour.pos[5]<<","<<neighbour.pos[6]<<","<<neighbour.pos[7]<<std::endl;*/
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
				if(!scene->ValidMove(current.setBall(pos),neighbour.setBall(pos),TAKING_BALL)){
					continue;
				}
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

	FullNode end=_end;

	if(ballMove && !_end.with){
		Point ballpos;
		ballpos[X]=_end.pos[BALL_X];ballpos[Y]=_end.pos[BALL_Y];ballpos[Z]=_end.pos[BALL_Z];
		for(int _try=0;_try<NB_TRY;_try++){
			Position r = scene->robot.RandomDrop(ballpos);
			Path p = OkCatch(r,_end.pos,&ballpos);
			if(!p.empty()){
				endPath=p;
				end=FullNode(r,-1,true);
				break;
			}
		}
	}

	neighbours.clear();
	findNeighbours(end.pos.ToPoint(),scene->MaxSize(),end.with,&tree,&neighbours);
	std::cout<<"found "<<neighbours.size()<<" possible end"<<std::endl;
	int endPoint=-1;
	double distToEnd=INFINITY;
	for(auto neighbour : neighbours){
		//std::cout<<"test neighbour"<<std::endl;
		//Si on peut l'atteindre depuis le début
		if(distances[neighbour.id]<INFINITY){
			//std::cout<<"--atteignable"<<std::endl;
			//Si on peut atteindre la fin depuis
			if(end.with==neighbour.with && scene->ValidMove(end.pos,neighbour.pos,IGNORE_BALL_COLLISION)){
				//std::cout<<"---atteignant"<<std::endl;
				//Si c'est mieux
				if(distances[neighbour.id]+Position(neighbour.pos-end.pos).Norm()<distToEnd){
					distToEnd=distances[neighbour.id]+Position(neighbour.pos-end.pos).Norm();
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
	

	
	if(ballMove && !_start.with){
		for(auto pos : startPath.waypoints){
			path.add(pos);
		}
		//path.add(start);
	}
	else{
		path.add(_start);		
	}

	while(!idPathStack.empty()){
		path.add(waypoints[idPathStack.top()]);
		idPathStack.pop();
	}

	if(ballMove && !_end.with){
		for(auto pos : endPath.waypoints){
			path.add(pos);
		}
		//path.add(start);
	}
	else{
		path.add(_end);		
	}

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
		   scene->ValidMove(neighbour.pos,node.pos,flags)){
				adjacency[neighbour.id].push_back(node);
		}
		else
			fail++;
		//std::cout<<"done"<<std::endl;
	}
	failRate.push_back(fail/double(neighbours.size()));
}

Path Roadmap::OkCatch(Position start,Position end,Point* ball){
	if(scene->Collision(end,IGNORE_BALL_COLLISION)){
		//std::cout<<"failed catch "<<std::endl;
		return Path();
	}

	//// A VERIFIER ! C'est là qu'on local plan
	Path p = getPath(FullNode(start,-1,false),FullNode(end,-1,false),ball);
	
	return p;
	
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
			addNode(node,(i==1)?TRANSPORTING_BALL:IGNORE_BALL_COLLISION);
			//std::cout<<"node added\n"<<std::endl;
			if(waypoints.size()%100==0 && waypoints.size()!=prec){
				std::cout<<waypoints.size()<<std::endl;
				prec=waypoints.size();
			}
		}
	}
	return;
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
		if(scene->ValidMove(node.setBall(&pos),scene->robot.Catch(node.pos,pos), TAKING_BALL)){
			std::cout<<"not interesting"<<std::endl;
			continue;
		}

		for(int i=0;i<NB_DROP;i++){
			//On essaie de l'attraper
			Position r = scene->robot.RandomCatch(pos);
			Path p=OkCatch(node.pos,r,&pos);
			if(!p.empty()){
				drop[node.id]=p;
				addNode(FullNode(r,waypoints.size(),true), IGNORE_BALL_COLLISION);
				adjacency[node.id].push_back(FullNode(r,waypoints.size()-1,true));		
				break;
			}			
		}

	}	
}