#include "roadmap.hpp"
#include <iostream>
#include <exception>


Path findPath(std::vector<Adj>& neighbours,int cible){
	for(auto n : neighbours){
		if(n.node.id==cible)
			return n.drop;
	}
}

Path Roadmap::getPath(FullNode _start, FullNode _end,Point *pos=NULL,bool main=false)
{
	bool ballMove = (_start.pos[BALL_X]!=_end.pos[BALL_X]) || (_start.pos[BALL_Y]!=_end.pos[BALL_Y]) || (_start.pos[BALL_Z]!=_end.pos[BALL_Z]);
	std::vector<FullNode> neighbours;
	Path path;

	//if start.ballPos != end.ballPos et start!= with, il faut chopper à la main
	FullNode start=_start;
	Path startPath,endPath;
	if(ballMove && !_start.with)
	{
		Point ballpos=start.pos.getBall();
		for(int i=0;i<NB_TRY;i++)
		{
			Position r = scene->robot.RandomCatch(ballpos);
			Path p=OkCatch(_start.pos,r,&ballpos);
			if(!p.empty()){
				start=FullNode(r,-1,true);
				startPath=p;
				std::cout<<"TOOK ball from start"<<std::endl;
				break;
			}
			else{
				std::cout<<"Cannot take ball from start"<<std::endl;
			}
		}
	}

	tree.AddNeighbors(start.pos.ToCoord(), NB_NEIGHBOURS, start.with, neighbours);
	if(main)
		std::cout<<"Found "<<neighbours.size()<<" potential start neighbours"<<std::endl;
	
	std::priority_queue<NodeComp> heap;
	bool *seen = new bool[waypoints.size()];
	double *distances = new double[waypoints.size()];
	int *father = new int[waypoints.size()];

	for(int i=0;i<waypoints.size();i++)
	{
		seen[i]=false;
		distances[i]= INFINITY;
		father[i]=-1;
	}
	

	for(int n=0;n<neighbours.size();n++)
	{
		FullNode neighbour = neighbours[n];
		//if(!start.with)
			//neighbour.pos=neighbour.setBall(&start.pos.getBall());

		neighbour.pos[BALL_X] = start.pos[BALL_X];
		neighbour.pos[BALL_Y] = start.pos[BALL_Y];
		neighbour.pos[BALL_Z] = start.pos[BALL_Z];

		if(scene->ValidMove(start.pos, neighbour.pos, TAKING_BALL))
		{
			distances[neighbour.id] = Position(neighbour.pos-start.pos).Norm();
			heap.push(NodeComp(neighbour, Position(neighbour.pos-start.pos).Norm()));
		}
	}
	
	
	if(main)
		std::cout<<"starting dijkstra with "<<heap.size()<<" nodes"<<std::endl;

	while(!heap.empty()){
		FullNode current = heap.top().node;
		heap.pop();
		if(seen[current.id])
			continue;
		seen[current.id]=true;
		for(int n=0;n<adjacency[current.id].size();n++){
			Adj& neighbour = adjacency[current.id][n];
			if(pos!=NULL){
				if(!scene->ValidMove(current.setBall(pos),neighbour.node.setBall(pos),TAKING_BALL)){
					continue;
				}
			}

			double dist = Position(current.pos-neighbour.node.pos).Norm();
			if(distances[current.id] + dist < distances[neighbour.node.id]){
				distances[neighbour.node.id] = distances[current.id] + dist;
				father[neighbour.node.id] = current.id;
				heap.push(NodeComp(neighbour.node,dist));
			}
		}
	}
	if(main)
		std::cout<<"dijkstra ended"<<std::endl;

	FullNode end=_end;

	if(ballMove && !_end.with){
		Point ballpos=_end.pos.getBall();
		for(int blah=0;blah<NB_TRY;blah++){
			Position r = scene->robot.RandomDrop(ballpos);
			Path p = OkCatch(r,_end.pos,&ballpos);
			if(!p.empty()){
				std::cout<<"FOUND drop point "<<std::endl;
				endPath=p;
				end=FullNode(r,-1,true);
				break;
			}
			else{
				std::cout<<"Cannot find drop point "<<std::endl;
			}
		}
	}

	neighbours.clear();
	tree.AddNeighbors(end.pos.ToCoord(), NB_NEIGHBOURS, end.with, neighbours);
	if(main)
		std::cout<<"found "<<neighbours.size()<<" possible end"<<std::endl;
	int endPoint=-1;
	double distToEnd=INFINITY;
	for(int n=0;n<neighbours.size();n++){
		FullNode &neighbour = neighbours[n];
		//std::cout<<"test neighbour"<<std::endl;
		//Si on peut l'atteindre depuis le début
		if(distances[neighbour.id]<INFINITY){
			//std::cout<<"--atteignable"<<std::endl;
			//Si on peut atteindre la fin depuis
			//if(!end.with)
				//neighbour.pos=neighbour.pos.setBall(&end.pos.getBall());
			if(end.with==neighbour.with && scene->ValidMove(end.pos,neighbour.pos,TAKING_BALL)){
				std::cout<<"---atteignant neighbour id "<<neighbour.id<<std::endl;
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
	if(main)
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

	int lastId=-1;
	while(!idPathStack.empty())
	{
		int curId = idPathStack.top();
		if(lastId!=-1){
			Path toAdd = findPath(adjacency[lastId],curId);
			if(!toAdd.empty()){
				for(auto p : toAdd.waypoints)
					path.add(p);
			}
			else
				path.add(waypoints[curId]);
		}
		else
			path.add(waypoints[curId]);

		idPathStack.pop();
		lastId=curId;
	}

	if(ballMove && !_end.with){
		for(auto pos : endPath.waypoints){
			pos.with=false;
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

	//std::cout<<"SIZE :"<<path.waypoints.size()<<std::endl;

	return path;
}

void Roadmap::explore(int curNode, int curClasse){
	if(classes[curNode]!=-1)
		return;
	classes[curNode]=curClasse;
	for(auto neighbour : adjacency[curNode])
		explore(neighbour.node.id,curClasse);
}

void Roadmap::addNode(FullNode node, int flags)
{
	if(scene->Collision(node.pos,flags))
		return;

	tree.AddValue(node);

	waypoints.push_back(node);
	adjacency.push_back(std::vector<Adj>());

	std::vector<FullNode> neighbours;
	tree.AddNeighbors(node.pos.ToCoord(), NB_NEIGHBOURS, node.with, neighbours);
	double fail=0;
	for(int n=0;n<neighbours.size();n++){
		FullNode& neighbour = neighbours[n];
		//std::cout<<"Cheking validMove"<<std::endl;
		if(neighbour.with == node.with && 
		   scene->ValidMove(neighbour.pos,node.pos,flags)){
				adjacency[neighbour.id].push_back(Adj(node));
				adjacency[node.id].push_back(Adj(neighbour));
		}
		else
			fail++;
		//std::cout<<"done"<<std::endl;
	}
	failRate.push_back(fail/double(neighbours.size()));
}

Path Roadmap::OkCatch(Position start,Position end,Point* ball)
{
	if(scene->Collision(end,TAKING_BALL))

		return Path();

	//// A VERIFIER ! C'est là qu'on local plan
	FullNode s(start.setBall(ball),-1,false);
	FullNode e(end.setBall(ball),-1,false);
	Path p = getPath(s,e,ball);
	
	return p;
	
}


Roadmap::Roadmap(Scene* scene) :
	scene(scene),
	tree(Coord(0), scene->MaxSize()*2)
{
	int prec=0;

	// On fait 2 roadmap, une avec le robot tenant l'objet, et une sans
	for(int i=0;i<2;i++)
	{
		std::cout<<"pass #"<<i<<std::endl;

		while(waypoints.size() < NB_WAYPOINTS/(2-i))
		{
			Position randompos = Random(scene->NegSize(), scene->PosSize(), i, scene->robot);
			FullNode node(randompos,waypoints.size(), (i==1));

			// Lors de la deuxième passe, on ajoute le flag BALL_ON_ARM
			addNode(node, (i==1) ? TRANSPORTING_BALL:IGNORE_BALL_COLLISION);
		}
	}

	// Pour chaque waypoints du robot tenant l'objet
	int nbIter = waypoints.size();
	for(int j=0; j<nbIter; j++)
	{
		FullNode& node = waypoints[j];
		if(!node.with)
			continue;

		//On lache l'objet
		Point pos;
		try{
			pos = scene->Drop(node.pos);
		}
		catch(NoDropPointException e)
		{
			std::cout<<"no drop"<<std::endl;
			continue;
		}

		//si on peut l'attraper directement sans se prendre d'obstacle, c'est pas intéressant
		if(scene->ValidMove(node.setBall(&pos), scene->robot.Catch(node.pos,pos), TAKING_BALL))
		{
			std::cout<<"not interesting"<<std::endl;
			/*
			FullNode newNode = FullNode(scene->robot.Catch(node.pos,pos),waypoints.size(),true);

			Path p;
			p.add(FullNode(node.setBall(&pos),-1,false));
			p.add(newNode);

			addNode(newNode, IGNORE_BALL_COLLISION);
			adjacency[waypoints[j].id].push_back(Adj(newNode,p));*/
			continue;
		}

		for(int i=0;i<NB_DROP;i++)
		{
			//On essaie de l'attraper
			Position r = scene->robot.RandomCatch(pos);
			Path p=OkCatch(node.pos,r,&pos);
			if(!p.empty())
			{
				std::cout<<"CHOPPE"<<std::endl;
				addNode(FullNode(r,waypoints.size(),true), IGNORE_BALL_COLLISION);
				adjacency[node.id].push_back(Adj(FullNode(r,waypoints.size()-1,true),p));
				break;
			}			
		}

	}	
} 