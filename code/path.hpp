typedef Position Vector;

class Path{

public:
	Position getNextPosition(Vector speed, double deltaT);
	bool isDone(){ return this.current==this.waypoints.size()-1; }
	Path(vector<Position>& w){
		this.waypoints=w;
		this.current=1;
		this.currentPos=w[0];
	}
private:
	vector<Position> waypoints;
	Position currentPos;
	int current;
};