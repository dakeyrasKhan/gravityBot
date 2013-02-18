#include <thread>
#include <iostream>
#include "Display.hpp"

Display* display;
//#define DEBUG_DROP
//#define DEBUG_BB

void Render()
{
	display->Render();
}

void KeyboardFunc(unsigned char key, int x, int y)
{
	display->KeyboardFunc(key, true);
}

void KeyboardUpFunc(unsigned char key, int x, int y)
{
	display->KeyboardFunc(key, false);
}

void MouseFunc(int button, int state, int x, int y)
{
	display->MouseFunc(button, state == GLUT_DOWN, x, y);
}

void MotionFunc(int x, int y)
{
	display->MotionFunc(x, y);
}


Display::Display(int* argc, char* argv[], Scene* scene) : scene(scene), width(800), height(800)
{
	displayPos = false;
	display = this;
	isTrajectoryEnded = true;
	lastWaypoint = 0;
	for(auto& x : keys)
		x = false;

	Position p;
	for(auto& x : p)
		x = 0;

	p[BALL_X] = 0;

	trajectory.push_back(p);

	camPos[0] = 10; camPos[1] = 5; camPos[2] = -10;
	direction[0] = -1; direction[1] = -0.5; direction[2] = 1;
	direction = direction.Normalize();
	up[0] = 0; up[1] = 1; up[2] = 0;

	glutInit(argc, argv);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);
	glutInitWindowSize(width, height);
	glutCreateWindow("Gravibot");

	glutIgnoreKeyRepeat(GLUT_DEVICE_IGNORE_KEY_REPEAT);
	glutDisplayFunc(&::Render);
	glutIdleFunc(&::Render);
	glutKeyboardFunc(&::KeyboardFunc);
	glutKeyboardUpFunc(&::KeyboardUpFunc);
	glutMouseFunc(&::MouseFunc);
	glutMotionFunc(&::MotionFunc);

	glEnable(GL_DEPTH_TEST);
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);

	lastRender = clock::now();
	renderInterval = std::chrono::milliseconds(30);
}


void Display::Render() 
{
	std::this_thread::sleep_until(lastRender + renderInterval);
	clock::duration diff = clock::now() - lastRender;
	double timediff = std::chrono::duration_cast<std::chrono::duration<double>>(diff).count();
	lastRender = clock::now();

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	SetView(timediff);

	DrawAxis();

	// Set the light
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	GLfloat lightpos[] = {10, 30, 20, 0};
	glLightfv(GL_LIGHT0, GL_POSITION, lightpos);

	// Draw the environment
	GLfloat color[4] = {0.f, .8f, .8f, 1.f};
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color);
	DrawObject(scene->StaticScene());

	// Draw the robot
	Position position = UpdatePosition(lastRender, timediff);
	if(scene->Collision(position, TAKING_BALL))
	{
		color[0] = .8; color[1] = 0; color[2] = 0;
	}
	else
	{
		color[0] = 0; color[1] = .8; color[2] = 0;
	}
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color);
	DrawObject(scene->RobotObject(position));

	// Draw the ball
	color[0] = 0; color[1] = 0; color[2] = .8;
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color);

#ifdef DEBUG_BB
	auto bb = scene->robot.GetBoundingBoxes(position, Position(0));
	DrawObject(bb[1].GetObject());
#endif

	glTranslated(position[BALL_X], position[BALL_Y], position[BALL_Z]);
	glutSolidSphere(scene->ballRadius, 100, 100);

#ifdef DEBUG_DROP
	try{
		Point dropPos = scene->Drop(position);
		glTranslated(0, dropPos[Y] - position[BALL_Y], 0);
		glutSolidSphere(scene->ballRadius, 100, 10);
	}
	catch(NoDropPointException) {}
#endif

	if(displayPos)
	{
		displayPos = false;
		std::cout << "Current position :" << std::endl;
		for(auto x : position)
			std::cout << x << std::endl;
	}

	glDisable(GL_LIGHTING);
	glutSwapBuffers();
}


void Display::DrawAxis()
{
	glColor3d(1, 0, 0);
	glBegin(GL_LINES);
	  glVertex3d(0, 0, 0);
	  glVertex3d(1000, 0, 0);
	glEnd();

	glColor3d(0, 1, 0);
	glBegin(GL_LINES);
	  glVertex3d(0, 0, 0);
	  glVertex3d(0, 1000, 0);
	glEnd();

	glColor3d(0, 0, 1);
	glBegin(GL_LINES);
	  glVertex3d(0, 0, 0);
	  glVertex3d(0, 0, 1000);
	glEnd();
}


void Display::DrawObject(const Object& object)
{
	glBegin(GL_TRIANGLES);

	for(auto t : object.triangles)
	{
		Point p[3] = { object.points[t[0]], object.points[t[1]], object.points[t[2]] };
		Point n = ((p[1]-p[0])^(p[2]-p[0])).Normalize();
		glNormal3d(n[0], n[1], n[2]);
		for(int i=0; i<3; i++)
			glVertex3f(p[i][0], p[i][1], p[i][2]);
	}
	glEnd();
}


void Display::SetView(double timediff)
{
	// Get new positions
	timediff *= CAM_SPEED;

	width = glutGet(GLUT_WINDOW_WIDTH);
	height = glutGet(GLUT_WINDOW_HEIGHT);

	Point yUp;
	yUp[0] = 0;
	yUp[1] = 1;
	yUp[2] = 0;
	Point left = (yUp^direction).Normalize();

	if(keys[Z])
		camPos += direction*timediff;
	if(keys[S])
		camPos -= direction*timediff;
	if(keys[Q])
		camPos += left*(timediff/2.0);
	if(keys[D])
		camPos -= left*(timediff/2.0);
	if(keys[R])
		camPos += up*(timediff/2.0);
	if(keys[F])
		camPos -= up*(timediff/2.0);

	if(keys[M_LEFT])
	{
		Pixel dMouse = mousePos - oldMousePos;
		double dAngleX = double(dMouse[0])*0.005;
		double dAngleY = double(dMouse[1])*0.005;

		direction = direction*cos(dAngleX) + left*sin(dAngleX);
		left = (yUp^direction).Normalize();
		direction = direction*cos(dAngleY) + up*sin(dAngleY);
		up = direction^left;

		up = up.Normalize();
		direction = direction.Normalize();
		oldMousePos = mousePos;
	}

	// Set the camera
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	GLdouble aspectratio = GLdouble(width) / GLdouble(height);
	gluPerspective(25, aspectratio, 0.01, 200); 

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	Point lookAt = camPos + direction;
	gluLookAt(
		camPos[0], camPos[1], camPos[2], 
		lookAt[0], lookAt[1], lookAt[2],
		up[0], up[1], up[2]);
}


void Display::KeyboardFunc(unsigned char key, bool down)
{
	switch(key)
	{
	case 'z':
		keys[Z] = down;
		break;
	case 'q':
		keys[Q] = down;
		break;
	case 's':
		keys[S] = down;
		break;
	case 'd':
		keys[D] = down;
		break;
	case 'r':
		keys[R] = down;
		break;
	case 'f':
		keys[F] = down;
		break;
	case '8':
		keys[N_8] = down;
		break;
	case '2':
		keys[N_2] = down;
		break;
	case '4':
		keys[N_4] = down;
		break;
	case '6':
		keys[N_6] = down;
		break;
	case '7':
		keys[N_7] = down;
		break;
	case '9':
		keys[N_9] = down;
		break;
	case '1':
		keys[N_1] = down;
		break;
	case '3':
		keys[N_3] = down;
		break;
	case '0':
		keys[N_0] = down;
		break;
	case '.':
		keys[DOT] = down;
		break;
	case ' ':
		if(down)
			displayPos = true;
		break;
	case '5':
		if(down)
			keys[N_5] = !keys[N_5];
	}
}


void Display::MouseFunc(int button, bool down, int x, int y)
{
	if(button == GLUT_LEFT_BUTTON)
		keys[M_LEFT] = down;

	oldMousePos[0] = x;
	oldMousePos[1] = y;
	mousePos = oldMousePos;
}
 

void Display::MotionFunc(int x, int y)
{
	mousePos[0] = x;
	mousePos[1] = y;
}


void Display::SetTrajectory(const std::vector<Position>& trajectory, const std::vector<bool>& ballOnArm)
{
	this->trajectory = trajectory;
	this->ballOnArm = ballOnArm;

	lastWaypoint = 0;

	if(trajectory.size() > 1)
	{
		isTrajectoryEnded = false;
		lastWaypointTime = clock::now();
	}
	else
		isTrajectoryEnded = true;

}


void Display::SetTrajectory(const Position& position, const bool ballOnArm)
{
	trajectory.clear();
	trajectory.push_back(position);
	lastWaypoint = 0;
	isTrajectoryEnded = true;

	this->ballOnArm.clear();
	this->ballOnArm.push_back(ballOnArm);
}


Position Display::UpdatePosition(clock::time_point time, double timediff)
{
	if(isTrajectoryEnded)
	{
		if(keys[N_8])
			trajectory[lastWaypoint][ROBOT_Z] -= timediff;
		if(keys[N_2])
			trajectory[lastWaypoint][ROBOT_Z] += timediff;
		if(keys[N_4])
			trajectory[lastWaypoint][ROBOT_X] -= timediff;
		if(keys[N_6])
			trajectory[lastWaypoint][ROBOT_X] += timediff;
		if(keys[N_7])
			trajectory[lastWaypoint][ROBOT_ROT] += timediff;
		if(keys[N_9])
			trajectory[lastWaypoint][ROBOT_ROT] -= timediff;
		if(keys[N_1])
			trajectory[lastWaypoint][ROBOT_ARM0] += timediff;
		if(keys[N_3])
			trajectory[lastWaypoint][ROBOT_ARM0] -= timediff;
		if(keys[N_0])
			trajectory[lastWaypoint][ROBOT_ARM1] += timediff;
		if(keys[DOT])
			trajectory[lastWaypoint][ROBOT_ARM1] -= timediff;
		if(keys[N_5])
			trajectory[lastWaypoint] = scene->robot.CorrectBallPos(trajectory[lastWaypoint]);


#ifdef DEBUG_DROP
		 trajectory[lastWaypoint] = scene->robot.CorrectBallPos(trajectory[lastWaypoint]);
#endif
		return trajectory[lastWaypoint];
	}

	else
	{
		clock::duration diff = time - lastWaypointTime;
		double timediff = std::chrono::duration_cast<std::chrono::duration<double>>(diff).count();
		double distanceDone = timediff*ROBOT_SPEED;
		Position direction = trajectory[lastWaypoint+1] - trajectory[lastWaypoint];
		double distance = direction.Norm();

		while(distanceDone >= distance)
		{
			distanceDone -= distance;
			direction = trajectory[lastWaypoint+1] - trajectory[lastWaypoint];
			distance = direction.Norm();

			lastWaypoint++;
			if(lastWaypoint+1 == trajectory.size())
			{
				isTrajectoryEnded = true;
				return trajectory[lastWaypoint];
			}
			lastWaypointTime = time;
		}

		Position pos = trajectory[lastWaypoint] + direction*(distanceDone/distance);
		if(ballOnArm[lastWaypoint] || ballOnArm[lastWaypoint+1]) //si ça merde il faudra reflechir ici
			pos = scene->robot.CorrectBallPos(pos);

		return pos;
	}
}

