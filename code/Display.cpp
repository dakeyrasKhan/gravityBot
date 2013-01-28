#include <thread>
#include <iostream>
#include "Display.hpp"

Display* display;

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
	display = this;
	isTrajectoryEnded = true;
	lastWaypoint = 0;
	for(auto& x : keys)
		x = false;

	Position p; p[0] = 0; p[1] = 0; p[2] = 0;
	trajectory.push_back(p);

	position[0] = 20; position[1] = 10; position[2] = 50;
	direction[0] = -2; direction[1] = -1; direction[2] = -5;
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
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	GLfloat lightpos[] = {10, 30, 20, 0};
	glLightfv(GL_LIGHT0, GL_POSITION, lightpos);

	GLfloat color[4] = {0.f, .8f, .8f, 1.f};
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color);
	DrawObject(scene->StaticScene());

	color[0] = 1; color[1] = 0; color[2] = .4;
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color);
	DrawObject(scene->RobotObject(UpdatePosition(lastRender)));

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
		position += direction*timediff;
	if(keys[S])
		position -= direction*timediff;
	if(keys[Q])
		position += left*(timediff/2.0);
	if(keys[D])
		position -= left*(timediff/2.0);
	if(keys[R])
		position += up*(timediff/2.0);
	if(keys[F])
		position -= up*(timediff/2.0);

	if(keys[M_LEFT])
	{
		Pixel dMouse = mousePos - oldMousePos;
		double dAngleX = double(dMouse[0])*0.001;
		double dAngleY = double(dMouse[1])*0.001;

		direction = direction*cos(dAngleX) + left*sin(dAngleX);
		//left = up^direction;
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

	Point lookAt = position + direction;
	gluLookAt(
		position[0], position[1], position[2], 
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


void Display::SetTrajectory(const std::vector<Position>& trajectory)
{
	this->trajectory = trajectory;
	isTrajectoryEnded = false;
	lastWaypointTime = clock::now();
	lastWaypoint = 0;

}


Position Display::UpdatePosition(clock::time_point time)
{
	if(isTrajectoryEnded)
		return trajectory[lastWaypoint];

	clock::duration diff = time - lastWaypointTime;
	double timediff = std::chrono::duration_cast<std::chrono::duration<double>>(diff).count();
	double distanceDone = timediff*ROBOT_SPEED;
	Position direction = trajectory[lastWaypoint] - trajectory[lastWaypoint];
	double distance = direction.Norm();

	while(distanceDone >= distance)
	{
		distanceDone -= distance;
		direction = trajectory[lastWaypoint] - trajectory[lastWaypoint];
		distance = direction.Norm();

		lastWaypoint++;
		if(lastWaypoint == trajectory.size() + 1)
		{
			isTrajectoryEnded = true;
			return *trajectory.end();
		}
		lastWaypointTime = time;
	}

	return trajectory[lastWaypoint] + direction*(distanceDone/distance);
}

