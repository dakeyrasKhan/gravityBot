#include "Display.h"

Display* display;
void Render()
{
	display->Render();
}

Display::Display(int* argc, char* argv[], Scene* scene) : scene(scene)
{
	display = this;

	glutInit(argc, argv);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);
	glutInitWindowSize(width, height);
	glutCreateWindow("Gravibot");

	glutDisplayFunc(&::Render);
}


void Display::Render() 
{
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f) ;
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	GLdouble aspectratio = GLdouble(width) / GLdouble(height);
	gluPerspective(25, aspectratio, 10, 200); 

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(20, 10, 50, 0, 0, 0, 0, 1, 0);

	// Enable depth test
	glEnable(GL_DEPTH_TEST);

	// Cull backfacing polygons
	glCullFace(GL_BACK);
	glEnable(GL_CULL_FACE);

	// Draw a coordinate axis
	glColor3d(0, 1, 1);

	glBegin(GL_LINES);
	  glVertex3d(0., 0., 0.);
	  glVertex3d(12., 0., 0.);
	  glVertex3d(0., 0., 0.);
	  glVertex3d(0., 12., 0.);
	  glVertex3d(0., 0., 0.);
	  glVertex3d(0., 0., 12.);
	glEnd();

	glBegin(GL_TRIANGLES);
	for(auto t : scene->triangles)
		for(int i=0; i<3; i++)
			glVertex3f(scene->points[t[i]][0], scene->points[t[i]][1], scene->points[t[i]][2]);
	glEnd();

	glutSwapBuffers();
}
