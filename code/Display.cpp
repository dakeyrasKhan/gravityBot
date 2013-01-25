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
	glutInitWindowSize(800,800);
	glutCreateWindow("Gravibot");

	glutDisplayFunc(&::Render);
}


void Display::Render() 
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glBegin(GL_TRIANGLES);
	for(auto t : scene->triangles)
		for(int i=0; i<3; i++)
			glVertex3f(scene->points[t[i]][0], scene->points[t[i]][1], scene->points[t[i]][2]);
	glEnd();

	glutSwapBuffers();
}
