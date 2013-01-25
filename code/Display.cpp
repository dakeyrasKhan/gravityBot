#include "Display.h"

Display* display;
void Render()
{
	display->Render();
}

Display::Display(int* argc, char* argv[])
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
		glVertex3f(-0.5,-0.5,0.0); glVertex3f(0.5,0.0,0.0); glVertex3f(0.0,0.5,0.0);
	glEnd();

	glutSwapBuffers();
}
