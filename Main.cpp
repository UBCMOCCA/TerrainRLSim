
#include "Main.h"

int main(int argc, char** argv)
{
	//HackTest();
	//HackTest1();
	//HackTest2();
	//HackDogMeasurements();
	//HackRaptorMeasurements();
	//HackBipedMeasurements();
	// testBVHReader();
	// srand((unsigned)0);
	
 	gArgc = argc;
	gArgv = argv;
	ParseArgs(gArgc, gArgv);

	InitCaffe();

	glutInit(&gArgc, gArgv);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);
	glutInitWindowSize(gWinWidth, gWinHeight);
	glutCreateWindow("Terrain RL");

	InitOpenGl();
	SetupScenario();

	Reshape(gWinWidth, gWinHeight);
	glutDisplayFunc(Display);
	glutReshapeFunc(Reshape);
	glutKeyboardFunc(Keyboard);
	glutMouseFunc(MouseClick);
	glutMotionFunc(MouseMove);
	glutTimerFunc(gDisplayAnimTime, Animate, 0);

	InitTime();
	glutMainLoop();

	return EXIT_SUCCESS;
}

