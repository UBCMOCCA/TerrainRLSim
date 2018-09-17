
#include "Main.h"

int main(int argc, char **argv) {

    gArgc = argc;
    gArgv = argv;
    ParseArgs(gArgc, gArgv);

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
