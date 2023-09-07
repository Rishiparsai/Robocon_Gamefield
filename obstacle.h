#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include "shapes.h"
using namespace std;

void draw_obstacle(char ch) {

    // Obstacle
    int R, G, B;
    if (ch == 'R') {
        R = 176;
        G = 49;
        B = 53;
    } else if (ch == 'B') {
        R = 0;
        G = 107;
        B = 172;
    }

    // Drawing cylinder
    glPushMatrix();
    draw_cylinder(107, 6, R, G, B);  
    glTranslatef(0.0, 0.0, 6.0);
    draw_cylinder(57, 1200-6, R, G, B);
    glPopMatrix();

    glColor3f(R/255.0, G/255.0, B/255.0); 

    // Drawing support structure
    glPushMatrix();

    glBegin(GL_QUADS);

    glNormal3d(0.0, 1.0, 0.0);
    glVertex3i(0, 3, 0);
    glVertex3i(97, 3, 0);
    glVertex3i(57, 3, 195);
    glVertex3i(0, 3, 195);

    glNormal3d(0.0, -1.0, 0.0);
    glVertex3i(0, -3, 0);
    glVertex3i(0, -3, 195);
    glVertex3i(57, -3, 195);
    glVertex3i(97, -3, 0);

    glNormal3d(0.0, 0.0, -1.0);
    glVertex3i(0, -3, 0);
    glVertex3i(97, -3, 0);
    glVertex3i(97, 3, 0);
    glVertex3i(0, 3, 0);

    glNormal3d(0.0, 0.0, 1.0);
    glVertex3i(0, -3, 195);
    glVertex3i(0, 3, 195);
    glVertex3i(57, 3, 195);
    glVertex3i(57, -3, 195);
    
    glNormal3d(1.0, 0.0, 0.0);
    glVertex3i(0, -3, 0);
    glVertex3i(0, 3, 0);
    glVertex3i(0, 3, 195);
    glVertex3i(0, -3, 195);

    glNormal3d(0.97, 0, 0.2);
    glVertex3i(97, -3, 0);
    glVertex3i(57, -3, 195);
    glVertex3i(57, 3, 195);
    glVertex3i(97, 3, 0);

    glEnd();
    glPopMatrix();

    glPushMatrix();
    glRotatef(120.0, 0.0, 0.0, 1.0);

    glBegin(GL_QUADS);

    glNormal3d(0.0, 1.0, 0.0);
    glVertex3i(0, 3, 0);
    glVertex3i(97, 3, 0);
    glVertex3i(57, 3, 195);
    glVertex3i(0, 3, 195);

    glNormal3d(0.0, -1.0, 0.0);
    glVertex3i(0, -3, 0);
    glVertex3i(0, -3, 195);
    glVertex3i(57, -3, 195);
    glVertex3i(97, -3, 0);

    glNormal3d(0.0, 0.0, -1.0);
    glVertex3i(0, -3, 0);
    glVertex3i(97, -3, 0);
    glVertex3i(97, 3, 0);
    glVertex3i(0, 3, 0);

    glNormal3d(0.0, 0.0, 1.0);
    glVertex3i(0, -3, 195);
    glVertex3i(0, 3, 195);
    glVertex3i(57, 3, 195);
    glVertex3i(57, -3, 195);
    
    glNormal3d(1.0, 0.0, 0.0);
    glVertex3i(0, -3, 0);
    glVertex3i(0, 3, 0);
    glVertex3i(0, 3, 195);
    glVertex3i(0, -3, 195);

    glNormal3d(0.97, 0, 0.2);
    glVertex3i(97, -3, 0);
    glVertex3i(57, -3, 195);
    glVertex3i(57, 3, 195);
    glVertex3i(97, 3, 0);

    glEnd();
    glPopMatrix();

    glPushMatrix();
    glRotatef(240.0, 0.0, 0.0, 1.0);

    glBegin(GL_QUADS);

    glNormal3d(0.0, 1.0, 0.0);
    glVertex3i(0, 3, 0);
    glVertex3i(97, 3, 0);
    glVertex3i(57, 3, 195);
    glVertex3i(0, 3, 195);

    glNormal3d(0.0, -1.0, 0.0);
    glVertex3i(0, -3, 0);
    glVertex3i(0, -3, 195);
    glVertex3i(57, -3, 195);
    glVertex3i(97, -3, 0);

    glNormal3d(0.0, 0.0, -1.0);
    glVertex3i(0, -3, 0);
    glVertex3i(97, -3, 0);
    glVertex3i(97, 3, 0);
    glVertex3i(0, 3, 0);

    glNormal3d(0.0, 0.0, 1.0);
    glVertex3i(0, -3, 195);
    glVertex3i(0, 3, 195);
    glVertex3i(57, 3, 195);
    glVertex3i(57, -3, 195);
    
    glNormal3d(1.0, 0.0, 0.0);
    glVertex3i(0, -3, 0);
    glVertex3i(0, 3, 0);
    glVertex3i(0, 3, 195);
    glVertex3i(0, -3, 195);

    glNormal3d(0.97, 0, 0.2);
    glVertex3i(97, -3, 0);
    glVertex3i(57, -3, 195);
    glVertex3i(57, 3, 195);
    glVertex3i(97, 3, 0);

    glEnd();
    glPopMatrix();
}