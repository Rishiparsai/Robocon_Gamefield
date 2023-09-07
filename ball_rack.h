#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include "shapes.h"

const float thick = 10.0;

void draw_ball_rack() {
    draw_cube(1950, 1500, thick, 97, 157, 71);                                          // Dark green base ...


    // Fence is not the part of ball rack


    // Drawing cylindrical part of holes
    glPushMatrix();
    glTranslatef(210.0, 350.0, thick);
    draw_open_cylinder(50, 100, 136, 202, 233);
    for (int i=0; i<4; i++) {
        glTranslatef(270.0, 0.0, 0.0);
        draw_open_cylinder(50, 100, 136, 202, 233);
    }
    glTranslatef(0.0, 800.0, 0.0);
    draw_open_cylinder(50, 100, 136, 202, 233);
    for (int i=0; i<4; i++) {
        glTranslatef(-270.0, 0.0, 0.0);
        draw_open_cylinder(50, 100, 136, 202, 233);
    }
    glPopMatrix();
    

    // Drawing bottom plate with holes
    glPushMatrix();

    int x = 210;

    glTranslatef(0.0, 250.0, thick);
    glEnable(GL_STENCIL_TEST);
    glClearStencil(0);
    glClear(GL_STENCIL_BUFFER_BIT);
    glStencilMask(0xFF);
    glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
    glDepthMask(GL_FALSE);

    glStencilFunc(GL_ALWAYS, 1, 0xFF);
    glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);

    glBegin(GL_QUADS);
    glNormal3d(0.0, 0.0, -1.0);
    glVertex3i(0, 0, 0);
    glVertex3i(1500, 0, 0);
    glVertex3i(1500, 1000, 0);
    glVertex3i(0, 1000, 0);
    glEnd();

    glStencilFunc(GL_ALWAYS, 0, 0xFF);
    glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);

    for (int i=0; i<5; i++) {
        draw_circle(x, 100, 50);
        draw_circle(x, 100+800, 50);
        x += 270;
    }

    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    glDepthMask(GL_TRUE);

    glStencilFunc(GL_NOTEQUAL, 0, 0xFF);
    glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);
    glColor3ub(136, 202, 233);

    glBegin(GL_QUADS);
    glNormal3d(0.0, 0.0, -1.0);
    glVertex3i(0, 0, 0);
    glVertex3i(1500, 0, 0);
    glVertex3i(1500, 1000, 0);
    glVertex3i(0, 1000, 0);
    glEnd();
    glDisable(GL_STENCIL_TEST);
    glPopMatrix();


    // Top plate with holes
    glPushMatrix();
    x = 210;
    glTranslatef(0.0, 250.0, thick + 100.0);
    glEnable(GL_STENCIL_TEST);
    glClearStencil(0);
    glClear(GL_STENCIL_BUFFER_BIT);
    glStencilMask(0xFF);
    glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
    glDepthMask(GL_FALSE);

    glStencilFunc(GL_ALWAYS, 1, 0xFF);
    glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);

    glBegin(GL_QUADS);
    glNormal3d(0.0, 0.0, 1.0);
    glVertex3i(0, 0, 0);
    glVertex3i(1500, 0, 0);
    glVertex3i(1500, 1000, 0);
    glVertex3i(0, 1000, 0);
    glEnd();

    glStencilFunc(GL_ALWAYS, 0, 0xFF);
    glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);

    for (int i=0; i<5; i++) {
        draw_circle(x, 100, 50);
        draw_circle(x, 100+800, 50);
        x += 270;
    }

    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    glDepthMask(GL_TRUE);

    glStencilFunc(GL_NOTEQUAL, 0, 0xFF);
    glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);
    glColor3ub(136, 202, 233);

    glBegin(GL_QUADS);
    glNormal3d(0.0, 0.0, 1.0);
    glVertex3i(0, 0, 0);
    glVertex3i(1500, 0, 0);
    glVertex3i(1500, 1000, 0);
    glVertex3i(0, 1000, 0);
    glEnd();
    glDisable(GL_STENCIL_TEST);
    glPopMatrix();

    glPushMatrix();
    glTranslatef(0.0, 250.0, thick);
    glBegin(GL_QUADS);
    glNormal3d(0.0, -1.0, 0.0);
    glVertex3i(0.0, 0.0, 0.0);
    glVertex3i(1500.0, 0.0, 0.0);
    glVertex3i(1500.0, 0.0, 100.0);
    glVertex3i(0.0, 0.0, 100.0);
    
    glNormal3d(0.0, 1.0, 0.0);
    glVertex3i(0.0, 1000.0, 0.0);
    glVertex3i(0.0, 1000.0, 100.0);
    glVertex3i(1500.0, 1000.0, 100.0);
    glVertex3i(1500.0, 1000.0, 0.0);

    glNormal3d(-1.0, 0.0, 0.0);
    glVertex3i(0.0, 0.0, 0.0);
    glVertex3i(0.0, 0.0, 100.0);
    glVertex3i(0.0, 1000.0, 100.0);
    glVertex3i(0.0, 1000.0, 0.0);

    glNormal3d(1.0, 0.0, 0.0);
    glVertex3i(1500.0, 0.0, 0.0);
    glVertex3i(1500.0, 1000.0, 0.0);
    glVertex3i(1500.0, 1000.0, 100.0);
    glVertex3i(1500.0, 0.0, 100.0);
    glEnd();
    glPopMatrix();


    // Cylindrical surface of holes
    glPushMatrix();
    glTranslatef(1650.0, 0.0, thick);

    glTranslatef(150.0, 102.0, 0.0);
    draw_open_cylinder(50, 100, 136, 202, 233);                                         // Light blue ...
    for (int i=0; i<6; i++) {
        glTranslatef(0.0, 216.0, 0.0);
        draw_open_cylinder(50, 100, 136, 202, 233);                                     // Light blue ...
    }
    glPopMatrix();


    // drawing bottom plate with holes
    glPushMatrix();

    int y = 102;

    glTranslatef(1650.0, 0.0, thick);
    glEnable(GL_STENCIL_TEST);
    glClearStencil(0);
    glClear(GL_STENCIL_BUFFER_BIT);
    glStencilMask(0xFF);
    glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
    glDepthMask(GL_FALSE);

    glStencilFunc(GL_ALWAYS, 1, 0xFF);
    glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);

    glBegin(GL_QUADS);
    glNormal3d(0.0, 0.0, -1.0);
    glVertex3i(0, 0, 0);
    glVertex3i(0, 1500, 0);
    glVertex3i(300, 1500, 0);
    glVertex3i(300, 0, 0);
    glEnd();

    glStencilFunc(GL_ALWAYS, 0, 0xFF);
    glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);

    for (int i=0; i<7; i++) {
        draw_circle(150, y, 50);
        y += 216;
    }

    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    glDepthMask(GL_TRUE);

    glStencilFunc(GL_NOTEQUAL, 0, 0xFF);
    glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);
    glColor3ub(136, 202, 233);

    glBegin(GL_QUADS);
    glNormal3d(0.0, 0.0, -1.0);
    glVertex3i(0, 0, 0);
    glVertex3i(0, 1500, 0);
    glVertex3i(300, 1500, 0);
    glVertex3i(300, 0, 0);
    glEnd();
    glDisable(GL_STENCIL_TEST);
    glPopMatrix();


    // drawing top plate with holes
    glPushMatrix();

    y = 102;

    glTranslatef(1650.0, 0.0, thick + 100);
    glEnable(GL_STENCIL_TEST);
    glClearStencil(0);
    glClear(GL_STENCIL_BUFFER_BIT);
    glStencilMask(0xFF);
    glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
    glDepthMask(GL_FALSE);

    glStencilFunc(GL_ALWAYS, 1, 0xFF);
    glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);

    glBegin(GL_QUADS);
    glNormal3d(0.0, 0.0, 1.0);
    glVertex3i(0, 0, 0);
    glVertex3i(0, 1500, 0);
    glVertex3i(300, 1500, 0);
    glVertex3i(300, 0, 0);
    glEnd();

    glStencilFunc(GL_ALWAYS, 0, 0xFF);
    glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);

    for (int i=0; i<7; i++) {
        draw_circle(150, y, 50);
        y += 216;
    }

    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    glDepthMask(GL_TRUE);

    glStencilFunc(GL_NOTEQUAL, 0, 0xFF);
    glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);
    glColor3ub(136, 202, 233);

    glBegin(GL_QUADS);
    glNormal3d(0.0, 0.0, 1.0);
    glVertex3i(0, 0, 0);
    glVertex3i(0, 1500, 0);
    glVertex3i(300, 1500, 0);
    glVertex3i(300, 0, 0);
    glEnd();
    glDisable(GL_STENCIL_TEST);
    glPopMatrix();

    glPushMatrix();
    glTranslatef(1650.0, 0.0, thick);
    glBegin(GL_QUADS);
    glNormal3d(0.0, -1.0, 0.0);
    glVertex3i(0, 0, 0);
    glVertex3i(300, 0, 0);
    glVertex3i(300, 0, 100);
    glVertex3i(0, 0, 100);

    glNormal3d(0.0, 1.0, 0.0);
    glVertex3i(0, 1500, 0);
    glVertex3i(0, 1500, 100);
    glVertex3i(300, 1500, 100);
    glVertex3i(300, 1500, 0);

    glNormal3d(-1.0, 0.0, 0.0);
    glVertex3i(0, 0, 0);
    glVertex3i(0, 0, 100);
    glVertex3i(0, 1500, 100);
    glVertex3i(0, 1500, 0);

    glNormal3d(1.0, 0.0, 0.0);
    glVertex3i(300, 0, 0);
    glVertex3i(300, 1500, 0);
    glVertex3i(300, 1500, 100);
    glVertex3i(300, 0, 100);
    glEnd();
    glPopMatrix();
}