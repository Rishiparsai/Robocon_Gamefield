#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include "shapes.h"

void draw_conversion_post() {
    glPushMatrix();
    draw_cube(1420, 700, 100, 126, 195, 79);                                            // Green ...
    glPopMatrix();


    // Vertical Poles
    glPushMatrix();
    glTranslatef(165.5, 114.5, 100.0);
    draw_cylinder(114.5, 6.0, 145, 113, 90);                                            // Brown ...
    glTranslatef(1089.0, 0.0, 0.0);
    draw_cylinder(114.5, 6.0, 145, 113, 90);                                            // Brown ...
    glPopMatrix();

    glPushMatrix();
    glTranslatef(165.5, 114.5, 100.0 + 6.0);
    draw_cylinder(44.5, 3000.0-6.0, 145, 113, 90);                                      // Brown ...
    glTranslatef(1089.0, 0.0, 0.0);
    draw_cylinder(44.5, 3000.0-6.0, 145, 113, 90);                                      // Brown ...
    glPopMatrix();


    // First support structure
    glPushMatrix();
    glTranslatef(165.5, 114.5, 100.0 + 6.0);
    glRotatef(180, 0.0, 0.0, 1.0);

    glBegin(GL_QUADS);

    glNormal3d(0.0, 1.0, 0.0);
    glVertex3f(0.0, 3.0, 0.0);
    glVertex3f(0.0, 3.0, 300.0);
    glVertex3f(44.5, 3.0, 300.0);
    glVertex3f(104.5, 3.0, 0.0);

    glNormal3d(0.0, -1.0, 0.0);
    glVertex3f(0.0, -3.0, 0.0);
    glVertex3f(104.5, -3.0, 0.0);
    glVertex3f(44.5, -3.0, 300.0);
    glVertex3f(0.0, -3.0, 300.0);

    glNormal3d(0.0, 0.0, -1.0);
    glVertex3f(0.0, 3.0, 0.0);
    glVertex3f(104.5, 3.0, 0.0);
    glVertex3f(104.5, -3.0, 0.0);
    glVertex3f(0.0, -3.0, 0.0);

    glNormal3d(0.0, 0.0, 1.0);
    glVertex3f(0.0, 3.0, 300.0);
    glVertex3f(0.0, -3.0, 300.0);
    glVertex3f(44.5, -3.0, 300.0);
    glVertex3f(44.5, 3.0, 300.0);

    glNormal3d(-1.0, 0.0, 0.0);
    glVertex3f(0.0, 3.0, 0.0);
    glVertex3f(0.0, -3.0, 0.0);
    glVertex3f(0.0, -3.0, 300.0);
    glVertex3f(0.0, 3.0, 300.0);

    glNormal3d(0.98, 0.19, 0.0);
    glVertex3f(104.5, 3.0, 0.0);
    glVertex3f(44.5, 3.0, 300.0);
    glVertex3f(44.5, -3.0, 300.0);
    glVertex3f(104.5, -3.0, 0.0);

    glEnd();

    glRotatef(120, 0.0, 0.0, 1.0);

    glBegin(GL_QUADS);

    glNormal3d(0.0, 1.0, 0.0);
    glVertex3f(0.0, 3.0, 0.0);
    glVertex3f(0.0, 3.0, 300.0);
    glVertex3f(44.5, 3.0, 300.0);
    glVertex3f(104.5, 3.0, 0.0);

    glNormal3d(0.0, -1.0, 0.0);
    glVertex3f(0.0, -3.0, 0.0);
    glVertex3f(104.5, -3.0, 0.0);
    glVertex3f(44.5, -3.0, 300.0);
    glVertex3f(0.0, -3.0, 300.0);

    glNormal3d(0.0, 0.0, -1.0);
    glVertex3f(0.0, 3.0, 0.0);
    glVertex3f(104.5, 3.0, 0.0);
    glVertex3f(104.5, -3.0, 0.0);
    glVertex3f(0.0, -3.0, 0.0);

    glNormal3d(0.0, 0.0, 1.0);
    glVertex3f(0.0, 3.0, 300.0);
    glVertex3f(0.0, -3.0, 300.0);
    glVertex3f(44.5, -3.0, 300.0);
    glVertex3f(44.5, 3.0, 300.0);

    glNormal3d(-1.0, 0.0, 0.0);
    glVertex3f(0.0, 3.0, 0.0);
    glVertex3f(0.0, -3.0, 0.0);
    glVertex3f(0.0, -3.0, 300.0);
    glVertex3f(0.0, 3.0, 300.0);

    glNormal3d(0.98, 0.19, 0.0);
    glVertex3f(104.5, 3.0, 0.0);
    glVertex3f(44.5, 3.0, 300.0);
    glVertex3f(44.5, -3.0, 300.0);
    glVertex3f(104.5, -3.0, 0.0);

    glEnd();

    glRotatef(120, 0.0, 0.0, 1.0);

    glBegin(GL_QUADS);

    glNormal3d(0.0, 1.0, 0.0);
    glVertex3f(0.0, 3.0, 0.0);
    glVertex3f(0.0, 3.0, 300.0);
    glVertex3f(44.5, 3.0, 300.0);
    glVertex3f(104.5, 3.0, 0.0);

    glNormal3d(0.0, -1.0, 0.0);
    glVertex3f(0.0, -3.0, 0.0);
    glVertex3f(104.5, -3.0, 0.0);
    glVertex3f(44.5, -3.0, 300.0);
    glVertex3f(0.0, -3.0, 300.0);

    glNormal3d(0.0, 0.0, -1.0);
    glVertex3f(0.0, 3.0, 0.0);
    glVertex3f(104.5, 3.0, 0.0);
    glVertex3f(104.5, -3.0, 0.0);
    glVertex3f(0.0, -3.0, 0.0);

    glNormal3d(0.0, 0.0, 1.0);
    glVertex3f(0.0, 3.0, 300.0);
    glVertex3f(0.0, -3.0, 300.0);
    glVertex3f(44.5, -3.0, 300.0);
    glVertex3f(44.5, 3.0, 300.0);

    glNormal3d(-1.0, 0.0, 0.0);
    glVertex3f(0.0, 3.0, 0.0);
    glVertex3f(0.0, -3.0, 0.0);
    glVertex3f(0.0, -3.0, 300.0);
    glVertex3f(0.0, 3.0, 300.0);

    glNormal3d(0.98, 0.19, 0.0);
    glVertex3f(104.5, 3.0, 0.0);
    glVertex3f(44.5, 3.0, 300.0);
    glVertex3f(44.5, -3.0, 300.0);
    glVertex3f(104.5, -3.0, 0.0);

    glEnd();

    glPopMatrix();


    // Second support structure
    glPushMatrix();

    glTranslatef(165.5 + 1089, 114.5, 100.0 + 6.0);

    glBegin(GL_QUADS);

    glNormal3d(0.0, 1.0, 0.0);
    glVertex3f(0.0, 3.0, 0.0);
    glVertex3f(0.0, 3.0, 300.0);
    glVertex3f(44.5, 3.0, 300.0);
    glVertex3f(104.5, 3.0, 0.0);

    glNormal3d(0.0, -1.0, 0.0);
    glVertex3f(0.0, -3.0, 0.0);
    glVertex3f(104.5, -3.0, 0.0);
    glVertex3f(44.5, -3.0, 300.0);
    glVertex3f(0.0, -3.0, 300.0);

    glNormal3d(0.0, 0.0, -1.0);
    glVertex3f(0.0, 3.0, 0.0);
    glVertex3f(104.5, 3.0, 0.0);
    glVertex3f(104.5, -3.0, 0.0);
    glVertex3f(0.0, -3.0, 0.0);

    glNormal3d(0.0, 0.0, 1.0);
    glVertex3f(0.0, 3.0, 300.0);
    glVertex3f(0.0, -3.0, 300.0);
    glVertex3f(44.5, -3.0, 300.0);
    glVertex3f(44.5, 3.0, 300.0);

    glNormal3d(-1.0, 0.0, 0.0);
    glVertex3f(0.0, 3.0, 0.0);
    glVertex3f(0.0, -3.0, 0.0);
    glVertex3f(0.0, -3.0, 300.0);
    glVertex3f(0.0, 3.0, 300.0);

    glNormal3d(0.98, 0.19, 0.0);
    glVertex3f(104.5, 3.0, 0.0);
    glVertex3f(44.5, 3.0, 300.0);
    glVertex3f(44.5, -3.0, 300.0);
    glVertex3f(104.5, -3.0, 0.0);

    glEnd();

    glRotatef(120, 0.0, 0.0, 1.0);

    glBegin(GL_QUADS);

    glNormal3d(0.0, 1.0, 0.0);
    glVertex3f(0.0, 3.0, 0.0);
    glVertex3f(0.0, 3.0, 300.0);
    glVertex3f(44.5, 3.0, 300.0);
    glVertex3f(104.5, 3.0, 0.0);

    glNormal3d(0.0, -1.0, 0.0);
    glVertex3f(0.0, -3.0, 0.0);
    glVertex3f(104.5, -3.0, 0.0);
    glVertex3f(44.5, -3.0, 300.0);
    glVertex3f(0.0, -3.0, 300.0);

    glNormal3d(0.0, 0.0, -1.0);
    glVertex3f(0.0, 3.0, 0.0);
    glVertex3f(104.5, 3.0, 0.0);
    glVertex3f(104.5, -3.0, 0.0);
    glVertex3f(0.0, -3.0, 0.0);

    glNormal3d(0.0, 0.0, 1.0);
    glVertex3f(0.0, 3.0, 300.0);
    glVertex3f(0.0, -3.0, 300.0);
    glVertex3f(44.5, -3.0, 300.0);
    glVertex3f(44.5, 3.0, 300.0);

    glNormal3d(-1.0, 0.0, 0.0);
    glVertex3f(0.0, 3.0, 0.0);
    glVertex3f(0.0, -3.0, 0.0);
    glVertex3f(0.0, -3.0, 300.0);
    glVertex3f(0.0, 3.0, 300.0);

    glNormal3d(0.98, 0.19, 0.0);
    glVertex3f(104.5, 3.0, 0.0);
    glVertex3f(44.5, 3.0, 300.0);
    glVertex3f(44.5, -3.0, 300.0);
    glVertex3f(104.5, -3.0, 0.0);

    glEnd();

    glRotatef(120, 0.0, 0.0, 1.0);

    glBegin(GL_QUADS);

    glNormal3d(0.0, 1.0, 0.0);
    glVertex3f(0.0, 3.0, 0.0);
    glVertex3f(0.0, 3.0, 300.0);
    glVertex3f(44.5, 3.0, 300.0);
    glVertex3f(104.5, 3.0, 0.0);

    glNormal3d(0.0, -1.0, 0.0);
    glVertex3f(0.0, -3.0, 0.0);
    glVertex3f(104.5, -3.0, 0.0);
    glVertex3f(44.5, -3.0, 300.0);
    glVertex3f(0.0, -3.0, 300.0);

    glNormal3d(0.0, 0.0, -1.0);
    glVertex3f(0.0, 3.0, 0.0);
    glVertex3f(104.5, 3.0, 0.0);
    glVertex3f(104.5, -3.0, 0.0);
    glVertex3f(0.0, -3.0, 0.0);

    glNormal3d(0.0, 0.0, 1.0);
    glVertex3f(0.0, 3.0, 300.0);
    glVertex3f(0.0, -3.0, 300.0);
    glVertex3f(44.5, -3.0, 300.0);
    glVertex3f(44.5, 3.0, 300.0);

    glNormal3d(-1.0, 0.0, 0.0);
    glVertex3f(0.0, 3.0, 0.0);
    glVertex3f(0.0, -3.0, 0.0);
    glVertex3f(0.0, -3.0, 300.0);
    glVertex3f(0.0, 3.0, 300.0);

    glNormal3d(0.98, 0.19, 0.0);
    glVertex3f(104.5, 3.0, 0.0);
    glVertex3f(44.5, 3.0, 300.0);
    glVertex3f(44.5, -3.0, 300.0);
    glVertex3f(104.5, -3.0, 0.0);

    glEnd();

    glPopMatrix();


    // horizontal pole
    glPushMatrix();
    glTranslatef(165.5, 114.5, 1470.0);
    glRotatef(90.0, 0.0, 1.0, 0.0);
    draw_cylinder(30.0, 1089, 145, 113, 90);                                            // Brown ...
    glPopMatrix();


    // Inclined pole
    glPushMatrix();
    glTranslatef(165.5, 700.0-90.0, 100.0);
    draw_cylinder(60.0, 6.0, 145, 113, 90);                                             // Brown ...
    glTranslatef(1089.0, 0.0, 0.0);
    draw_cylinder(60.0, 6.0, 145, 113, 90);                                             // Brown ...
    glPopMatrix();

    glPushMatrix();
    glTranslatef(165.5, 700.0-90.0, 80.0);
    float rot_ang = atan(540.0/780.0) * 180.0 / PI;
    glRotatef(rot_ang, 1.0, 0.0, 0.0);
    draw_cylinder(30.0, 850.0, 145, 113, 90);                                           // Brown ...
    glRotatef(-rot_ang, 1.0, 0.0, 0.0);
    glTranslatef(1089.0, 0.0, 0.0);
    glRotatef(rot_ang, 1.0, 0.0, 0.0);
    draw_cylinder(30.0, 850.0, 145, 113, 90);                                           // Brown ...
    
    glPopMatrix();
}