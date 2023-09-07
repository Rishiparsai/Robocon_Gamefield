#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include "shapes.h"

void draw_try_spots() {
    glPushMatrix();
    draw_cube(1300, 475, 100, 126, 195, 79);                                            // Green ...
    glTranslatef(0.0, 525.0, 0.0);
    draw_cube(1300, 475, 100, 126, 195, 79);                                            // Green ...
    glTranslatef(7200.0, 0.0, 0.0);
    draw_cube(1300, 475, 100, 126, 195, 79);                                            // Green ...
    glTranslatef(0.0, -525.0, 0.0);
    draw_cube(1300, 475, 100, 126, 195, 79);                                            // Green ...

    for (int i=0; i<4; i++) {
        glTranslatef(-1330.0, 0.0, 0.0);
        draw_cube(750, 475, 100, 126, 195, 79);
    }

    glTranslatef(0.0, 525.0, 0.0);

    for (int i=0; i<4; i++) {
        draw_cube(750, 475, 100, 126, 195, 79);
        glTranslatef(1330.0, 0.0, 0.0);
    }

    glPopMatrix();

    glPushMatrix();

    glTranslatef(0.0, 475.0, 0.0);
    draw_cube(8500, 50, 100, 255, 255, 255);                                            // White ...
    
    glPopMatrix();
    glPushMatrix();

    glTranslatef(1300.0, 0.0, 0.0);
    for (int i=0; i<5; i++) {
        draw_cube(580, 10, 10, 253, 229, 153);                                          // Yellow ...
        glTranslatef(1330.0, 0.0, 0.0);
    }
    glTranslatef(0.0, 990.0, 0.0);
    for (int i=0; i<5; i++) {
        glTranslatef(-1330.0, 0.0, 0.0);
        draw_cube(580, 10, 10, 253, 229, 153);                                          // Yellow ...
    }
    glPopMatrix();
}