#include <bits/stdc++.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include "try_spots.h"
#include "obstacle.h"
#include "conversion_post.h"
#include "ball_rack.h"
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
//point, radius, xL, xR, yB, yT, velocity_x, velocity_y
//vector<float> ball()




// Necessary constant declarations ...
const float PIE = 4*atan(1);
const int window_height = 900, window_width = 900;
bool state = true;
float indicator_height = 700;
float indicator_angle = 0.0;
int indicator_sign = 1;
int viewer = 0, vprev = 0;							//initial camera
int num = 1;
bool kick = false;
int score = 0;
bool closed = false;
bool mute = false;
bool night = false;
int width, height, channels;
unsigned char* image_data;
//x, y, z, r, vel, ux, uy, velz
vector<float> ball(8, 0);

// viewAngle,        lx, ly, deltaAngle, x, y, deltaMove, roboMoveAngle, xL, xR, yB, yT, velocity_x, velocity_y
//robots[num][0], [1], [2], [3],      [4],[5],[6],      [7], 
vector<vector<float>> robots(4, vector<float>(8, 0));

vector<pair<int, int>> poles = { {2560+50, 4420+50}, {2560+50,4420+2660+50}, {4100+50,3090+50}, {4100+50,5750+50}, {4100+50,8410+50}, {10740+50,4420+50}, {10740+50,4420+2660+50}, {50+9200,3140}, {9200,3090+2710}, {9250,3090+5330+50}};

// Function declarations ...
void display();
void reshape(int, int);
void update(int);
void init();
void reset_camera();
void draw_ground();
void set_lighting();
void special_key(int, int, int);
void keyboard(unsigned char, int, int);
void draw_robot(int);
void release_key(int, int, int);
void get_pos();
void get_dir();
void draw_indicator();
void get_ball_pos();
void check_ball_collision_change_vel();
void play_sound();
void reset_ball();
void play_sound1();

std::thread* t2;
std::thread* t1;
void window_close_callback(){
    closed=true;
    
    t1->join();
    t2->join();
}


int main(int argc, char **argv) {
    //INITIAL POSITIONS
    t1=new std::thread(play_sound);
    for(int i = 0; i<4; i++){
    	if(i<2){
    		robots[i][0] = -PIE/2;
    		robots[i][1] = -1;
    	}else{
    		robots[i][0] = PIE/2;
    		robots[i][1] = 1;
    	}
    	robots[i][2] = 0;
    }
    robots[2][4] = 4100;
    robots[2][5] = 3090+1330;
    robots[3][4] = 4100;
    robots[3][5] = 5750+1330;
    robots[0][4] = 9200;
    robots[0][5] = 3090+1330;
    robots[1][4] = 9200;
    robots[1][5] = 3090+3990;

    ball[0] = 6650;
    ball[1] = 5000;
    ball[2] = 140;
    ball[3] = 60;
    ball[5] = 1;

    
    image_data = stbi_load("image.png", &width, &height, &channels, STBI_rgb_alpha);

    // INIT GLUT AND CREATE WINDOW
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);           
    glutInitWindowPosition(200, 100);
    glutInitWindowSize(window_width, window_height);                                    // Sets window height and width ...
    glutCreateWindow("Model");

    init();                                                                             // Initializes lighting ...

    // REGISTER CALLBACKS
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);                                                           // Reshapes view window after window resize ...
    glutIdleFunc(display);
    glutTimerFunc(0, update, 0);                                                        // Setting up the frame every time ...
    
    // KEY BINDINGS
    glutKeyboardFunc(keyboard);
    glutSpecialFunc(special_key);
    glutIgnoreKeyRepeat(1);
    glutSpecialUpFunc(release_key);

    glutMainLoop();

    stbi_image_free(image_data);
    return 0;
}


void init() {
    glClearColor(0.0, 0.0, 0.0, 1.0);
    glEnable(GL_DEPTH_TEST);
    set_lighting();
}

void back_to_initial(){
    for(int i = 0; i<4; i++){
    	if(i<2){
    		robots[i][0] = -PIE/2;
    		robots[i][1] = -1;
    	}else{
    		robots[i][0] = PIE/2;
    		robots[i][1] = 1;
    	}
    	robots[i][2] = 0;
    }
    robots[2][4] = 4100;
    robots[2][5] = 3090+1330;
    robots[3][4] = 4100;
    robots[3][5] = 5750+1330;
    robots[0][4] = 9200;
    robots[0][5] = 3090+1330;
    robots[1][4] = 9200;
    robots[1][5] = 3090+3990;
    viewer = 0;
    num = 1;
}

bool isGoal(){
    if(ball[0]>6000 && ball[0]<7400){
        if(ball[1]>10100){
            reset_ball();
            back_to_initial();
            if(!mute)
                t2=new std::thread(play_sound1);
            return true;
        }
    }
    return false;
}

bool check_collision(float nx, float ny, float ndirx, float ndiry)
{  
    for(int i = 0; i<poles.size(); i++){
        float to_pc_x = poles[i].first-nx;
        float to_pc_y = poles[i].second-ny;
        float ppar_dist = to_pc_x*ndirx + to_pc_y*ndiry;
        if(abs(ppar_dist)<= 117){
            // float ppar_vec_x = ppar_dist*robots[i][1];
            // float ppar_vec_y = ppar_dist*robots[i][2];
            // to_cc_x -= ppar_vec_x;
            // to_cc_y -= ppar_vec_y;
            if((to_pc_x*to_pc_x + to_pc_y*to_pc_y)-ppar_dist*ppar_dist<26244){
                return true;
            }
        }
    }
	// for(int i = 0; i<poles.size(); i++){
	// 	bool xcol = (poles[i].first+107 >= (nx-105)) && ((nx+105) >= poles[i].first-7);
    // 	bool ycol = (poles[i].second + 107 >= (ny-60)) && ((ny+60) >= poles[i].second-7);
	// 	if(xcol && ycol){
	// 		return true;
	// 	}
	// }
    for(int i = 0; i<4; i++){
        if(i == num)
            continue;
        bool xcol = (robots[i][4] + 60 >= nx - 60 && nx + 60 >= robots[i][4] - 60);
        bool ycol = (robots[i][5] + 105 >= ny - 105 && ny + 105 >= robots[i][5] - 105);
        if(xcol && ycol)
            return true;
    }

    return false;
}


void get_pos() {
    float new_x = robots[num][4] + robots[num][6]*robots[num][1];
    float new_y = robots[num][5] + robots[num][6]*robots[num][2];
    bool collide = false;
    collide = check_collision(new_x, new_y, robots[num][1], robots[num][2]);
    if(!collide){
    if (new_x > 0 && new_x < 13400) robots[num][4] = new_x;
    if (new_y > -150 && new_y < 10850) {
        if (new_y > 10100) { if (robots[num][4] > 5990 && robots[num][4] < 7410) robots[num][5] = new_y; }
        else if (new_y < 0) { if (robots[num][4] > 5990 && robots[num][4] < 7410) robots[num][5] = new_y; }
        else robots[num][5] = new_y;
    }
    }
}


void get_dir() {
    float new_dir = robots[num][0] + robots[num][3];
    float new_dir_x = sin(new_dir);
    float new_dir_y = -cos(new_dir);
    bool collide = false;
    collide = check_collision(robots[num][4], robots[num][5], new_dir_x, new_dir_y);
    if(!collide){
    robots[num][0] += robots[num][3];
    robots[num][1] = sin(robots[num][0]);
    robots[num][2] = -cos(robots[num][0]);
    }
}

void check_ball_collision_change_vel(){
    if(ball[4]>0){
    //checking with poles now
    for(int i = 0; i<poles.size(); i++){
        float to_pole_x = poles[i].first-ball[0];
        float to_pole_y = poles[i].second-ball[1];
        float to_pole_mod = sqrt(to_pole_x*to_pole_x + to_pole_y*to_pole_y);
        if(to_pole_mod<=170){
            ball[5] = ball[5]-2*(ball[5]*(to_pole_x/to_pole_mod) + ball[6]*(to_pole_y/to_pole_mod))*(to_pole_x/to_pole_mod);
            ball[6] = ball[6]-2*(ball[5]*(to_pole_x/to_pole_mod) + ball[6]*(to_pole_y/to_pole_mod))*(to_pole_y/to_pole_mod);
            ball[0] -= (to_pole_x/to_pole_mod);
            ball[1] -= (to_pole_y/to_pole_mod);
            return;
        }
    }
    }
    //checking with all players 
    //if found with one we change the vel and return which is not very admirable
    if(ball[2]<603){
    for(int i = 0; i<4; i++){
        float to_cc_x = ball[0]-robots[i][4];
        float to_cc_y = ball[1]-robots[i][5];
        float ppar_dist = to_cc_x*robots[i][1] + to_cc_y*robots[i][2];
        if(abs(ppar_dist)<=ball[3] + 60){
            // float ppar_vec_x = ppar_dist*robots[i][1];
            // float ppar_vec_y = ppar_dist*robots[i][2];
            // to_cc_x -= ppar_vec_x;
            // to_cc_y -= ppar_vec_y;
            if((to_cc_x*to_cc_x + to_cc_y*to_cc_y)-ppar_dist*ppar_dist<27225){
                if(i == num && robots[num][3]!=0){
                    float x_poc = ball[0] - ppar_dist*robots[num][1];
                    float y_poc = ball[1] - ppar_dist*robots[num][2];
                    int sign = (robots[num][3]>0)?-1:1;
                    float new_x = ball[0] = ball[0] + ((120 - ppar_dist)*robots[num][1] + 0.014*to_cc_y*sign);
                    float new_y = ball[1] = ball[1] + ((120 - ppar_dist)*robots[num][2] - 0.014*to_cc_x*sign);
                    if (new_x > 0 && new_x < 13300) ball[0] = new_x;
                    if (new_y > 0 && new_y < 10000) {
                        if (new_y > 10100) { if (ball[0] > 5990 && ball[0] < 7410) ball[1] = new_y; }
                        else if (new_y < 0) { if (ball[0] > 5990 && ball[0] < 7410) ball[1] = new_y; }
                        else ball[1] = new_y;
                    }
                    ball[4] = 8;
                    ball[5] = robots[i][1];
                    ball[6] = robots[i][2];
                    if(kick){
                        //vprev = viewer;
                        //viewer = 4;
                        ball[4] = 42;
                        ball[8] = 24;
                        kick = false;
                    }
                }else{
                    ball[4] = 18;
                    ball[5] = robots[i][1];
                    ball[6] = robots[i][2];
                    return;
                }
            }
        }
    }
    }
}

void get_ball_pos(){
    int new_x = ball[0] + ball[4]*ball[5];
    int new_y = ball[1] + ball[4]*ball[6];
    if (new_x > 0 && new_x < 13400) ball[0] = new_x;
    if (new_y > -150 && new_y < 10850) {
        if (new_y > 10100) { if (ball[0] > 5990 && ball[0] < 7410) ball[1] = new_y; }
        else if (new_y < 0) { if (ball[0] > 5990 && ball[0] < 7410) ball[1] = new_y; }
        else ball[1] = new_y;
    }
}

void get_ball_z(){
    float new_z = ball[2] + ball[8];
    if(new_z<140){
        ball[2] = 140;
        if((ball[8])<-4){
            ball[8] = -(0.45)*ball[8];
        }else{
            //viewer = vprev;
            ball[8] = 0.4f;
        }
    }else{
        ball[2] = new_z;
    }
}

void draw_sphere(){
    check_ball_collision_change_vel();
    if(ball[4]>0){
        get_ball_pos();
        ball[4] -= 0.125;
    }
    if((int)ball[8]!=0 || (int)ball[2]!=140){
        get_ball_z();
        ball[8] -= 0.3;
    }
    glPushMatrix();
    glTranslatef(ball[0], ball[1], ball[2]);
    glColor3f(1, 1, 1);
    glutSolidSphere(ball[3], 20, 20);
    glPopMatrix();
    if(kick)
        kick = false;
}

void display() {
    if(isGoal()){
        score += 1;
    }
    if (robots[num][3]) get_dir();
    if (robots[num][6]) get_pos();

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    
    GLuint texture;
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, image_data);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, texture);
    glBegin(GL_QUADS);
    glTexCoord2f(0.0f, 0.0f); glVertex3f(6200.0f, 10215.0f, 2100.0f);
    glTexCoord2f(1.0f, 0.0f); glVertex3f(7250.0f, 10215.0f, 2100.0f);
    glTexCoord2f(1.0f, 1.0f); glVertex3f(7250.0f, 10215.0f, 1500.0f);
    glTexCoord2f(0.0f, 1.0f); glVertex3f(6200.0f, 10215.0f, 1500.0f);
    glEnd();
    glDisable(GL_TEXTURE_2D);
    
    
    reset_camera();
    draw_ground();

    for(int i = 0; i<robots.size(); i++){
    	draw_robot(i);
    }
    draw_indicator();

    


    if (robots[num][6]) { // SHOW HANDS/LEGS MOVEMENT
        if (state) {
            if (robots[num][7] < 30) robots[num][7]+=1.5f;
            else state=false;
        } else {
            if (robots[num][7] > -30) robots[num][7]-=1.5f;
            else state=true;
        }
    } else robots[num][7] = 0.0f;
    cout << "robot number-"<<num<<", x-" << robots[num][4] << ", y-" << robots[num][5] <<", ball vel-"<<ball[8]<< endl;
    draw_sphere();

    glutSwapBuffers();
}


void reshape(int w, int h) {
    if (h == 0) h = 1;
    float ratio =  w * 1.0 / h;
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glViewport(0, 0, w, h);                                                             // Setting up the viewport ...
    gluPerspective(60, ratio, 2.0, 40000.0);                                                // Setting view frustum ...
    glMatrixMode(GL_MODELVIEW);
}


void update(int a) {
    glutPostRedisplay();
    glutTimerFunc(1000/60, update, 0);                                                  // frame rate = 60fps ...
}

void reset_camera() {
    glLoadIdentity();
    if(viewer == 0){
    gluLookAt(  robots[num][4]-robots[num][1]*1000  ,     robots[num][5]-robots[num][2]*1000 , 650.0f,
                robots[num][4]  ,     robots[num][5] , 650.0f,
                0.0f, 0.0f,  1.0f);
        glColor3f(1.0,1.0,1.0);
        glRasterPos3f(robots[num][4] -  robots[num][2]*520 ,     robots[num][5] + robots[num][1]*520 , 1150.0f);
        glDisable(GL_LIGHTING);
        string s = "SCORE: ";
        for(int i = 0; i<s.size(); i++){
            glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_24, s[i]);
        }
        string sc = to_string(score);
        for(int i = 0; i<sc.size(); i++){
            glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_24, sc[i]);
        }
        glEnable(GL_LIGHTING);
    }else if(viewer == 1){
    // gluLookAt(  robots[num][4]-robots[num][1]*1000  ,     robots[num][5]-robots[num][2]*1000 , 650.0f,
    //             robots[num][4] ,     robots[num][5] , 650.0f,
    //             0.0f, 0.0f,  1.0f);
    gluLookAt(  robots[num][4] - 500*robots[num][1]  ,     robots[num][5] - 500*robots[num][2] , 2000.0f,
                robots[num][4] + 500*robots[num][1] ,     robots[num][5] + 500*robots[num][2] , 650.0f,
                0.0f, 0.0f,  1.0f);
    }else if(viewer == 2){
    gluLookAt(  robots[num][4]  ,     robots[num][5] , 2000.0f,
                robots[num][4] + 100*robots[num][1] ,     robots[num][5] + 100*robots[num][2] , 650.0f,
                0.0f, 0.0f,  1.0f);
    }else if(viewer == 3){
    gluLookAt(  6650  ,     -3000 , 2500.0f,
                 6650 ,    3090+3990 , 0.0f ,
                0.0f, 0.0f,  1.0f);
    }else{
        gluLookAt(  ball[0]-500*ball[5]  ,     ball[1]-500*ball[6] , ball[2]+200,
                 ball[0] ,    ball[1] , ball[2]+100 ,
                0.0f, 0.0f,  1.0f);
    }
}


void set_lighting() {
    float light_pos[] = { 6650, 5000, 5000, 1};
    // Setting light properties
    //LIGHT0
    float ambient_light[] = { 0.2, 0.2, 0.2, 1};
    float diffuse_light[] = {0.8, 0.8, 0.8, 1};
    float specular_light[] = {0.9, 0.9, 0.9, 1};
    float specular_material[] = {1, 1, 1, 1};
    float emission_material[] = {0, 0, 0, 1};
    float shininess[] = {60};
    float global_ambient[] = {0.2, 0.2, 0.2, 1};                                        // default ...
    glLightfv(GL_LIGHT0, GL_POSITION, light_pos);
    glLightfv(GL_LIGHT0, GL_AMBIENT, ambient_light);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse_light);
    glLightfv(GL_LIGHT0, GL_SPECULAR, specular_light);

    // LIGHT1
    float ambient_light1[] = { 0.2, 0.2, 0.2, 1};
    float diffuse_light1[] = {0.2, 0.2, 0.6, 1};
    float specular_light1[] = {0.8, 0.8, 0.8, 1};                                   // default ...
    glLightfv(GL_LIGHT1, GL_POSITION, light_pos);
    glLightfv(GL_LIGHT1, GL_AMBIENT, ambient_light1);
    glLightfv(GL_LIGHT1, GL_DIFFUSE, diffuse_light1);
    glLightfv(GL_LIGHT1, GL_SPECULAR, specular_light1);

    glLightModelfv(GL_AMBIENT, global_ambient);

    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_COLOR_MATERIAL);

    // Setting material properties
    glColorMaterial(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specular_material);
    glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, emission_material);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, shininess);
}


void special_key(int key, int x, int y) {
    float move = 10.0f, ang = 0.016f;

    switch (key) {
        case GLUT_KEY_LEFT : robots[num][3] = ang; break;
        case GLUT_KEY_RIGHT : robots[num][3] = -ang; break;
        case GLUT_KEY_UP : robots[num][6] = move; break;
        case GLUT_KEY_DOWN : robots[num][6] = -move; break;
    }
}


void release_key(int key, int x, int y) {
    switch (key) {
        case GLUT_KEY_LEFT : 
        case GLUT_KEY_RIGHT : robots[num][3] = 0.0f;break;
        case GLUT_KEY_UP :
        case GLUT_KEY_DOWN : robots[num][6] = 0;break;
    }
}

void reset_ball(){
    ball[0] = 6650;
    ball[1] = 5000;
    ball[2] = 600;
    ball[3] = 60;
    ball[4] = 0;
    ball[5] = 1;
    ball[6] = 0;
    ball[7] = 0;
}


void keyboard(unsigned char key, int x, int y) {
    switch(key) {
        // PRESS ESCAPE TO EXIT
        case 27:
            exit(0);
        case 'Q':
            //vprev = viewer;
            viewer++;
            viewer %= 5;
            break;
        case 49:
            num = 0;
            break;
        case 50:
            num = 1;
            break;
        case 51:
            num = 2;
            break;
        case 52:
            num = 3;
            break;
        case 'W':
            kick = true;
            break;
        case 'R':
            reset_ball();
            break;
        case 'M':
            if(mute == false){
                closed = true;
                mute = true;
            }else{
                closed = false;
                t1 = new std::thread(play_sound);
                mute = false;
            }
            break;
        case 'N':
            if(night){
                night = false;
                glDisable(GL_LIGHT1);
                glEnable(GL_LIGHT0);
                float shininess[] = {60};
                glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, shininess);
            }else{
                night = true;
                glDisable(GL_LIGHT0);
                glEnable(GL_LIGHT1);
                float shininess[] = {100};
                glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, shininess);
            }
            break;
        default:
            break;
    }
}


void drawCube(float x, float y, float z, string str) {
    if (str == "brown") {
        draw_cube(x, y, z, 145, 113, 90);
    } else if (str == "darkGreen") {
        draw_cube(x, y, z, 97, 157, 71);
    } else if (str == "green") {
        draw_cube(x, y, z, 126, 195, 79);
    } else if (str == "red") {
        draw_cube(x, y, z, 176, 49, 53);
    } else if (str == "blue") {
        draw_cube(x, y, z, 0, 107, 172);
    }
}

// i = robot number
void draw_robot(int i) {

    glPushMatrix();
    glTranslatef(robots[i][4], robots[i][5], 0);
    glRotatef((robots[i][0]*180)/PIE, 0.0f, 0.0f, 1.0f);

    // HEAD
    glPushMatrix();
    glTranslatef(30, 0, 543);
    glRotatef(180, 0.0f, 0.0f, 1.0f);
    draw_cube(60, 60, 60, 255, 255, 100);
    glPopMatrix();

    // UPPER BODY
    glPushMatrix();
    glTranslatef(-75, 0, 360);
    if(i>=2){
        drawCube(150, 60, 180, "blue");
    }else{
        drawCube(150, 60, 180, "red");
    }
    glPopMatrix();

    // LEFT LEG
    glPushMatrix();
    glTranslatef(-75, 0, 330);
    glRotatef(robots[i][7], 1.0f, 0.0f, 0.0f);
    drawCube(60, 60, -270, "blue");
    glPopMatrix();

    // RIGHT LEG
    glPushMatrix();
    glTranslatef(15, 0, 330);
    glRotatef(-robots[i][7], 1.0f, 0.0f, 0.0f);
    drawCube(60, 60, -270, "blue");
    glPopMatrix();

    // LEFT HAND
    glPushMatrix();
    glTranslatef(-105, 0, 525);
    glRotatef(-robots[i][7], 1.0f, 0.0f, 0.0f);
    draw_cube(30, 60, -210, 0, 100, 100);
    glPopMatrix();

    // RIGHT HAND
    glPushMatrix();
    glTranslatef(75, 0, 525);
    glRotatef(robots[i][7], 1.0f, 0.0f, 0.0f);
    draw_cube(30, 60, -210, 0, 100, 100);
    glPopMatrix();
    glPopMatrix();
}


void draw_ground() {

    // drawing fence
    glPushMatrix();
    glTranslatef(0.0, 0.0, thick);
    drawCube(13300+100, 50, 100, "brown");
    glTranslatef(0.0, 10000+50, 0.0);
    drawCube(13300+100, 50, 100, "brown");
    glTranslatef(0.0, -10000, 0.0);
    drawCube(50, 10000, 100, "brown");
    glTranslatef(13300+50, 0.0, 0.0);
    drawCube(50, 10000, 100, "brown");
    glPopMatrix();


    //Lower vertical fences
    glPushMatrix();
    glTranslatef(50.0, 0.0, thick);
    glTranslatef(4075.0, 50.0, 0.0);
    drawCube(50, 2983, 100, "brown");
    glTranslatef(5100.0, 0.0, 0.0);
    drawCube(50, 2983, 100, "brown");
    glPopMatrix();


    // Upper vertical fences
    glPushMatrix();
    glTranslatef(50.0, 0.0, thick);
    glTranslatef(4075.0, 8567.0, 0.0);
    drawCube(50, 1483, 100, "brown");
    glTranslatef(5100.0, 0.0, 0.0);
    drawCube(50, 1483, 100, "brown");
    glPopMatrix();


    //Receiving Zone
    glPushMatrix();
    glTranslatef(50.0, 50.0, 0.0);
    glPushMatrix();
    drawCube(1575, 9000, 10, "darkGreen");
    glTranslatef(11725.0, 0.0, 0.0);
    drawCube(1575, 9000, 10, "darkGreen");
    glPopMatrix();
    glPushMatrix();
    glTranslatef(1000.0, 9000.0, 0.0);
    drawCube(575, 1000, 10, "darkGreen");
    glTranslatef(10725.0, 0.0, 0.0);
    drawCube(575, 1000, 10, "darkGreen");
    glPopMatrix();
    glPopMatrix();


    // Start Zone
    glPushMatrix();
    glTranslatef(50.0, 50.0, 0.0);
    glTranslatef(0.0, 9000.0, 0.0);
    // Red Start Zone
    drawCube(1000, 1000, 10, "red");
    // Blue Start Zone
    glTranslatef(12300.0, 0.0, 0.0);
    drawCube(1000, 1000, 10, "blue");
    glPopMatrix();


    // Kicking Zone - Red kicking zone
    glPushMatrix();
    glTranslatef(50.0, 50.0, 0.0);
    glTranslatef(1575.0, 0.0, 0.0);
    draw_cube(2500, 2500, 10, 176, 49, 53);                                             // Kicking Zone 3 - Dark Red ...
    glTranslatef(0.0, 2500.0, 0.0);
    draw_cube(2500, 2500, 10, 190, 76, 62);                                             // Kicking Zone 2 - Medium Red ...
    glTranslatef(0.0, 2500.0, 0.0);
    draw_cube(2500, 5000, 10, 233, 110, 108);                                           // Kicking Zone 1 - Light Red ...
    glPopMatrix();
    

    // Kicking Zone - Blue kicking zone
    glPushMatrix();
    glTranslatef(50.0, 50.0, 0.0);
    glTranslatef(9225.0, 0.0, 0.0);
    draw_cube(2500, 2500, 10, 0, 107, 172);                                             // Kicking Zone 3 - Dark Blue ...
    glTranslatef(0.0, 2500.0, 0.0);             
    draw_cube(2500, 2500, 10, 76, 173, 223);                                            // Kicking Zone 2 - Medium Blue ...
    glTranslatef(0.0, 2500.0, 0.0);
    draw_cube(2500, 5000, 10, 136, 202, 233);                                           // Kicking Zone 1 - Light Blue ...
    glPopMatrix();

    glPushMatrix();
    glTranslatef(50.0, 50.0, 0.0);
    glTranslatef(4075.0, 0.0, 0.0);
    // draw_cube(1000, 1000, 10, 176, 49, 53);                                             // Red ...
    drawCube(1000, 1000, 10, "red");
    glTranslatef(4150.0, 0.0, 0.0);
    // draw_cube(1000, 1000, 10, 0, 107, 172);                                             // Blue ...
    drawCube(1000, 1000, 10, "blue");
    glPopMatrix();


    // Passing zone
    glPushMatrix();
    glTranslatef(50.0, 50.0, 0.0);
    glTranslatef(4075.0, 1000.0, 0.0);
    drawCube(1000, 9000, 10, "green");
    glTranslatef(4150.0, 0.0, 0.0);
    drawCube(1000, 9000, 10, "green");
    glPopMatrix();

    glPushMatrix();
    glTranslatef(50.0, 50.0, 0.0);
    glTranslatef(5075.0, 0.0, 0.0);
    drawCube(1075, 10000, 10, "green");
    glTranslatef(2075.0, 0.0, 0.0);
    drawCube(1075, 10000, 10, "green");
    glPopMatrix(); 


    // Try spots
    glPushMatrix();
    glTranslatef(50.0, 50.0, thick);
    glTranslatef(7150.0, 1500.0, 0.0);
    glRotatef(90, 0.0, 0.0, 1.0);
    draw_try_spots();
    glPopMatrix();


    // Below try spots
    glPushMatrix();
    glTranslatef(50.0, 50.0, 0.0);
    glTranslatef(6150.0, 1500.0, 0.0);
    draw_cube(1000, 8500, 10, 97, 157, 71);
    glPopMatrix();


    // Ball rack
    glPushMatrix();
    glTranslatef(50.0, 50.0, thick);
    glTranslatef(5900.0, 1500.0, 0.0);
    glRotatef(-90, 0.0, 0.0, 1.0);
    draw_ball_rack();
    glPopMatrix();


    // Conversion Post
    glPushMatrix();
    glTranslatef(50.0, 50.0, thick);
    glTranslatef(5940.0, 10050.0, 0.0);
    draw_conversion_post();
    glPopMatrix();


    // Obstacles - Blue
    glPushMatrix();
    glTranslatef(50.0, 50.0, thick);
    glTranslatef(2560.0, 4420.0, 0.0);
    draw_obstacle('B');
    glTranslatef(0.0, 2660.0, 0.0);
    draw_obstacle('B');
    glPopMatrix();
    glPushMatrix();
    glTranslatef(50.0, 50.0, thick);
    glTranslatef(4100.0, 3090.0, 0.0);
    draw_obstacle('B');
    glTranslatef(0.0, 2660.0, 0.0);
    draw_obstacle('B');
    glTranslatef(0.0, 2660.0, 0.0);
    draw_obstacle('B');
    glPopMatrix();


    // Obstacles - Red
    glPushMatrix();
    glTranslatef(50.0, 50.0, thick);
    glTranslatef(10740.0, 4420.0, 0.0);
    draw_obstacle('R');
    glTranslatef(0.0, 2660.0, 0.0);
    draw_obstacle('R');
    glPopMatrix();
    glPushMatrix();
    glTranslatef(50.0, 50.0, thick);
    glTranslatef(9200.0, 3090.0, 0.0);
    draw_obstacle('R');
    glTranslatef(0.0, 2660.0, 0.0);
    draw_obstacle('R');
    glTranslatef(0.0, 2660.0, 0.0);
    draw_obstacle('R');
    glPopMatrix();


    // Guidelines

    // Extreme left and right vertical guidelines
    glPushMatrix();
    glTranslatef(50.0, 50.0, thick);
    glTranslatef(485.0, 0.0, 0.0);
    draw_cube(30, 10000, 5, 255, 255, 255);
    glTranslatef(12300.0, 0.0, 0.0);
    draw_cube(30, 10000, 5, 255, 255, 255);
    glPopMatrix();

    // Guidelines in start zone
    glPushMatrix();
    glTranslatef(50.0, 50.0, thick);
    glTranslatef(0.0, 9485.0, 0.0);
    draw_cube(1000, 30, 5, 255, 255, 255);
    glTranslatef(12300.0, 0.0, 0.0);
    draw_cube(1000, 30, 5, 255, 255, 255);
    glPopMatrix();

    // Left side horizontal guideline 
    glPushMatrix();
    glTranslatef(50.0, 50.0, thick);
    glTranslatef(485.0, 7737.0, 0.0);
    draw_cube(2847, 30, 5, 255, 255, 255);
    glTranslatef(0.0, -1995.0, 0.0);
    draw_cube(2847, 30, 5, 255, 255, 255);
    glTranslatef(0.0, -1995.0, 0.0);
    draw_cube(2847, 30, 5, 255, 255, 255);
    glPopMatrix();

    // Right side horizontal guidelines
    glPushMatrix();
    glTranslatef(50.0, 50.0, thick);
    glTranslatef(9968.0, 7737.0, 0.0);
    draw_cube(2847, 30, 5, 255, 255, 255);
    glTranslatef(0.0, -1995.0, 0.0);
    draw_cube(2847, 30, 5, 255, 255, 255);
    glTranslatef(0.0, -1995.0, 0.0);
    draw_cube(2847, 30, 5, 255, 255, 255);
    glPopMatrix();

    // Left side vertical guideline
    glPushMatrix();
    glTranslatef(50.0, 50.0, thick);
    glTranslatef(3302.0, 3747.0, 0.0);
    draw_cube(30, 4020, 5, 255, 255, 255);
    glPopMatrix();

    // Right side vertical guideline
    glPushMatrix();
    glTranslatef(50.0, 50.0, thick);
    glTranslatef(9968.0, 3747.0, 0.0);
    draw_cube(30, 4020, 5, 255, 255, 255);
    glPopMatrix();

    // Left side horizontal guidelines - 2
    glPushMatrix();
    glTranslatef(50.0, 50.0, thick);
    glTranslatef(3332.0, 7064.0, 0.0);
    draw_cube(2363, 30, 5, 255, 255, 255);
    glTranslatef(0.0, -2660.0, 0.0);
    draw_cube(2363, 30, 5, 255, 255, 255);
    glPopMatrix();

    // Right side horizontal guidelines - 2
    glPushMatrix();
    glTranslatef(50.0, 50.0, thick);
    glTranslatef(7635.0, 7064.0, 0.0);
    draw_cube(2363, 30, 5, 255, 255, 255);
    glTranslatef(0.0, -2660.0, 0.0);
    draw_cube(2363, 30, 5, 255, 255, 255);
    glPopMatrix();

    // Left side vertical guideline
    glPushMatrix();
    glTranslatef(50.0, 50.0, thick);
    glTranslatef(5665.0, 0.0, 0.0);
    draw_cube(30, 10000, 5, 255, 255, 255);
    glPopMatrix();

    // Right side vertical guideline
    glPushMatrix();
    glTranslatef(50.0, 50.0, thick);
    glTranslatef(7605.0, 0.0, 0.0);
    draw_cube(30, 10000, 5, 255, 255, 255);
    glPopMatrix();

    // Horizontal guidelines in passing zone
    glPushMatrix();
    glTranslatef(50.0, 50.0, thick);
    glTranslatef(4125.0, 485.0, 0.0);
    draw_cube(1540, 30, 5, 255, 255, 255);
    glPopMatrix();
    glPushMatrix();
    glTranslatef(50.0, 50.0, thick);
    glTranslatef(7635.0, 485.0, 0.0);
    draw_cube(1540, 30, 5, 255, 255, 255);
    glPopMatrix();

    // Vertical lines in passing zone
    glPushMatrix();
    glTranslatef(50.0, 50.0, thick);
    glTranslatef(4560.0, 0.0, 0.0);
    draw_cube(30, 1000, 5, 255, 255, 255);
    glPopMatrix();
    glPushMatrix();
    glTranslatef(50.0, 50.0, thick);
    glTranslatef(8710.0, 0.0, 0.0);
    draw_cube(30, 1000, 5, 255, 255, 255);
    glPopMatrix();

    // Horizontal dashes
    glPushMatrix();
    glTranslatef(50.0, 50.0, thick);
    glTranslatef(5500.0, 8395.0, 0.0);
    draw_cube(300, 30, 5, 255, 255, 255);
    glTranslatef(0.0, -1330.0, 0.0);
    draw_cube(300, 30, 5, 255, 255, 255);
    glTranslatef(0.0, -1330.0, 0.0);
    draw_cube(300, 30, 5, 255, 255, 255);
    glTranslatef(0.0, -1330.0, 0.0);
    draw_cube(300, 30, 5, 255, 255, 255);
    glTranslatef(0.0, -1330.0, 0.0);
    draw_cube(300, 30, 5, 255, 255, 255);
    glPopMatrix();

    glPushMatrix();
    glTranslatef(50.0, 50.0, thick);
    glTranslatef(7500.0, 8395.0, 0.0);
    draw_cube(300, 30, 5, 255, 255, 255);
    glTranslatef(0.0, -1330.0, 0.0);
    draw_cube(300, 30, 5, 255, 255, 255);
    glTranslatef(0.0, -1330.0, 0.0);
    draw_cube(300, 30, 5, 255, 255, 255);
    glTranslatef(0.0, -1330.0, 0.0);
    draw_cube(300, 30, 5, 255, 255, 255);
    glTranslatef(0.0, -1330.0, 0.0);
    draw_cube(300, 30, 5, 255, 255, 255);
    glPopMatrix();
}

void draw_indicator(){
    glPushMatrix();
    glTranslatef(robots[num][4], robots[num][5], indicator_height);
    glRotatef(((indicator_angle++)*6)/PIE, 0.0f, 0.0f, 1.0f);
    indicator_height += (3*indicator_sign);
    if(indicator_height>800){
        indicator_sign = -1;
    }else if(indicator_height<700){
        indicator_sign = 1;
    }
    glBegin(GL_TRIANGLE_STRIP);
    glColor3f(1, 1, 1); glVertex3f(0, 0, 0);
    glColor3f(1, 0, 0); glVertex3f(-50, -29, 100);
    glColor3f(0, 1, 0); glVertex3f(50, -29, 100);
    glColor3f(0, 0, 1); glVertex3f(0, 50, 100);
    glColor3f(1, 1, 1); glVertex3f(0, 0, 0);
    glColor3f(1, 0, 0); glVertex3f(-50, -29, 100);
    glEnd();
    glPopMatrix();
}

void play_sound1(){
    system("play -q m2.wav &");
}

void play_sound(){
    while (!closed) {
        // Start playing the audio file
        system("play -q m1.mp3 &");

        // Wait for a few seconds before checking the status of closed

        // Check the status of closed and stop playback if it's true
        for(int i=0;i<6;i++){
            sleep(1);
            if (closed) {
                // int pid = system("pgrep play");
                // pid = atoi(std::to_string(pid).c_str());
                // kill(pid, SIGINT);
                break;
            }   
        }   
    }
}