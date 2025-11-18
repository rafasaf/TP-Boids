#include <GL/glut.h>
#include <cmath>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <algorithm>
#include <string>   // <-- necessário para o modo debug

struct Vec3 {
    float x, y, z;
    Vec3(float X=0, float Y=0, float Z=0) : x(X), y(Y), z(Z) {}

    Vec3 operator+(const Vec3& o) const { return Vec3(x+o.x, y+o.y, z+o.z); }
    Vec3 operator-(const Vec3& o) const { return Vec3(x-o.x, y-o.y, z-o.z); }
    Vec3 operator*(float s) const { return Vec3(x*s, y*s, z*s); }
    Vec3 operator/(float s) const { return Vec3(x/s, y/s, z/s); }

    Vec3& operator+=(const Vec3& o) { x+=o.x; y+=o.y; z+=o.z; return *this; }
    Vec3& operator-=(const Vec3& o) { x-=o.x; y-=o.y; z-=o.z; return *this; }
    Vec3& operator*=(float s) { x*=s; y*=s; z*=s; return *this; }

    float length() const { return std::sqrt(x*x + y*y + z*z); }
};

Vec3 normalize(const Vec3& v) {
    float len = v.length();
    if (len < 1e-6f) return Vec3(0,0,0);
    return v / len;
}

// Mundo
const float WORLD_SIZE = 80.0f;
const float FLOOR_Y = 0.0f;

// Torre
const float TOWER_HEIGHT = 20.0f;
const float TOWER_RADIUS = 5.0f;

// Fog
bool fogEnabled = false;

// Luz das sombras
Vec3 lightDir(-0.5f, -1.0f, -0.3f);
GLfloat shadowMat[16];

void initShadowMatrix() {
    float lx = lightDir.x;
    float ly = lightDir.y;
    float lz = lightDir.z;
    if (std::fabs(ly) < 1e-3f) ly = (ly >= 0 ? 1e-3f : -1e-3f);

    shadowMat[0]  = 1.0f;        shadowMat[4]  = -lx/ly;   shadowMat[8]  = 0.0f;  shadowMat[12] = 0.0f;
    shadowMat[1]  = 0.0f;        shadowMat[5]  = 0.0f;     shadowMat[9]  = 0.0f;  shadowMat[13] = 0.0f;
    shadowMat[2]  = 0.0f;        shadowMat[6]  = -lz/ly;   shadowMat[10] = 1.0f;  shadowMat[14] = 0.0f;
    shadowMat[3]  = 0.0f;        shadowMat[7]  = 0.0f;     shadowMat[11] = 0.0f;  shadowMat[15] = 1.0f;
}

// Boids
struct Boid {
    Vec3 pos;
    Vec3 vel;
    float wingAngle;
    float roll;
    float rollVel;
    float wingDir;
};

std::vector<Boid> boids;

struct Obstacle {
    Vec3 pos;
    float radius;
    bool isCone;
    float height;
};

std::vector<Obstacle> obstacles;
float weightObstacle = 4.0f;

// Boid alvo
Vec3 targetPos(0.0f, 10.0f, 0.0f);
Vec3 targetVel(0.5f, 0.0f, 0.0f);

// Simulação
float dt = 0.016f;
float maxSpeed = 8.0f;
float maxForce = 15.0f;
float neighborRadius = 15.0f;
float desiredSeparation = 4.0f;
float weightSeparation = 1.5f;
float weightAlignment  = 1.0f;
float weightCohesion   = 1.0f;
float weightTarget     = 2.0f;

// Animação das asas
float wingMinAngle = -35.0f;
float wingMaxAngle = 35.0f;
float wingSpeed    = 160.0f;

// Câmera
int   cameraMode   = 1;
float cameraDist   = 35.0f;
float cameraHeight = 15.0f;

int initialBoids = 25;

// ------------------------
// MODO DE PAUSA E DEBUG
// ------------------------
bool paused = false;
bool debugMode = false;
// ------------------------

float frand(float a, float b) {
    return a + (b - a) * (float(rand()) / float(RAND_MAX));
}

Vec3 limit(const Vec3& v, float maxLen) {
    float len = v.length();
    if (len > maxLen && len > 1e-6f) {
        return v * (maxLen / len);
    }
    return v;
}

// Flocking
Vec3 computeSeparation(int i) {
    Vec3 steer(0,0,0);
    int count = 0;
    for (int j = 0; j < (int)boids.size(); ++j) {
        if (i == j) continue;
        Vec3 diff = boids[i].pos - boids[j].pos;
        float d = diff.length();
        if (d > 0 && d < desiredSeparation) {
            steer += diff / (d*d + 1.0f);
            count++;
        }
    }
    if (count > 0) steer = steer / (float)count;

    if (steer.length() > 0) {
        steer = normalize(steer) * maxSpeed - boids[i].vel;
        steer = limit(steer, maxForce);
    }
    return steer;
}

Vec3 computeAlignment(int i) {
    Vec3 sumVel(0,0,0);
    int count = 0;
    for (int j = 0; j < (int)boids.size(); ++j) {
        if (i == j) continue;
        float d = (boids[j].pos - boids[i].pos).length();
        if (d < neighborRadius) {
            sumVel += boids[j].vel;
            count++;
        }
    }
    if (count > 0) {
        sumVel = sumVel / (float)count;
        sumVel = normalize(sumVel) * maxSpeed;
        Vec3 steer = sumVel - boids[i].vel;
        return limit(steer, maxForce);
    }
    return Vec3(0,0,0);
}

Vec3 computeCohesion(int i) {
    Vec3 center(0,0,0);
    int count = 0;
    for (int j = 0; j < (int)boids.size(); ++j) {
        if (i == j) continue;
        float d = (boids[j].pos - boids[i].pos).length();
        if (d < neighborRadius) {
            center += boids[j].pos;
            count++;
        }
    }
    if (count > 0) {
        center = center / (float)count;
        Vec3 desired = center - boids[i].pos;
        desired = normalize(desired) * maxSpeed;
        Vec3 steer = desired - boids[i].vel;
        return limit(steer, maxForce);
    }
    return Vec3(0,0,0);
}

Vec3 computeObstacleAvoidance(int i) {
    Vec3 steer(0,0,0);
    if (boids[i].vel.length() < 1e-4f) return steer;

    Vec3 dir = normalize(boids[i].vel);
    Vec3 futurePos = boids[i].pos + dir * 6.0f;

    for (const auto& o : obstacles) {
        Vec3 center = o.pos;
        float effectiveRadius = o.radius;

        if (o.isCone) {
            center.y += o.height * 0.5f;
            effectiveRadius = std::max(o.radius, o.height * 0.5f);
        }

        Vec3 diff = futurePos - center;
        float d = diff.length();
        float safeRadius = effectiveRadius + 2.0f;

        if (d < safeRadius) {
            Vec3 away = normalize(diff) * maxSpeed;
            Vec3 localSteer = away - boids[i].vel;
            localSteer = limit(localSteer, maxForce);
            float factor = (safeRadius - d) / safeRadius;
            steer += localSteer * factor;
        }
    }
    return steer;
}

Vec3 computeTargetAttraction(int i) {
    Vec3 desired = targetPos - boids[i].pos;
    if (desired.length() < 1e-4f) return Vec3(0,0,0);
    desired = normalize(desired) * maxSpeed;
    Vec3 steer = desired - boids[i].vel;
    return limit(steer, maxForce);
}

Vec3 computeBounds(int i) {
    Vec3 steer(0,0,0);
    float bound = WORLD_SIZE * 0.9f;
    Vec3 p = boids[i].pos;
    float k = 20.0f;

    if (p.x >  bound) steer.x -= k;
    if (p.x < -bound) steer.x += k;
    if (p.z >  bound) steer.z -= k;
    if (p.z < -bound) steer.z += k;

    if (p.y < 2.0f) steer.y += k;
    if (p.y > WORLD_SIZE*0.5f) steer.y -= k;

    return steer;
}

Vec3 computeFlockCenter() {
    Vec3 center(0,0,0);
    for (auto& b : boids) center += b.pos;
    if (!boids.empty()) center = center / (float)boids.size();
    return center;
}

Vec3 computeFlockVelocity() {
    Vec3 v(0,0,0);
    for (auto& b : boids) v += b.vel;
    if (!boids.empty()) v = v / (float)boids.size();
    return v;
}

void drawFloor() {
    float half = WORLD_SIZE;
    glDisable(GL_LIGHTING);
    glColor3f(0.2f, 0.5f, 0.2f);
    glBegin(GL_QUADS);
        glVertex3f(-half, FLOOR_Y, -half);
        glVertex3f( half, FLOOR_Y, -half);
        glVertex3f( half, FLOOR_Y,  half);
        glVertex3f(-half, FLOOR_Y,  half);
    glEnd();
    glEnable(GL_LIGHTING);
}

void drawTower() {
    glPushMatrix();
        glTranslatef(0.0f, FLOOR_Y, 0.0f);
        glRotatef(-90.0f, 1,0,0);
        glColor3f(0.7f, 0.4f, 0.2f);
        glutSolidCone(TOWER_RADIUS, TOWER_HEIGHT, 32, 32);
    glPopMatrix();
}

void drawSingleBoid(const Boid& b, bool shadow=false) {
    glPushMatrix();
        glTranslatef(b.pos.x, b.pos.y, b.pos.z);

        Vec3 v = b.vel;
        if (v.length() < 1e-4f) v = Vec3(0,0,1);
        float yaw = atan2(v.x, v.z) * 180.0f / 3.14159265f;
        glRotatef(yaw, 0,1,0);
        glRotatef(b.roll, 0,0,1);

        float thickness = 0.25f;
        float a = b.wingAngle * 3.14159265f / 180.0f;
        float wingOffset = 0.8f * sin(a);

        GLfloat noseTop[3]={0, thickness,2}, noseBottom[3]={0,-thickness,2};
        GLfloat tailTop[3]={0, thickness,-1.8}, tailBottom[3]={0,-thickness,-1.8};
        GLfloat leftWingTop[3]={-2,wingOffset+thickness,0}, leftWingBottom[3]={-2,wingOffset-thickness,0};
        GLfloat rightWingTop[3]={2,wingOffset+thickness,0}, rightWingBottom[3]={2,wingOffset-thickness,0};

        auto setRed=[&](){ if(shadow) glColor3f(0,0,0); else glColor3f(1,0.2f,0.2f); };
        auto setYellow=[&](){ if(shadow) glColor3f(0,0,0); else glColor3f(1,1,0); };

        glBegin(GL_TRIANGLES);
            glNormal3f(0,1,0);
            setRed(); glVertex3fv(noseTop); glVertex3fv(rightWingTop); glVertex3fv(leftWingTop);
            setYellow(); glVertex3fv(tailTop); glVertex3fv(leftWingTop); glVertex3fv(rightWingTop);
        glEnd();

        glBegin(GL_TRIANGLES);
            glNormal3f(0,-1,0);
            setRed(); glVertex3fv(noseBottom); glVertex3fv(leftWingBottom); glVertex3fv(rightWingBottom);
            setYellow(); glVertex3fv(tailBottom); glVertex3fv(rightWingBottom); glVertex3fv(leftWingBottom);
        glEnd();

        glBegin(GL_QUADS);
            glNormal3f(1,0,0);
            setYellow();
            glVertex3fv(noseTop); glVertex3fv(noseBottom); glVertex3fv(rightWingBottom); glVertex3fv(rightWingTop);

            glNormal3f(-1,0,0);
            glVertex3fv(noseTop); glVertex3fv(leftWingTop); glVertex3fv(leftWingBottom); glVertex3fv(noseBottom);

            glNormal3f(1,0,0);
            glVertex3fv(rightWingTop); glVertex3fv(rightWingBottom); glVertex3fv(tailBottom); glVertex3fv(tailTop);

            glNormal3f(-1,0,0);
            glVertex3fv(leftWingTop); glVertex3fv(tailTop); glVertex3fv(tailBottom); glVertex3fv(leftWingBottom);

            glNormal3f(0,0,-1);
            glVertex3fv(noseTop); glVertex3fv(tailTop); glVertex3fv(tailBottom); glVertex3fv(noseBottom);
        glEnd();

    glPopMatrix();
}

void drawBoids() {
    for (auto& b : boids) drawSingleBoid(b, false);
}

void drawBoidShadows() {
    glDisable(GL_LIGHTING);
    glColor3f(0,0,0);

    glPushMatrix();
        glMultMatrixf(shadowMat);
        glTranslatef(0,0.01f,0);

        for (auto& b : boids) drawSingleBoid(b, true);
    glPopMatrix();

    glEnable(GL_LIGHTING);
}

void drawObstacles() {
    for (const auto& o : obstacles) {
        glPushMatrix();
        if (!o.isCone) {
            glTranslatef(o.pos.x, o.pos.y, o.pos.z);
            glColor3f(0.2,0.2,0.9);
            glutSolidSphere(o.radius, 24,24);
        } else {
            glTranslatef(o.pos.x, o.pos.y, o.pos.z);
            glRotatef(-90,1,0,0);
            glColor3f(0.7,0.2,0.7);
            glutSolidCone(o.radius, o.height, 24,24);
        }
        glPopMatrix();
    }
}

void drawTarget() {
    glPushMatrix();
        glTranslatef(targetPos.x, targetPos.y, targetPos.z);
        glColor3f(0.1f,0.4f,0.9f);
        glutSolidSphere(1.0,16,16);
    glPopMatrix();
}

void setupCamera() {
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    Vec3 center = computeFlockCenter();
    Vec3 flockVel = computeFlockVelocity();
    if (flockVel.length() < 1e-4f) flockVel = Vec3(0,0,1);
    Vec3 eye;

    if (cameraMode == 1) {
        eye = Vec3(0,TOWER_HEIGHT+2,0);
    }
    else if (cameraMode == 2) {
        Vec3 dir = normalize(flockVel);
        eye = center - dir * cameraDist + Vec3(0,cameraHeight,0);
    }
    else {
        Vec3 velXZ(flockVel.x, 0, flockVel.z);
        if (velXZ.length() < 1e-4f) velXZ = Vec3(0,0,1);
        velXZ = normalize(velXZ);
        Vec3 up(0,1,0);
        Vec3 lateral(
            up.y * velXZ.z - up.z * velXZ.y,
            up.z * velXZ.x - up.x * velXZ.z,
            up.x * velXZ.y - up.y * velXZ.x
        );
        lateral = normalize(lateral);
        eye = center + lateral * cameraDist + Vec3(0,cameraHeight,0);
    }

    gluLookAt(eye.x,eye.y,eye.z, center.x,center.y,center.z, 0,1,0);
}

void setupLighting() {
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_NORMALIZE);

    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);

    GLfloat ambient[]={0.2,0.2,0.2,1}, diffuse[]={0.8,0.8,0.8,1};
    GLfloat specular[]={0.5,0.5,0.5,1}, position[]={30,50,30,1};

    glLightfv(GL_LIGHT0,GL_AMBIENT,ambient);
    glLightfv(GL_LIGHT0,GL_DIFFUSE,diffuse);
    glLightfv(GL_LIGHT0,GL_SPECULAR,specular);
    glLightfv(GL_LIGHT0,GL_POSITION,position);

    GLfloat mat_a[]={1,1,1,1}, mat_d[]={1,1,1,1}, mat_s[]={0.4,0.4,0.4,1}, shine[]={16};
    glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT,mat_a);
    glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,mat_d);
    glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,mat_s);
    glMaterialfv(GL_FRONT_AND_BACK,GL_SHININESS,shine);
}

void setupFog() {
    GLfloat fogColor[4]={0.5,0.7,1,1};

    glFogi(GL_FOG_MODE,GL_EXP2);
    glFogfv(GL_FOG_COLOR,fogColor);
    glFogf(GL_FOG_DENSITY,0.02f);
    glHint(GL_FOG_HINT,GL_NICEST);

    glFogf(GL_FOG_START,20);
    glFogf(GL_FOG_END,120);

    glDisable(GL_FOG);
}

void updateBoids() {
    if (boids.empty()) return;

    std::vector<Vec3> accelerations(boids.size(), Vec3(0,0,0));

    for (int i=0;i<(int)boids.size();++i) {
        Vec3 sep  = computeSeparation(i)*weightSeparation;
        Vec3 ali  = computeAlignment(i)*weightAlignment;
        Vec3 coh  = computeCohesion(i)*weightCohesion;
        Vec3 tgt  = computeTargetAttraction(i)*weightTarget;
        Vec3 bnd  = computeBounds(i);
        Vec3 obs  = computeObstacleAvoidance(i)*weightObstacle;

        accelerations[i] = sep+ali+coh+tgt+bnd+obs;
    }

    for (int i=0;i<(int)boids.size();++i) {
        // banking addition
        float turnStrength = accelerations[i].x * boids[i].vel.z - accelerations[i].z * boids[i].vel.x;
        float targetRoll = -turnStrength * 10.0f;
        boids[i].rollVel += (targetRoll - boids[i].roll)*0.05f;
        boids[i].rollVel *= 0.95f;
        boids[i].roll += boids[i].rollVel;

        boids[i].vel += accelerations[i]*dt;
        boids[i].vel = limit(boids[i].vel,maxSpeed);
        boids[i].pos += boids[i].vel*dt;

        boids[i].wingAngle += boids[i].wingDir * wingSpeed * dt;
        if (boids[i].wingAngle > wingMaxAngle) {
            boids[i].wingAngle = wingMaxAngle;
            boids[i].wingDir = -1;
        }
        else if (boids[i].wingAngle < wingMinAngle) {
            boids[i].wingAngle = wingMinAngle;
            boids[i].wingDir = 1;
        }
    }
}

void updateTarget() {
    targetVel = limit(targetVel, maxSpeed*1.2f);
    targetPos += targetVel * dt;

    if (targetPos.x > WORLD_SIZE) targetVel.x *= -0.5f, targetPos.x = WORLD_SIZE;
    if (targetPos.x < -WORLD_SIZE) targetVel.x *= -0.5f, targetPos.x = -WORLD_SIZE;
    if (targetPos.z > WORLD_SIZE) targetVel.z *= -0.5f, targetPos.z = WORLD_SIZE;
    if (targetPos.z < -WORLD_SIZE) targetVel.z *= -0.5f, targetPos.z = -WORLD_SIZE;

    if (targetPos.y < 5) targetVel.y *= -0.5f, targetPos.y = 5;
    if (targetPos.y > WORLD_SIZE*0.5f) {
        targetVel.y *= -0.5f;
        targetPos.y = WORLD_SIZE*0.5f;
    }

    targetVel *= 0.99f;
}

void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    setupCamera();
    setupLighting();

    if (fogEnabled) glEnable(GL_FOG);
    else glDisable(GL_FOG);

    drawFloor();
    drawTower();
    drawObstacles();
    drawTarget();

    drawBoidShadows();
    drawBoids();

    // -------------------------------------------------------
    // MODO DEBUG — desenha texto na tela
    // -------------------------------------------------------
    if (debugMode) {
        glDisable(GL_LIGHTING);
        glColor3f(1,1,1);

        glMatrixMode(GL_PROJECTION);
        glPushMatrix();
        glLoadIdentity();
        gluOrtho2D(0, 1024, 0, 768);

        glMatrixMode(GL_MODELVIEW);
        glPushMatrix();
        glLoadIdentity();

        glRasterPos2i(10, 740);

        std::string dbg =
            "DEBUG ON | Boids: " + std::to_string(boids.size()) +
            " | Target(" + std::to_string((int)targetPos.x) + ", "
                         + std::to_string((int)targetPos.y) + ", "
                         + std::to_string((int)targetPos.z) + ")";

        for (char c : dbg)
            glutBitmapCharacter(GLUT_BITMAP_9_BY_15, c);

        glPopMatrix();
        glMatrixMode(GL_PROJECTION);
        glPopMatrix();
        glMatrixMode(GL_MODELVIEW);

        glEnable(GL_LIGHTING);
    }
    // -------------------------------------------------------

    glutSwapBuffers();
}

void reshape(int w, int h) {
    if (h==0) h=1;
    float aspect = float(w)/h;
    glViewport(0,0,w,h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0, aspect, 0.1,500.0);
    glMatrixMode(GL_MODELVIEW);
}

void timer(int value) {

    // --------------------------------------------
    // PAUSA — não atualiza nada durante pausa
    // --------------------------------------------
    if (!paused) {
        updateTarget();
        updateBoids();
    }
    // --------------------------------------------

    glutPostRedisplay();
    glutTimerFunc(16, timer, 0);
}

void keyboard(unsigned char key, int x, int y) {
    switch (key) {

    case 27:
    case 'q':
    case 'Q':
        exit(0);
        break;

    case 'f':
    case 'F':
        fogEnabled = !fogEnabled;
        break;

    case '1': cameraMode = 1; break;
    case '2': cameraMode = 2; break;
    case '3': cameraMode = 3; break;

    case 'w':
    case 'W':
        targetVel.y += 2.0f;
        break;

    case 's':
    case 'S':
        targetVel.y -= 2.0f;
        break;

    case '+':
    case '=': {
        Vec3 center = computeFlockCenter();
        Boid b;
        b.pos = center + Vec3(frand(-5,5), frand(2,15), frand(-5,5));
        b.vel = Vec3(frand(-2,2), frand(-1,1), frand(-2,2));
        b.wingAngle = frand(wingMinAngle, wingMaxAngle);
        b.wingDir   = (frand(0,1) < 0.5f)? 1 : -1;
        b.roll=0; b.rollVel=0;
        boids.push_back(b);
        break;
    }

    case '-':
    case '_':
        if (!boids.empty()) {
            int idx = rand() % boids.size();
            boids.erase(boids.begin()+idx);
        }
        break;

    // --------------------------------------------
    // TECLA DE PAUSA
    // --------------------------------------------
    case 'p':
    case 'P':
        paused = !paused;
        break;

    // --------------------------------------------
    // TECLA DE MODO DEBUG
    // --------------------------------------------
    case 'd':
    case 'D':
        debugMode = !debugMode;
        break;

    case 'r':
    case 'R':
        boids.clear();
        targetPos = Vec3(0,10,0);
        targetVel = Vec3(0.5f,0,0);
        for (int i=0;i<initialBoids;i++) {
            Boid b;
            b.pos = Vec3(frand(-10,10), frand(5,20), frand(-10,10));
            b.vel = Vec3(frand(-3,3), frand(-1,1), frand(-3,3));
            b.wingAngle = frand(wingMinAngle, wingMaxAngle);
            b.wingDir = (frand(0,1)<0.5)?1:-1;
        b.roll=0; b.rollVel=0;
            boids.push_back(b);
        }
        break;
    }
}

void specialKeyboard(int key, int x, int y) {
    float acc = 2.0f;
    switch (key) {
        case GLUT_KEY_UP:    targetVel.z -= acc; break;
        case GLUT_KEY_DOWN:  targetVel.z += acc; break;
        case GLUT_KEY_LEFT:  targetVel.x -= acc; break;
        case GLUT_KEY_RIGHT: targetVel.x += acc; break;
    }
}

void initBoids() {
    boids.clear();
    for (int i=0;i<initialBoids;i++) {
        Boid b;
        b.pos = Vec3(frand(-10,10), frand(5,20), frand(-10,10));
        b.vel = Vec3(frand(-3,3), frand(-1,1), frand(-3,3));
        b.wingAngle = frand(wingMinAngle, wingMaxAngle);
        b.wingDir = (frand(0,1)<0.5)?1:-1;
        boids.push_back(b);
    }
}

void initObstacles() {
    obstacles.clear();

    Obstacle s1;
    s1.pos=Vec3(15,8,5); s1.radius=4; s1.isCone=false; obstacles.push_back(s1);

    Obstacle s2;
    s2.pos=Vec3(-10,6,-18); s2.radius=3; s2.isCone=false; obstacles.push_back(s2);

    Obstacle c1;
    c1.pos=Vec3(-20,FLOOR_Y,10); c1.radius=3; c1.height=10; c1.isCone=true;
    obstacles.push_back(c1);

    Obstacle towerObs;
    towerObs.pos=Vec3(0,FLOOR_Y,0);
    towerObs.radius=TOWER_RADIUS+1;
    towerObs.height=TOWER_HEIGHT;
    towerObs.isCone=true;
    obstacles.push_back(towerObs);
}

void initGL() {
    glEnable(GL_DEPTH_TEST);
    glShadeModel(GL_SMOOTH);
    glClearColor(0.5f,0.7f,1.0f,1.0f);

    setupLighting();
    setupFog();
}

int main(int argc, char** argv) {
    srand((unsigned)time(nullptr));

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE|GLUT_RGB|GLUT_DEPTH);
    glutInitWindowSize(1024,768);
    glutCreateWindow("Boids 3D - Computacao Grafica");

    initGL();
    initBoids();
    initObstacles();
    initShadowMatrix();

    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);
    glutSpecialFunc(specialKeyboard);
    glutTimerFunc(16, timer, 0);

    glutMainLoop();
    return 0;
}
