#include <GL/glut.h>
#include <cmath>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <algorithm>

// -------------------------------------------------------
// Estruturas e parâmetros globais
// -------------------------------------------------------

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

// Torre (cone) no centro
const float TOWER_HEIGHT = 20.0f;
const float TOWER_RADIUS = 5.0f;

// Boids
struct Boid {
    Vec3 pos;
    Vec3 vel;

    // estado da asa
    float wingAngle;   // graus
    float wingDir;     // +1 ou -1
};

std::vector<Boid> boids;

struct Obstacle {
    Vec3 pos;      // centro (esfera) ou centro da base (cone)
    float radius;  // raio (esfera ou base do cone)
    bool isCone;   // false = esfera, true = cone
    float height;  // usado só se for cone
};

std::vector<Obstacle> obstacles;

// peso da força de desvio
float weightObstacle = 4.0f;

// Boid-objetivo (alvo do bando)
Vec3 targetPos(0.0f, 10.0f, 0.0f);
Vec3 targetVel(0.5f, 0.0f, 0.0f);

// Parâmetros da simulação
float dt = 0.016f;              // ~60 FPS
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
float wingSpeed    = 160.0f;   // graus/s

// Câmera
int cameraMode = 1; // 1: torre, 2: atrás do bando, 3: lateral
float cameraDist = 35.0f;
float cameraHeight = 15.0f;

// Controles
int initialBoids = 25;

// -------------------------------------------------------
// Utilitários
// -------------------------------------------------------

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

// -------------------------------------------------------
// Regras de flocking para um boid
// -------------------------------------------------------

Vec3 computeSeparation(int i) {
    Vec3 steer(0,0,0);
    int count = 0;
    for (int j = 0; j < (int)boids.size(); ++j) {
        if (i == j) continue;
        Vec3 diff = boids[i].pos - boids[j].pos;
        float d = diff.length();
        if (d > 0 && d < desiredSeparation) {
            steer += diff / (d*d + 1.0f); // mais forte se mais perto
            count++;
        }
    }
    if (count > 0) {
        steer = steer / (float)count;
    }
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
        steer = limit(steer, maxForce);
        return steer;
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
        steer = limit(steer, maxForce);
        return steer;
    }
    return Vec3(0,0,0);
}

Vec3 computeObstacleAvoidance(int i) {
    Vec3 steer(0,0,0);
    if (boids[i].vel.length() < 1e-4f) return steer;

    Vec3 dir = normalize(boids[i].vel);
    // posição futura onde o boid "olha" à frente
    Vec3 futurePos = boids[i].pos + dir * 6.0f;

    for (const auto& o : obstacles) {
        // centro aproximado para evitar (esfera ou cone)
        Vec3 center = o.pos;
        float effectiveRadius = o.radius;

        if (o.isCone) {
            // para cones, considera o centro no meio da altura
            center.y += o.height * 0.5f;
            effectiveRadius = std::max(o.radius, o.height * 0.5f);
        }

        Vec3 diff = futurePos - center;
        float d = diff.length();
        float safeRadius = effectiveRadius + 2.0f; // margem de segurança

        if (d < safeRadius && d > 1e-4f) {
            // vetor de fuga
            Vec3 away = normalize(diff) * maxSpeed;
            Vec3 localSteer = away - boids[i].vel;
            localSteer = limit(localSteer, maxForce);

            // quanto mais perto, mais forte
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
    steer = limit(steer, maxForce);
    return steer;
}

// “Força” para manter dentro do mundo
Vec3 computeBounds(int i) {
    Vec3 steer(0,0,0);
    float bound = WORLD_SIZE * 0.9f;
    Vec3 p = boids[i].pos;
    float k = 20.0f;

    if (p.x >  bound) steer.x -= k;
    if (p.x < -bound) steer.x += k;
    if (p.z >  bound) steer.z -= k;
    if (p.z < -bound) steer.z += k;

    // Evitar atravessar o chão
    if (p.y < 2.0f) steer.y += k;
    if (p.y > WORLD_SIZE*0.5f) steer.y -= k;

    return steer;
}

// -------------------------------------------------------
// Cálculo do centro e velocidade média do bando
// -------------------------------------------------------

Vec3 computeFlockCenter() {
    Vec3 center(0,0,0);
    if (boids.empty()) return center;
    for (const auto& b : boids) center += b.pos;
    return center / (float)boids.size();
}

Vec3 computeFlockVelocity() {
    Vec3 v(0,0,0);
    if (boids.empty()) return v;
    for (const auto& b : boids) v += b.vel;
    return v / (float)boids.size();
}

// -------------------------------------------------------
// Desenho da cena
// -------------------------------------------------------

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
        glRotatef(-90.0f, 1, 0, 0); // cone padrão do GLUT é ao longo de +Z
        glColor3f(0.7f, 0.4f, 0.2f);
        glutSolidCone(TOWER_RADIUS, TOWER_HEIGHT, 32, 32);
    glPopMatrix();
}

void drawSingleBoid(const Boid& b) {
    glPushMatrix();
        glTranslatef(b.pos.x, b.pos.y, b.pos.z);

        // Orienta o boid na direção da velocidade (yaw)
        Vec3 v = b.vel;
        if (v.length() < 1e-4f) v = Vec3(0,0,1);
        float yaw = std::atan2(v.x, v.z) * 180.0f / 3.14159265f;
        glRotatef(yaw, 0, 1, 0);

        // Espessura do “corpo” (3D)
        float thickness = 0.25f;

        // Animação das asas (sobe/desce em Y)
        float a = b.wingAngle * 3.14159265f / 180.0f;
        float wingOffset = 0.8f * std::sin(a);

        // Vértices top/bottom do poliedro
        GLfloat noseTop[3]      = { 0.0f,  thickness,  2.0f };
        GLfloat noseBottom[3]   = { 0.0f, -thickness,  2.0f };

        GLfloat tailTop[3]      = { 0.0f,  thickness, -1.8f };
        GLfloat tailBottom[3]   = { 0.0f, -thickness, -1.8f };

        GLfloat leftWingTop[3]  = { -2.0f, wingOffset + thickness,  0.0f };
        GLfloat leftWingBottom[3]= { -2.0f, wingOffset - thickness, 0.0f };

        GLfloat rightWingTop[3]  = {  2.0f, wingOffset + thickness, 0.0f };
        GLfloat rightWingBottom[3]= { 2.0f, wingOffset - thickness, 0.0f };

        // ----- Faces de cima (triângulos) -----
        glBegin(GL_TRIANGLES);
            // “nariz” vermelho (topo)
            glNormal3f(0, 1, 0);
            glColor3f(1.0f, 0.2f, 0.2f);
            glVertex3fv(noseTop);
            glVertex3fv(rightWingTop);
            glVertex3fv(leftWingTop);

            // parte de trás amarela (topo)
            glColor3f(1.0f, 1.0f, 0.0f);
            glVertex3fv(tailTop);
            glVertex3fv(leftWingTop);
            glVertex3fv(rightWingTop);
        glEnd();

        // ----- Faces de baixo (triângulos) -----
        glBegin(GL_TRIANGLES);
            // nariz vermelho (baixo)
            glNormal3f(0, -1, 0);
            glColor3f(1.0f, 0.2f, 0.2f);
            glVertex3fv(noseBottom);
            glVertex3fv(leftWingBottom);
            glVertex3fv(rightWingBottom);

            // parte de trás amarela (baixo)
            glColor3f(1.0f, 1.0f, 0.0f);
            glVertex3fv(tailBottom);
            glVertex3fv(rightWingBottom);
            glVertex3fv(leftWingBottom);
        glEnd();

        // ----- Faces laterais (quads ligando top/bottom) -----
        glBegin(GL_QUADS);
            // Lado entre nariz e asa direita
            glNormal3f(1, 0, 0);
            glVertex3fv(noseTop);
            glVertex3fv(noseBottom);
            glVertex3fv(rightWingBottom);
            glVertex3fv(rightWingTop);

            // Lado entre nariz e asa esquerda
            glNormal3f(-1, 0, 0);
            glVertex3fv(noseTop);
            glVertex3fv(leftWingTop);
            glVertex3fv(leftWingBottom);
            glVertex3fv(noseBottom);

            // Lado asa direita – cauda
            glNormal3f(1, 0, 0);
            glVertex3fv(rightWingTop);
            glVertex3fv(rightWingBottom);
            glVertex3fv(tailBottom);
            glVertex3fv(tailTop);

            // Lado asa esquerda – cauda
            glNormal3f(-1, 0, 0);
            glVertex3fv(leftWingTop);
            glVertex3fv(tailTop);
            glVertex3fv(tailBottom);
            glVertex3fv(leftWingBottom);

            // Lado central (embaixo do corpo, ligando nariz a cauda)
            glNormal3f(0, 0, -1);
            glVertex3fv(noseTop);
            glVertex3fv(tailTop);
            glVertex3fv(tailBottom);
            glVertex3fv(noseBottom);
        glEnd();

    glPopMatrix();
}


void drawBoids() {
    for (const auto& b : boids) {
        drawSingleBoid(b);
    }
}

void drawObstacles() {
    for (const auto& o : obstacles) {
        glPushMatrix();
            if (!o.isCone) {
                // Esfera
                glTranslatef(o.pos.x, o.pos.y, o.pos.z);
                glColor3f(0.2f, 0.2f, 0.9f);
                glutSolidSphere(o.radius, 24, 24);
            } else {
                // Cone (base em o.pos)
                glTranslatef(o.pos.x, o.pos.y, o.pos.z);
                glRotatef(-90.0f, 1, 0, 0);
                glColor3f(0.7f, 0.2f, 0.7f);
                glutSolidCone(o.radius, o.height, 24, 24);
            }
        glPopMatrix();
    }
}

void drawTarget() {
    glPushMatrix();
        glTranslatef(targetPos.x, targetPos.y, targetPos.z);
        glColor3f(0.1f, 0.4f, 0.9f);
        glutSolidSphere(1.0f, 16, 16);
    glPopMatrix();
}

// -------------------------------------------------------
// Câmera
// -------------------------------------------------------

void setupCamera() {
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    Vec3 center = computeFlockCenter();
    Vec3 flockVel = computeFlockVelocity();
    if (flockVel.length() < 1e-4f) flockVel = Vec3(0,0,1);
    Vec3 eye;

    if (cameraMode == 1) {
        // Olho no topo da torre
        eye = Vec3(0.0f, TOWER_HEIGHT + 2.0f, 0.0f);
    } else if (cameraMode == 2) {
        // Atrás do bando
        Vec3 dir = normalize(flockVel);
        eye = center - dir * cameraDist + Vec3(0.0f, cameraHeight, 0.0f);
    } else {
        // cameraMode == 3 -> lateral (perpendicular à velocidade, paralelo ao plano do chão)
        Vec3 velXZ(flockVel.x, 0.0f, flockVel.z);
        if (velXZ.length() < 1e-4f) velXZ = Vec3(0,0,1);
        velXZ = normalize(velXZ);
        // lateral = up x dir
        Vec3 up(0,1,0);
        Vec3 lateral(
            up.y * velXZ.z - up.z * velXZ.y,
            up.z * velXZ.x - up.x * velXZ.z,
            up.x * velXZ.y - up.y * velXZ.x
        );
        lateral = normalize(lateral);
        eye = center + lateral * cameraDist + Vec3(0.0f, cameraHeight, 0.0f);
    }

    // A direção de visualização é do olho para o centro do bando
    gluLookAt(eye.x, eye.y, eye.z,
              center.x, center.y, center.z,
              0.0f, 1.0f, 0.0f);
}

// -------------------------------------------------------
// Iluminação
// -------------------------------------------------------

void setupLighting() {
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_NORMALIZE);

    // >>> IMPORTANTE: deixar o glColor controlar a cor do material
    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);

    GLfloat ambient[]  = {0.2f, 0.2f, 0.2f, 1.0f};
    GLfloat diffuse[]  = {0.8f, 0.8f, 0.8f, 1.0f};
    GLfloat specular[] = {0.5f, 0.5f, 0.5f, 1.0f};
    GLfloat position[] = {30.0f, 50.0f, 30.0f, 1.0f};

    glLightfv(GL_LIGHT0, GL_AMBIENT,  ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE,  diffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, specular);
    glLightfv(GL_LIGHT0, GL_POSITION, position);

    // Material base branco para não “cinzentar” as cores
    GLfloat mat_ambient[]  = {1.0f, 1.0f, 1.0f, 1.0f};
    GLfloat mat_diffuse[]  = {1.0f, 1.0f, 1.0f, 1.0f};
    GLfloat mat_specular[] = {0.4f, 0.4f, 0.4f, 1.0f};
    GLfloat shininess[]    = {16.0f};

    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT,   mat_ambient);
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE,   mat_diffuse);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,  mat_specular);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, shininess);
}

// -------------------------------------------------------
// Atualização da simulação
// -------------------------------------------------------

void updateBoids() {
    if (boids.empty()) return;

    std::vector<Vec3> accelerations(boids.size(), Vec3(0,0,0));

    for (int i = 0; i < (int)boids.size(); ++i) {
        Vec3 sep  = computeSeparation(i) * weightSeparation;
        Vec3 ali  = computeAlignment(i)  * weightAlignment;
        Vec3 coh  = computeCohesion(i)   * weightCohesion;
        Vec3 tgt  = computeTargetAttraction(i) * weightTarget;
        Vec3 bnd  = computeBounds(i);
        Vec3 obs  = computeObstacleAvoidance(i) * weightObstacle; // <<< NOVO

        // Obstáculos têm peso grande, podem "quebrar" o alinhamento/cohesão
        Vec3 acc = sep + ali + coh + tgt + bnd + obs;
        accelerations[i] = acc;
    }

    for (int i = 0; i < (int)boids.size(); ++i) {
        boids[i].vel += accelerations[i] * dt;
        boids[i].vel = limit(boids[i].vel, maxSpeed);
        boids[i].pos += boids[i].vel * dt;

        // Animação das asas
        boids[i].wingAngle += boids[i].wingDir * wingSpeed * dt;
        if (boids[i].wingAngle > wingMaxAngle) {
            boids[i].wingAngle = wingMaxAngle;
            boids[i].wingDir   = -1.0f;
        } else if (boids[i].wingAngle < wingMinAngle) {
            boids[i].wingAngle = wingMinAngle;
            boids[i].wingDir   = 1.0f;
        }
    }
}

void updateTarget() {
    // Limita velocidade do boid-objetivo
    targetVel = limit(targetVel, maxSpeed * 1.2f);
    targetPos += targetVel * dt;

    // Mantém alvo dentro do mundo e acima do chão
    if (targetPos.x >  WORLD_SIZE) { targetPos.x =  WORLD_SIZE; targetVel.x *= -0.5f; }
    if (targetPos.x < -WORLD_SIZE) { targetPos.x = -WORLD_SIZE; targetVel.x *= -0.5f; }
    if (targetPos.z >  WORLD_SIZE) { targetPos.z =  WORLD_SIZE; targetVel.z *= -0.5f; }
    if (targetPos.z < -WORLD_SIZE) { targetPos.z = -WORLD_SIZE; targetVel.z *= -0.5f; }
    if (targetPos.y < 5.0f)        { targetPos.y = 5.0f;        targetVel.y *= -0.5f; }
    if (targetPos.y > WORLD_SIZE*0.5f) {
        targetPos.y = WORLD_SIZE*0.5f;
        targetVel.y *= -0.5f;
    }

    // Um pequeno “atrito” para não acumular velocidade infinita
    targetVel *= 0.99f;
}

// -------------------------------------------------------
// Callbacks GLUT
// -------------------------------------------------------

void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    setupCamera();
    setupLighting();

    drawFloor();
    drawTower();
    drawObstacles();
    drawTarget();
    drawBoids();

    glutSwapBuffers();
}

void reshape(int w, int h) {
    if (h == 0) h = 1;
    float aspect = (float)w / (float)h;
    glViewport(0,0,w,h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0, aspect, 0.1, 500.0);
    glMatrixMode(GL_MODELVIEW);
}

void timer(int value) {
    updateTarget();
    updateBoids();

    glutPostRedisplay();
    glutTimerFunc(16, timer, 0); // ~60 FPS
}

// Teclado normal
void keyboard(unsigned char key, int x, int y) {
    switch (key) {
    case 27: // ESC
    case 'q':
    case 'Q':
        std::exit(0);
        break;

    case '1':
        cameraMode = 1;
        break;
    case '2':
        cameraMode = 2;
        break;
    case '3':
        cameraMode = 3;
        break;

    case 'w':
    case 'W':
        targetVel.y += 2.0f;   // sobe
        break;

    case 's':
    case 'S':
        targetVel.y -= 2.0f;   // desce
        break;

    case '+':
    case '=': {
        // adiciona boid perto do centro do bando
        Vec3 center = computeFlockCenter();
        Boid b;
        b.pos = center + Vec3(frand(-5,5), frand(2,15), frand(-5,5));
        b.vel = Vec3(frand(-2,2), frand(-1,1), frand(-2,2));
        b.wingAngle = frand(wingMinAngle, wingMaxAngle);
        b.wingDir   = (frand(0,1) < 0.5f)? 1.0f : -1.0f;
        boids.push_back(b);
        break;
    }

    case '-':
    case '_':
        if (!boids.empty()) {
            int idx = rand() % boids.size();
            boids.erase(boids.begin() + idx);
        }
        break;

    case 'r':
    case 'R':
        // reset simples: volta alvo e bando
        boids.clear();
        targetPos = Vec3(0.0f, 10.0f, 0.0f);
        targetVel = Vec3(0.5f, 0.0f, 0.0f);
        for (int i = 0; i < initialBoids; ++i) {
            Boid b;
            b.pos = Vec3(frand(-10,10), frand(5,20), frand(-10,10));
            b.vel = Vec3(frand(-3,3), frand(-1,1), frand(-3,3));
            b.wingAngle = frand(wingMinAngle, wingMaxAngle);
            b.wingDir   = (frand(0,1) < 0.5f)? 1.0f : -1.0f;
            boids.push_back(b);
        }
        break;
    }
}

// Teclas especiais (setas) para controlar o boid-objetivo
void specialKeyboard(int key, int x, int y) {
    float acc = 2.0f;

    switch (key) {
    case GLUT_KEY_UP:    // mover para -Z
        targetVel.z -= acc;
        break;
    case GLUT_KEY_DOWN:  // +Z
        targetVel.z += acc;
        break;
    case GLUT_KEY_LEFT:  // -X
        targetVel.x -= acc;
        break;
    case GLUT_KEY_RIGHT: // +X
        targetVel.x += acc;
        break;
    }
}

// -------------------------------------------------------
// Inicialização
// -------------------------------------------------------

void initBoids() {
    boids.clear();
    for (int i = 0; i < initialBoids; ++i) {
        Boid b;
        b.pos = Vec3(frand(-10,10), frand(5,20), frand(-10,10));
        b.vel = Vec3(frand(-3,3), frand(-1,1), frand(-3,3));
        b.wingAngle = frand(wingMinAngle, wingMaxAngle);
        b.wingDir   = (frand(0,1) < 0.5f)? 1.0f : -1.0f;
        boids.push_back(b);
    }
}

void initObstacles() {
    obstacles.clear();

    // Esfera 1
    Obstacle s1;
    s1.pos    = Vec3(15.0f, 8.0f,  5.0f); // centro da esfera
    s1.radius = 4.0f;
    s1.isCone = false;
    s1.height = 0.0f;
    obstacles.push_back(s1);

    // Esfera 2
    Obstacle s2;
    s2.pos    = Vec3(-10.0f, 6.0f, -18.0f);
    s2.radius = 3.0f;
    s2.isCone = false;
    s2.height = 0.0f;
    obstacles.push_back(s2);

    // Cone 1 (base no chão)
    Obstacle c1;
    c1.pos    = Vec3(-20.0f, FLOOR_Y, 10.0f); // centro da base
    c1.radius = 3.0f;   // raio da base
    c1.height = 10.0f;  // altura
    c1.isCone = true;
    obstacles.push_back(c1);

    // (Opcional) torre como obstáculo também
    Obstacle towerObs;
    towerObs.pos    = Vec3(0.0f, FLOOR_Y, 0.0f); // base da torre
    towerObs.radius = TOWER_RADIUS + 1.0f;
    towerObs.height = TOWER_HEIGHT;
    towerObs.isCone = true;
    obstacles.push_back(towerObs);
}


void initGL() {
    glEnable(GL_DEPTH_TEST);
    glShadeModel(GL_SMOOTH);
    glClearColor(0.5f, 0.7f, 1.0f, 1.0f); // céu

    setupLighting();
}

// -------------------------------------------------------
// main
// -------------------------------------------------------

int main(int argc, char** argv) {
    std::srand((unsigned)std::time(nullptr));

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(1024, 768);
    glutCreateWindow("Boids 3D - Computacao Grafica");

    initGL();
    initBoids();
    initObstacles();

    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);
    glutSpecialFunc(specialKeyboard);
    glutTimerFunc(16, timer, 0);

    glutMainLoop();
    return 0;
}
