#include "Adafruit_Arcada.h"
Adafruit_Arcada arcada;
uint16_t *framebuffer;

#define PU8  const int  *
#define P8   const int   * 
#define PU16 const unsigned int *

// INCLUDE MODEL DATA
//#include "sphere.h"
#include "bunny.h"
//#include "small_bunny.h"
//#include "face.h"


const float AXLEN = 0.5;
const float MIND = 0.01;

const int delay_time = 1;
long int delay_timer = 0;

unsigned int X0 = 80;
unsigned int Y0 = 64;

void setLocation(unsigned int x0, unsigned int y0) {
    X0 = x0;
    Y0 = y0;
}

void drawVertices(Model *M, int *vertices, GFXcanvas16 *_mycanvas) {
    unsigned int nv = (M->NVertices);
    int color = 0xFFFF;
    _mycanvas->fillScreen(ARCADA_BLACK);
    for (int i=0; i<nv; i++) 
        _mycanvas->writePixel(vertices[i*3]+X0,vertices[i*3+1]+Y0, color);
}

void drawEdges(Model *M, int *vertices) {
    if (M->edges==NULL) {
        drawMesh(M,vertices);
        return;
    }
    unsigned int ne = M->NEdges;
    int color = 0xFFFF;
    for (int i=0; i<ne; i++) {
        PU8 e = &M->edges[i*2];
        int a = e[0];
        int b = e[1];
        int *p = &vertices[a*3];
        int *q = &vertices[b*3];
        arcada.display->drawLine(p[0]+X0,p[1]+Y0,q[0]+X0,q[1]+Y0, color);
    }
}

void get_triangle_points(Model *M, int *vertices, uint i, int **p, int **q, int **r) {
    // Get the vertex indecies for the triangle
    PU8 t = &M->faces[i*3];
    uint pi = t[0];
    uint qi = t[1];
    uint ri = t[2];
    // get the  X and Y coordinates for the triangle
    *p = &vertices[pi*3];
    *q = &vertices[qi*3];
    *r = &vertices[ri*3];
}

unsigned int facing_camera(int *p, int *q, int *r) {
    return (int)(r[0]-p[0])*(q[1]-p[1])<(int)(q[0]-p[0])*(r[1]-p[1]);
}

void drawMesh(Model *M, int *vertices) {
    unsigned int nt = M->NFaces;
    int color = 0xFFFF;
    for (int i=0; i<nt; i++) {
        int *p,*q,*r;
        get_triangle_points(M,vertices,i,&p,&q,&r);
        if (facing_camera(p,q,r)) {
            arcada.display->drawTriangle(p[0]+X0,p[1]+Y0,q[0]+X0,q[1]+Y0, r[0]+X0,r[1]+Y0, color);
        }
    }
}


void drawMeshSorted(Model *M, int *vertices, unsigned int *draw_order, GFXcanvas16 *_mycanvas) {
    updateDrawingOrder(M,vertices,draw_order);
    unsigned int nt = M->NFaces;
    unsigned int color = 0xFFFF;
    _mycanvas->fillScreen(ARCADA_BLACK);
    for (int j=0; j<nt; j++) {
        int i = draw_order? draw_order[j] : j;
        int *p,*q,*r;
        get_triangle_points(M,vertices,i,&p,&q,&r);
        if (facing_camera(p,q,r)) {
            _mycanvas->drawTriangle(p[0]+X0, p[1]+Y0, q[0]+X0, q[1]+Y0, r[0]+X0, r[1]+Y0, color);
        }
    }
}

void fillFaces(Model *M, int *vertices, unsigned int *face_colors, unsigned int *draw_order, GFXcanvas16 *_mycanvas) {
    updateDrawingOrder(M,vertices,draw_order);
    unsigned int nt = M->NFaces;
    unsigned int color = 0;
    _mycanvas->fillScreen(ARCADA_BLACK);
    //for (int j=0; j<nt; j++) {
    for (int j = nt - 1; j >= 0; j--) {
        int i = draw_order? draw_order[j] : j;
        int *p,*q,*r;
        unsigned int fc = face_colors[i] + 50;
        get_triangle_points(M,vertices,i,&p,&q,&r);
        if (facing_camera(p,q,r)) {
            if (face_colors != NULL) {
                color = color = arcada.display->color565(fc, fc, fc);
                unsigned int x1 = p[0]+X0;
                unsigned int x2 = q[0]+X0;
                unsigned int x3 = r[0]+X0;
                _mycanvas->fillTriangle(x1, p[1]+Y0, x2, q[1]+Y0, x3, r[1]+Y0, __builtin_bswap16(color));
            }
        }
    }
}

void getScaleTransform(float AXLEN, float *abuff1) {
  for (uint i=0; i<9; i++) abuff1[i]=0;
  abuff1[0] = abuff1[4] = abuff1[8] = AXLEN;
}


void rotateTransformXY(float *input_transform, float dx, float dy, float *output_transform) {
    float cdx = cos(dx);
    float sdx = sin(dx);
    float cdy = cos(dy);
    float sdy = sin(dy);
    for (int j=0; j<3; j++) {
        float *a = &input_transform[j*3];
        float *b = &output_transform[j*3];
        float x = a[0];
        float y = a[1];
        float z = a[2];
        float nz = cdx*z - sdx*x;      
        b[0] = cdx*x  + sdx*z;
        b[1] = cdy*y  + sdy*nz;
        b[2] = cdy*nz - sdy*y;
    }
}

void transformPoint(float *transform,P8 p,int *q) {
    int nx = p[0];
    int ny = p[1];
    int nz = p[2];
    for (uint i=0; i<3; i++)
        q[i] = nx*transform[i]+ny*transform[i+3]+nz*transform[i+6];
}

int getZNormal(float *transform, P8 normal) {
    int p[3];
    transformPoint(transform,normal,&p[0]);
    return p[2];
}

void applyTransform(Model *M, float *transform, int *vertices) {
    unsigned int nv = M->NVertices;
    for (int i=0; i<nv; i++) 
        transformPoint(transform,&M->vertices[i*3],&vertices[i*3]);
}

inline float inverseMagnitude(float x, float y, float z) {
    return 1.0/sqrt(x*x+y*y+z*z);
}


void normalizeTransform(float *transform, float *output) {
    for (uint j=0; j<3; j++) {
        float scale = inverseMagnitude(transform[j],transform[j+3],transform[j+6]);
        for (uint i=j; i<9; i+=3) output[i] = transform[i]*scale;
    }
}

void project_normals(float *transform, P8 normals,unsigned int n,uint *output) {
    float normalized[9];
    normalizeTransform(transform,normalized);
    for (unsigned int i=0; i<n; i++) {
        int z = getZNormal(normalized,&normals[i*3]);
        output[i] = z<0?0:z;
    }
}

void computeVertexLightingColors(Model *M, float *transform, unsigned int *vertex_colors) {
    project_normals(transform,M->vertexNormals,M->NVertices,vertex_colors);
}

void computeFaceLightingColors(Model *M, float *transform, unsigned int *face_colors) {
    project_normals(transform,M->faceNormals,M->NFaces,face_colors);
}

void computeTriangleDepths(Model *M, int *vertices, uint *draw_order, uint *depths) {
    unsigned int nt = M->NFaces;
    for (int j=0; j<nt; j++) {
        int i = draw_order!=NULL? draw_order[j]:j;
        int *p,*q,*r;
        get_triangle_points(M,vertices,i,&p,&q,&r);
        // get the rotated vertex Z coordinates for the triangle
        int z = (p[2]+q[2]+r[2])/3;
        depths[j] = z;
    }
}

/**
 *
 * Sort triangles from front to back to properly handle occlusions. Bubble sort
 * is in fact the efficient solution here. It is O(N) for sorted data, and 
 * requires no additional memory to sort. Triangles remain mostly sorted as 
 * object rotates.
 */
void updateDrawingOrder(Model *M, int *vertices, unsigned int *draw_order) {
    if (draw_order==NULL) return;
    unsigned int nt = M->NFaces;
    unsigned int depths[nt];
    computeTriangleDepths(M,vertices,draw_order,depths);
    // Bubble sort the triangles by depth keeping track of the permutation
    uint sorted = 0;
    while (!sorted) {
        sorted = 1;
        for (int i=1;i<nt;i++) {
            int d1 = depths[i-1];
            int d2 = depths[i];
            if (d2>d1) {
                depths[i-1] = d2;
                depths[i]   = d1;
                uint temp    = draw_order[i];
                draw_order[i]   = draw_order[i-1];
                draw_order[i-1] = temp;
                sorted = 0;
            }
        }
    }
}

void setup() {
  Serial.begin(115200);
  // while (!Serial); delay(100);
  Serial.println("PixelDust demo");
  // Start TFT and fill black
  if (!arcada.arcadaBegin()) {
    Serial.print("Failed to begin");
    while (1);
  }
  arcada.displayBegin();
  arcada.display->fillScreen(ARCADA_BLUE);
  
  // Turn on backlight
  arcada.setBacklight(255);

  if (! arcada.createFrameBuffer(ARCADA_TFT_WIDTH, ARCADA_TFT_HEIGHT)) {
    arcada.haltBox("Could not allocate framebuffer");
  }
  framebuffer = arcada.getFrameBuffer();

}

void loop() {	
  model();
}

void model() {

  init_model();
  Model *M = &model_data;
  
  // Rotated versions of the points are buffered here
  int buff[NVERTICES*2*3];
  int *vbuff1 = &buff[0];
  int *vbuff2 = &buff[NVERTICES*3];

  // Define 3D axis. We rotate this based on touch input
  float axis[4*2*3];
  float *abuff1 = &axis[0];
  float *abuff2 = &axis[4*3]; 
  getScaleTransform(AXLEN,abuff1);
  applyTransform(M,abuff1,vbuff1);
  applyTransform(M,abuff1,vbuff2);

  // Initialize permutation. We keep this across frames
  // Because sorting a mostly ordered set is faster. 
  unsigned int permutation[NTRIANGLES];
  for (unsigned int i=0; i<NTRIANGLES; i++) permutation[i]=i;

  GFXcanvas16 *canvas = arcada.getCanvas();
  
  while (1) 
  {
    float dx = arcada.readJoystickX() / 4096.0; 
    float dy = arcada.readJoystickY() / 4096.0;


    // If there is a sufficient change, update the model
    if (dx<-MIND || dx>MIND || dy<-MIND || dy>MIND) {
        rotateTransformXY(abuff1,dx,dy,abuff2);
        {float *temp2 = abuff1; abuff1 = abuff2; abuff2 = temp2;}
        applyTransform(M,abuff2,vbuff1);
        {int *temp = vbuff1; vbuff1 = vbuff2; vbuff2 = temp;}
    }
    // mesh render - backface culling but no hidden line removal
    //drawMeshSorted(M, vbuff2, permutation, canvas);
    // point render
    //drawVertices(M, vbuff2, canvas);
    // facet render
    unsigned int face_colors[NTRIANGLES];
    computeFaceLightingColors(M, abuff2, face_colors);
    fillFaces(M, vbuff2, face_colors, permutation, canvas); 
    arcada.blitFrameBuffer(0, 0, true, true);
   
  }
}
