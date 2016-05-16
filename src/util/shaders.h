#ifndef _UMF_SHADERS_H_
#define _UMF_SHADERS_H_

//shader for drawing lines (Use OpenCV renderer instead)
extern const char* shLineVertex;
extern const char* shLineFragment;

// Add texture coordinates to Vertex structure as follows
typedef struct {
    float Position[3];
    float TexCoord[2];
} QuadVertex;

//Full screen quad for rendering
extern const QuadVertex QuadFullScreen[4];

//Full screen quad with Y (vertical) mirrored
extern const QuadVertex QuadReverseFullScreen[4];

//Basic vertex shader used by all consecutive fragment shader
extern const char* shQuadVertex;

extern const char* shQuadFragment; //simple background renderer using normal texture
extern const char* shQuadNV21Fragment; //background renderer that uses 2 textures (one for Y) and a 2dim UV texture
extern const char* shQuadNV21FragmentGreen; //background renderer with chromakeying
extern const char* shQuadESExtTexFragment; //background renderer using external texture (Android)
extern const char* shQuadESExtTexFragmentGreen; //background renderer with a single chromekying color
extern const char* shQuadESExtTexFragmentGreen2; //more andvanced chromakeying using 3 colors


#endif