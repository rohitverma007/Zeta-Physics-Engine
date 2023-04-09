#ifndef SCENE_H
#define SCENE_H

#include "gunslinger/gs.h"
#include <vector>
#include "bodies.h"

#ifdef GS_PLATFORM_WEB
    #define GS_GL_VERSION_STR "#version 300 es\n"
#else
    #define GS_GL_VERSION_STR "#version 330 core\n"
#endif


struct VertexData{
    gs_vec3 pos;
    gs_vec3 aNormal;
    gs_vec2 textCoord;
    gs_vec4 color;
};

struct RectPrismData{
    // each this vectors should be 6 in length
    VertexData leftVertices[6];   // left
    VertexData rightVertices[6];  // right
    VertexData topVertices[6];    // top
    VertexData bottomVertices[6]; // bottom
    VertexData backVertices[6];   // back
    VertexData frontVertices[6];  // front
};

static const char* v_src =
    GS_GL_VERSION_STR
    "layout(location = 0) in vec3 a_pos;\n"
    "layout(location = 1) in vec3 a_normal;\n"
    "layout(location = 2) in vec2 a_texcoord;\n"
    "layout(location = 3) in vec4 a_color;\n"
    "uniform mat4 u_proj;\n"
    "uniform mat4 u_view;\n"
    "uniform mat4 u_model;\n"
    "out vec3 frag_pos;\n"
    "out vec3 normal;\n"
    "out vec2 tex_coord;\n"
    "out vec4 color;\n"
    "void main() {\n"
    // "   frag_pos = vec3(u_model * vec4(a_pos, 1.0));\n"
    "   frag_pos = vec3(vec4(a_pos, 1.0));\n"
    "   normal = mat3(transpose(inverse(u_model))) * a_normal;\n"
    "   tex_coord = a_texcoord;\n"
    "   color = a_color;\n"
    "   gl_Position = u_proj * u_view * vec4(frag_pos, 1.0);\n"
    "}\n";

static const char* f_src =
    GS_GL_VERSION_STR
    "precision mediump float;\n"
    "layout(location = 0) out vec4 frag_color;\n"
    "in vec3 frag_pos;\n"
    "in vec3 normal;\n"
    "in vec2 tex_coord;\n"
    "in vec4 color;\n"
    "struct material_t {\n"
    "   sampler2D diffuse;\n"
    "   sampler2D specular;\n"
    "   float shininess;\n"
    "};\n"
    "struct light_t {\n"
    "   vec3 position;\n"
    "   vec3 ambient;\n"
    "   vec3 diffuse;\n"
    "   vec3 specular;\n"
    "   float constant;\n"
    "   float linear;\n"
    "   float quadratic;\n"
    "};\n"
    "uniform vec3 u_viewpos;\n"
    "uniform material_t u_material;\n"
    "uniform light_t u_light;\n"

    "void main() {\n"
   "    // ambient\n"
    // "   vec3 ambient = u_light.ambient * texture(u_material.diffuse, tex_coord).rgb;\n"
    "   vec3 ambient = u_light.ambient * vec3(color.r, color.g, color.b);\n"
    "                                                                           \n" 
    "   // diffuse\n"
    "   vec3 norm = normalize(normal);\n"
    "   vec3 lightDir = normalize(u_light.position - frag_pos);\n"
    "   float diff = max(dot(norm, lightDir), 0.0);\n"
    "   vec3 diffuse = u_light.diffuse * diff * texture(u_material.diffuse, tex_coord).rgb;\n"
    "                                                                                  \n" 
    "   // specular\n"
    "   vec3 viewDir = normalize(u_viewpos - frag_pos);\n"
    "   vec3 reflectDir = reflect(-lightDir, norm);\n"
    "   float spec = pow(max(dot(viewDir, reflectDir), 0.0), u_material.shininess);\n"
    "   vec3 specular = u_light.specular * spec * texture(u_material.specular, tex_coord).rgb;\n"
    "                                                                                     \n" 
    "   // attenuation\n"
    "   float distance    = length(u_light.position - frag_pos);\n"
    "   float attenuation = 1.0 / (u_light.constant + u_light.linear * distance + u_light.quadratic * (distance * distance));\n"
    "                                                                                                                  \n"
    "   ambient  *= attenuation;\n"
    "   diffuse  *= attenuation;\n"
    "   specular *= attenuation;\n"  
    "                                                                                                                  \n" 
    "   vec3 result = ambient + diffuse + specular;\n"
    // "   result = vec3(255.0f, 0.0f, 0.0f);\n"
    "   frag_color = vec4(result, 1.0);\n"
    "}\n";

struct RectPrism{
    gs_vec3 position;
    gs_vec4 color;
    ZMath::Vec3D* vertices;
};

struct AppState{
    std::vector<RectPrism> rectPrisms;
};

void DrawRectPrism(AppState *appState, ZMath::Vec3D* vertices, gs_vec3 position, gs_vec4 color);
void SetupScene(gs_camera_t cam);
void UpdateScene(AppState *appState, gs_camera_t cam);

gs_vec4 rand_colors[] = {
    {146,215,247, 1},
    {201,31,23, 1},
    {227,59,163, 1},
    {213,138,240, 1},
    {181,221,110, 1},
    {7,195,161, 1},
    {123,232,162, 1},
    {240,205,189, 1},
    {79,27,53, 1},
    {243,112,131, 1},
    {44,109,71, 1},
    {93,255,9, 1},
    {156,117,243, 1},
    {123,165,110, 1},
    {82,198,180, 1},
    {49,72,244, 1},
    {204,83,171, 1},
    {86,34,134, 1},
    {87,54,68, 1},
    {137,142,159, 1},
    {87,161,9, 1},
    {43,116,148, 1},
    {84,162,125, 1},
    {46,116,237, 1},
    {189,251,14, 1},
    {12,54,41, 1},
    {237,237,203, 1},
    {111,31,236, 1},
    {15,105,126, 1},
    {40,12,178, 1},
    {23,59,161, 1},
    {147,10,83, 1},
    {119,209,196, 1},
    {153,117,93, 1},
    {156,127,183, 1},
    {205,102,252, 1},
    {248,204,67, 1},
    {99,172,19, 1},
    {12,177,46, 1},
    {214,45,147, 1},
    {175,0,20, 1},
    {131,53,31, 1},
    {240,7,122, 1},
    {227,145,145, 1},
    {242,98,42, 1},
    {185,151,114, 1},
    {204,200,211, 1},
    {119,244,240, 1},
    {251,224,226, 1},
    {110,253,203, 1},
    {244,173,218, 1},
    {92,23,113, 1},
    {95,247,241, 1},
    {116,174,118, 1},
    {182,190,208, 1},
    {164,179,54, 1},
    {23,155,16, 1},
    {65,168,219, 1},
    {239,23,183, 1},
    {153,50,162, 1},
    {177,77,183, 1},
    {208,118,99, 1},
    {232,75,199, 1},
    {51,104,73, 1},
    {216,223,203, 1},
    {42,29,85, 1},
    {193,45,252, 1},
    {0,72,159, 1},
    {175,205,74, 1},
    {202,243,179, 1},
    {180,158,245, 1},
    {8,204,137, 1},
    {233,167,36, 1},
    {21,93,248, 1},
    {15,252,238, 1},
    {162,51,9, 1},
    {249,74,93, 1},
    {33,57,201, 1},
    {192,89,75, 1},
    {3,167,150, 1},
    {156,108,44, 1},
    {208,249,4, 1},
    {104,100,153, 1},
    {202,192,146, 1},
    {203,163,32, 1},
    {227,123,168, 1},
    {100,10,232, 1},
    {192,226,46, 1},
    {33,212,209, 1},
    {2,48,145, 1},
    {155,161,179, 1},
    {230,74,104, 1},
    {252,249,151, 1},
    {91,0,201, 1},
    {167,230,213, 1},
    {200,66,54, 1},
    {204,236,172, 1},
    {26,149,150, 1},
    {246,187,186, 1},
    {231,128,84, 1},
    {151,95,78, 1},
    {108,74,208, 1},
    {119,203,40, 1},
    {133,71,21, 1},
    {245,93,33, 1},
    {151,53,165, 1},
    {167,210,82, 1},
    {184,104,192, 1},
    {93,8,129, 1},
    {158,53,118, 1},
    {112,165,241, 1},
    {24,103,17, 1},
    {239,169,162, 1},
    {114,0,94, 1},
    {148,24,13, 1},
    {106,249,122, 1},
    {212,182,215, 1},
    {239,169,255, 1},
    {167,15,58, 1},
    {113,178,178, 1},
    {7,118,173, 1},
    {200,45,50, 1},
    {173,91,165, 1},
    {179,41,72, 1},
    {231,197,245, 1},
    {252,240,220, 1},
    {34,185,230, 1},
    {55,46,58, 1},
    {235,139,223, 1},
    {22,56,179, 1},
    {28,4,236, 1},
    {133,98,33, 1},
    {48,172,181, 1},
    {153,199,255, 1},
    {215,13,42, 1},
    {87,252,211, 1},
    {146,103,145, 1},
    {211,132,202, 1},
    {210,199,116, 1},
    {19,17,224, 1},
    {149,195,145, 1},
    {103,84,18, 1},
    {95,56,29, 1},
    {113,128,71, 1},
    {156,71,170, 1},
    {159,185,226, 1},
    {159,53,230, 1},
    {67,116,46, 1},
    {132,39,69, 1},
    {250,236,97, 1},
    {121,171,175, 1},
    {88,60,131, 1},
    {186,155,218, 1},
    {198,150,195, 1},
    {25,134,87, 1},
    {208,30,143, 1},
    {193,36,42, 1},
    {5,128,125, 1},
    {77,63,92, 1},
    {234,174,214, 1},
    {206,177,247, 1},
    {35,40,72, 1},
    {108,195,74, 1},
    {48,71,66, 1},
    {85,249,83, 1},
    {163,205,181, 1},
    {54,240,15, 1},
    {29,90,210, 1},
    {113,250,139, 1},
    {72,82,195, 1},
    {138,36,77, 1},
    {134,249,230, 1},
    {222,53,115, 1},
    {50,106,173, 1},
    {245,125,129, 1},
    {31,208,151, 1},
    {230,85,185, 1},
    {114,114,35, 1},
    {77,231,112, 1},
    {255,59,51, 1},
    {238,185,147, 1},
    {131,115,241, 1},
    {144,145,218, 1},
    {46,214,166, 1},
    {46,37,95, 1},
    {41,58,130, 1},
    {81,110,71, 1},
    {91,166,7, 1},
    {59,15,215, 1},
    {16,37,191, 1},
    {23,156,225, 1},
    {107,148,168, 1},
    {193,13,53, 1},
    {92,37,35, 1},
    {44,126,174, 1},
    {215,204,217, 1},
    {115,192,46, 1},
    {59,71,141, 1},
    {201,171,184, 1},
    {124,221,169, 1},
    {6,84,147, 1},
    {52,222,217, 1},
    {179,254,142, 1},
    {58,229,162, 1},
    {233,153,131, 1},
    {36,130,0, 1},
    {101,107,220, 1},
    {62,182,94, 1},
    {26,149,119, 1},
    {228,1,75, 1},
    {202,85,112, 1},
    {105,119,12, 1},
    {21,78,93, 1},
    {51,51,45, 1},
    {133,76,163, 1},
    {184,246,249, 1},
    {91,26,84, 1},
    {5,140,39, 1},
    {148,19,180, 1},
    {227,176,224, 1},
    {123,215,166, 1},
    {36,145,99, 1},
    {179,246,38, 1},
    {97,5,17, 1},
    {228,219,246, 1},
    {122,249,132, 1},
    {158,205,69, 1},
    {178,30,71, 1},
    {25,190,68, 1},
    {119,191,13, 1},
    {238,82,62, 1},
    {24,201,33, 1},
    {236,242,76, 1},
    {39,112,231, 1},
    {86,123,218, 1},
    {94,241,157, 1},
    {89,109,22, 1},
    {178,68,214, 1},
    {153,180,103, 1},
    {216,182,5, 1},
    {65,14,204, 1},
    {172,36,48, 1},
    {49,67,200, 1},
    {204,179,77, 1},
    {3,200,254, 1},
    {244,181,15, 1},
    {135,121,167, 1},
    {38,138,108, 1},
    {168,209,49, 1},
    {250,176,243, 1},
    {55,64,163, 1},
    {169,223,8, 1},
    {154,140,225, 1},
    {79,235,237, 1},
    {111,59,101, 1},
    {133,224,141, 1},
    {24,200,105, 1},
    {160,185,125, 1},
    {151,193,70, 1},
    {52,122,11, 1},
    {196,128,182, 1},
    {57,226,99, 1},
    {117,171,61, 1},
    {184,194,76, 1},
    {165,4,169, 1},
    {179,16,93, 1},
    {87,223,197, 1},
    {26,253,58, 1},
    {244,8,169, 1},
    {136,79,199, 1},
    {101,243,147, 1},
    {44,112,66, 1},
    {34,99,37, 1},
    {228,48,121, 1},
    {86,178,111, 1},
    {20,4,221, 1},
    {72,131,251, 1},
    {134,121,82, 1},
    {197,74,248, 1},
    {86,195,139, 1},
    {111,109,239, 1},
    {1,157,230, 1},
    {88,8,230, 1},
    {176,129,19, 1},
    {193,3,122, 1},
    {35,250,66, 1},
    {114,42,206, 1},
    {67,81,87, 1},
    {30,50,154, 1},
    {240,136,56, 1},
    {203,157,205, 1},
    {106,224,107, 1},
    {177,200,5, 1},
    {130,108,145, 1},
    {240,76,132, 1},
    {110,34,74, 1},
    {246,225,254, 1},
    {227,134,54, 1},
    {43,122,37, 1},
    {178,108,195, 1},
    {114,103,88, 1},
    {83,131,115, 1},
    {78,255,191, 1},
    {178,55,200, 1},
    {37,102,100, 1},
    {255,13,68, 1},
    {167,175,57, 1},
    {188,69,121, 1},
    {98,38,215, 1},
    {148,146,119, 1},
    {108,83,240, 1},
    {231,136,209, 1},
    {110,203,47, 1},
    {191,140,62, 1},
    {108,74,215, 1},
    {55,199,245, 1},
    {215,84,135, 1},
    {1,232,105, 1},
    {187,199,232, 1},
    {171,77,133, 1},
    {221,93,164, 1},
    {232,56,136, 1},
    {54,73,61, 1},
    {127,84,99, 1},
    {122,152,140, 1},
    {49,178,19, 1},
    {231,52,97, 1},
    {238,29,192, 1},
    {91,97,97, 1},
    {160,229,234, 1},
    {81,178,67, 1},
    {4,125,124, 1},
    {152,134,52, 1},
    {13,203,136, 1},
    {221,210,238, 1},
    {164,157,53, 1},
    {68,34,202, 1},
    {10,13,33, 1},
    {224,190,209, 1},
    {18,242,242, 1},
    {230,227,167, 1},
    {192,116,98, 1},
    {198,229,18, 1},
    {87,246,100, 1},
    {161,128,123, 1},
    {181,131,145, 1},
    {193,61,75, 1},
    {173,7,159, 1},
    {172,131,103, 1},
    {83,173,227, 1},
    {83,72,196, 1},
    {65,234,182, 1},
    {1,121,194, 1},
    {138,152,139, 1},
    {102,25,120, 1},
    {203,214,181, 1},
    {3,235,169, 1},
    {175,120,126, 1},
    {226,125,158, 1},
    {186,77,180, 1},
    {217,125,194, 1},
    {131,51,202, 1},
    {218,162,120, 1},
    {250,66,211, 1},
    {149,192,153, 1},
    {243,47,90, 1},
    {191,224,43, 1},
    {113,130,238, 1},
    {145,129,13, 1},
    {244,235,1, 1},
    {18,190,247, 1},
    {245,153,71, 1},
    {134,79,242, 1},
    {53,18,218, 1},
    {250,227,117, 1},
    {77,196,147, 1},
    {190,91,201, 1},
    {139,141,249, 1},
    {123,189,12, 1},
    {22,97,245, 1},
    {7,153,123, 1},
    {10,168,47, 1},
    {117,118,106, 1},
    {187,89,54, 1},
    {98,228,47, 1},
    {99,23,97, 1},
    {7,47,73, 1},
    {239,130,195, 1},
    {163,161,110, 1},
    {239,33,126, 1},
    {34,73,251, 1},
    {84,229,222, 1},
    {137,96,140, 1},
    {62,209,134, 1},
    {238,154,102, 1},
    {184,0,112, 1},
    {235,51,162, 1},
    {104,209,88, 1},
    {230,109,111, 1},
    {136,33,153, 1},
    {131,36,193, 1},
    {224,210,86, 1},
    {114,104,29, 1},
    {80,181,157, 1},
    {32,26,206, 1},
    {87,58,164, 1},
    {20,200,137, 1},
    {75,12,56, 1},
    {189,236,54, 1},
    {236,88,84, 1},
    {238,30,65, 1},
    {226,76,28, 1},
    {111,75,33, 1},
    {184,129,11, 1},
    {93,189,195, 1},
    {167,123,204, 1},
    {103,184,163, 1},
    {163,177,2, 1},
    {231,204,1, 1},
    {169,245,12, 1},
    {108,106,194, 1},
    {200,74,16, 1},
    {35,45,244, 1},
    {26,63,163, 1},
    {104,105,45, 1},
    {182,127,150, 1},
    {76,82,94, 1},
    {176,41,247, 1},
    {244,165,191, 1},
    {156,211,129, 1},
    {75,181,98, 1},
    {86,122,203, 1},
    {77,189,146, 1},
    {250,14,97, 1},
    {224,120,25, 1},
    {91,255,22, 1},
    {74,70,171, 1},
    {2,14,217, 1},
    {171,219,163, 1},
    {106,228,120, 1},
    {236,164,199, 1},
    {178,83,7, 1},
    {215,22,106, 1},
    {224,232,203, 1},
    {54,162,126, 1},
    {123,164,13, 1},
    {211,208,40, 1},
    {6,165,100, 1},
    {1,12,61, 1},
    {50,23,213, 1},
    {124,67,245, 1},
    {28,64,92, 1},
    {200,193,125, 1},
    {176,87,59, 1},
    {9,192,162, 1},
    {8,135,158, 1},
    {40,90,83, 1},
    {193,146,36, 1},
    {17,56,43, 1},
    {99,213,37, 1},
    {15,165,171, 1},
    {216,249,121, 1},
    {87,243,133, 1},
    {81,97,146, 1},
    {7,218,153, 1},
    {2,100,16, 1},
    {221,171,12, 1},
    {170,243,129, 1},
    {146,91,176, 1},
    {133,164,255, 1},
    {129,102,185, 1},
    {189,250,253, 1},
    {209,158,38, 1},
    {62,71,92, 1},
    {222,132,225, 1},
    {75,221,121, 1},
    {13,25,250, 1},
    {164,76,43, 1},
    {168,214,79, 1},
    {72,50,247, 1},
    {25,73,218, 1},
    {202,12,253, 1},
    {126,185,45, 1},
    {148,190,199, 1},
    {151,115,236, 1},
    {162,160,71, 1},
    {86,112,61, 1},
    {210,248,112, 1},
    {31,176,86, 1},
    {177,35,25, 1},
    {94,199,110, 1},
    {46,215,41, 1},
    {138,155,161, 1},
    {141,69,89, 1},
    {133,227,191, 1},
    {91,53,113, 1},
    {31,8,27, 1},
    {56,227,171, 1},
    {6,155,12, 1},
    {37,9,86, 1}
};

#endif


#if 0

 float v_data_2[] = {
            // positions                                    // normals           // texture coords    // color    
            // left
            vertices[0].x + pos.x, vertices[0].y + pos.y, vertices[0].z + pos.z,  0.0f,  0.0f, -1.0f,  0.0f,  0.0f,     color.x, color.y, color.z, 1.0f,     
            vertices[1].x + pos.x, vertices[1].y + pos.y, vertices[1].z + pos.z,  0.0f,  0.0f, -1.0f,  1.0f,  0.0f,     color.x, color.y, color.z, 1.0f,
            vertices[5].x + pos.x, vertices[5].y + pos.y, vertices[5].z + pos.z,  0.0f,  0.0f, -1.0f,  1.0f,  1.0f,     color.x, color.y, color.z, 1.0f,
            vertices[5].x + pos.x, vertices[5].y + pos.y, vertices[5].z + pos.z,  0.0f,  0.0f, -1.0f,  1.0f,  1.0f,     color.x, color.y, color.z, 1.0f,
            vertices[1].x + pos.x, vertices[1].y + pos.y, vertices[1].z + pos.z,  0.0f,  0.0f, -1.0f,  0.0f,  1.0f,     color.x, color.y, color.z, 1.0f,
            vertices[4].x + pos.x, vertices[4].y + pos.y, vertices[4].z + pos.z,  0.0f,  0.0f, -1.0f,  0.0f,  0.0f,     color.x, color.y, color.z, 1.0f,

            // right
            vertices[3].x + pos.x, vertices[3].y + pos.y, vertices[3].z + pos.z,  0.0f,  0.0f,  1.0f,  0.0f,  0.0f,     color.x, color.y, color.z, 1.0f,
            vertices[2].x + pos.x, vertices[2].y + pos.y, vertices[2].z + pos.z,  0.0f,  0.0f,  1.0f,  1.0f,  0.0f,     color.x, color.y, color.z, 1.0f,
            vertices[6].x + pos.x, vertices[6].y + pos.y, vertices[6].z + pos.z,  0.0f,  0.0f,  1.0f,  1.0f,  1.0f,     color.x, color.y, color.z, 1.0f,
            vertices[6].x + pos.x, vertices[6].y + pos.y, vertices[6].z + pos.z,  0.0f,  0.0f,  1.0f,  1.0f,  1.0f,     color.x, color.y, color.z, 1.0f,
            vertices[7].x + pos.x, vertices[7].y + pos.y, vertices[7].z + pos.z,  0.0f,  0.0f,  1.0f,  0.0f,  1.0f,     color.x, color.y, color.z, 1.0f,
            vertices[2].x + pos.x, vertices[2].y + pos.y, vertices[2].z + pos.z,  0.0f,  0.0f,  1.0f,  0.0f,  0.0f,     color.x, color.y, color.z, 1.0f,

            // top
            vertices[5].x + pos.x, vertices[5].y + pos.y, vertices[5].z + pos.z, -1.0f,  0.0f,  0.0f,  1.0f,  0.0f,     color.x, color.y, color.z, 1.0f,
            vertices[4].x + pos.x, vertices[4].y + pos.y, vertices[4].z + pos.z, -1.0f,  0.0f,  0.0f,  1.0f,  1.0f,     color.x, color.y, color.z, 1.0f,
            vertices[7].x + pos.x, vertices[7].y + pos.y, vertices[7].z + pos.z, -1.0f,  0.0f,  0.0f,  0.0f,  1.0f,     color.x, color.y, color.z, 1.0f,
            vertices[7].x + pos.x, vertices[7].y + pos.y, vertices[7].z + pos.z, -1.0f,  0.0f,  0.0f,  0.0f,  1.0f,     color.x, color.y, color.z, 1.0f,
            vertices[6].x + pos.x, vertices[6].y + pos.y, vertices[6].z + pos.z, -1.0f,  0.0f,  0.0f,  0.0f,  0.0f,     color.x, color.y, color.z, 1.0f,
            vertices[5].x + pos.x, vertices[5].y + pos.y, vertices[5].z + pos.z, -1.0f,  0.0f,  0.0f,  1.0f,  0.0f,     color.x, color.y, color.z, 1.0f,

            // bottom
            vertices[0].x + pos.x, vertices[0].y + pos.y, vertices[0].z + pos.z,  1.0f,  0.0f,  0.0f,  1.0f,  0.0f,     color.x, color.y, color.z, 1.0f,
            vertices[1].x + pos.x, vertices[1].y + pos.y, vertices[1].z + pos.z,  1.0f,  0.0f,  0.0f,  1.0f,  1.0f,     color.x, color.y, color.z, 1.0f,
            vertices[2].x + pos.x, vertices[2].y + pos.y, vertices[2].z + pos.z,  1.0f,  0.0f,  0.0f,  0.0f,  1.0f,     color.x, color.y, color.z, 1.0f,
            vertices[2].x + pos.x, vertices[2].y + pos.y, vertices[2].z + pos.z,  1.0f,  0.0f,  0.0f,  0.0f,  1.0f,     color.x, color.y, color.z, 1.0f,
            vertices[0].x + pos.x, vertices[0].y + pos.y, vertices[0].z + pos.z,  1.0f,  0.0f,  0.0f,  0.0f,  0.0f,     color.x, color.y, color.z, 1.0f,
            vertices[3].x + pos.x, vertices[3].y + pos.y, vertices[3].z + pos.z,  1.0f,  0.0f,  0.0f,  1.0f,  0.0f,     color.x, color.y, color.z, 1.0f,

            // back
            vertices[0].x + pos.x, vertices[0].y + pos.y, vertices[0].z + pos.z,  0.0f, -1.0f,  0.0f,  0.0f,  1.0f,     color.x, color.y, color.z, 1.0f,
            vertices[5].x + pos.x, vertices[5].y + pos.y, vertices[5].z + pos.z,  0.0f, -1.0f,  0.0f,  1.0f,  1.0f,     color.x, color.y, color.z, 1.0f,
            vertices[6].x + pos.x, vertices[6].y + pos.y, vertices[6].z + pos.z,  0.0f, -1.0f,  0.0f,  1.0f,  0.0f,     color.x, color.y, color.z, 1.0f,
            vertices[6].x + pos.x, vertices[6].y + pos.y, vertices[6].z + pos.z,  0.0f, -1.0f,  0.0f,  1.0f,  0.0f,     color.x, color.y, color.z, 1.0f,
            vertices[0].x + pos.x, vertices[0].y + pos.y, vertices[0].z + pos.z,  0.0f, -1.0f,  0.0f,  0.0f,  0.0f,     color.x, color.y, color.z, 1.0f,
            vertices[3].x + pos.x, vertices[3].y + pos.y, vertices[3].z + pos.z,  0.0f, -1.0f,  0.0f,  0.0f,  1.0f,     color.x, color.y, color.z, 1.0f,
            
            // front
            vertices[1].x + pos.x, vertices[1].y + pos.y, vertices[1].z + pos.z,  0.0f,  1.0f,  0.0f,  0.0f,  1.0f,     color.x, color.y, color.z, 1.0f,
            vertices[4].x + pos.x, vertices[4].y + pos.y, vertices[4].z + pos.z,  0.0f,  1.0f,  0.0f,  1.0f,  1.0f,     color.x, color.y, color.z, 1.0f,
            vertices[2].x + pos.x, vertices[2].y + pos.y, vertices[2].z + pos.z,  0.0f,  1.0f,  0.0f,  1.0f,  0.0f,     color.x, color.y, color.z, 1.0f,
            vertices[2].x + pos.x, vertices[2].y + pos.y, vertices[2].z + pos.z,  0.0f,  1.0f,  0.0f,  1.0f,  0.0f,     color.x, color.y, color.z, 1.0f,
            vertices[4].x + pos.x, vertices[4].y + pos.y, vertices[4].z + pos.z,  0.0f,  1.0f,  0.0f,  0.0f,  0.0f,     color.x, color.y, color.z, 1.0f,
            vertices[7].x + pos.x, vertices[7].y + pos.y, vertices[7].z + pos.z,  0.0f,  1.0f,  0.0f,  0.0f,  1.0f,     color.x, color.y, color.z, 1.0f
        };  




#endif
