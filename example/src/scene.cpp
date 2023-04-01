#include "scene.h"

gs_command_buffer_t commandBuffer = {};
gs_graphics_bind_desc_t binds = {};

gs_handle(gs_graphics_pipeline_t) pip = {};
gs_handle(gs_graphics_shader_t) shader = {};
gs_handle(gs_graphics_vertex_buffer_t) vbo = {};
gs_handle(gs_graphics_uniform_t) u_model = {};

gs_handle(gs_graphics_texture_t)         t_diffuse  = {0};
gs_handle(gs_graphics_texture_t)         t_specular = {0};
gs_handle(gs_graphics_uniform_t)         u_view     = {0};
gs_handle(gs_graphics_uniform_t)         u_proj     = {0};
gs_handle(gs_graphics_uniform_t)         u_viewpos  = {0};
gs_handle(gs_graphics_uniform_t)         u_light    = {0};
gs_handle(gs_graphics_uniform_t)         u_material = {0};

// Structure to match light params definition in shader
struct light_params_t {
    gs_vec3 position;
    gs_vec3 ambient;
    gs_vec3 diffuse;
    gs_vec3 specular;
    float constant;
    float linear;
    float quadratic;
};

struct material_params_t {
    gs_handle(gs_graphics_texture_t) diffuse;
    gs_handle(gs_graphics_texture_t) specular;
    float shininess;
};


#define ROW_COL_CT  10

const char *temp_src = "hello world";
gs_platform_t* platform = 0;

void SetupScene(gs_camera_t cam){

    commandBuffer = gs_command_buffer_new();
    
    // Construct vertex buffer
    gs_graphics_vertex_buffer_desc_t vb_desc  = {};

    vb_desc.data = v_data;
    vb_desc.size = sizeof(v_data);

    vbo = gs_graphics_vertex_buffer_create(&vb_desc);
    
    // Generate procedural texture data (checkered texture)
    gs_color_t c0 = GS_COLOR_WHITE;
    gs_color_t c1 = gs_color(20, 50, 150, 255);
    gs_color_t pixels[ROW_COL_CT * ROW_COL_CT] = gs_default_val();
    for (uint32_t r = 0; r < ROW_COL_CT; ++r) {
        for (uint32_t c = 0; c < ROW_COL_CT; ++c) {
            const bool re = (r % 2) == 0;
            const bool ce = (c % 2) == 0;
            uint32_t idx = r * ROW_COL_CT + c;
            pixels[idx] = (re && ce) ? c0 : (re) ? c1 : (ce) ? c1 : c0;
        } 
    }
    

    // Construct individual uniforms
    // model_matrix uniform 
    gs_graphics_uniform_layout_desc_t u_model_layout_desc[1] = {};
    u_model_layout_desc[0].type = GS_GRAPHICS_UNIFORM_MAT4;

    gs_graphics_uniform_desc_t u_model_desc = {};
    u_model_desc.name[0] = 'u';
    u_model_desc.name[1] = '_';
    u_model_desc.name[2] = 'm';
    u_model_desc.name[3] = 'o';
    u_model_desc.name[4] = 'd';
    u_model_desc.name[5] = 'e';
    u_model_desc.name[6] = 'l';

    u_model_desc.layout = u_model_layout_desc; 

    u_model = gs_graphics_uniform_create(&u_model_desc);

    // view matrix uniform
    gs_graphics_uniform_layout_desc_t u_view_layout_desc[1] = {};
    u_view_layout_desc[0].type = GS_GRAPHICS_UNIFORM_MAT4;

    gs_graphics_uniform_desc_t u_view_desc = {};
    u_view_desc.name[0] = 'u';
    u_view_desc.name[1] = '_';
    u_view_desc.name[2] = 'v';
    u_view_desc.name[3] = 'i';
    u_view_desc.name[4] = 'e';
    u_view_desc.name[5] = 'w';

    u_view_desc.layout = u_view_layout_desc;

    u_view = gs_graphics_uniform_create (&u_view_desc);

    // projection matrix uniform
    gs_graphics_uniform_layout_desc_t u_proj_layout_desc[1] = {};
    u_proj_layout_desc[0].type = GS_GRAPHICS_UNIFORM_MAT4;

    gs_graphics_uniform_desc_t u_proj_desc = {};
    u_proj_desc.name[0] = 'u';
    u_proj_desc.name[1] = '_';
    u_proj_desc.name[2] = 'p';
    u_proj_desc.name[3] = 'r';
    u_proj_desc.name[4] = 'o';
    u_proj_desc.name[5] = 'j';

    u_proj_desc.layout = u_proj_layout_desc;

    u_proj = gs_graphics_uniform_create (&u_proj_desc);


    // view matrix position
    gs_graphics_uniform_layout_desc_t u_viewpos_layout_desc[1] = {};
    u_viewpos_layout_desc[0].type = GS_GRAPHICS_UNIFORM_VEC3;

    gs_graphics_uniform_desc_t u_viewpos_desc = {};
    u_viewpos_desc.name[0] = 'u';
    u_viewpos_desc.name[1] = '_';
    u_viewpos_desc.name[2] = 'v';
    u_viewpos_desc.name[3] = 'i';
    u_viewpos_desc.name[4] = 'e';
    u_viewpos_desc.name[5] = 'w';
    u_viewpos_desc.name[6] = 'p';
    u_viewpos_desc.name[7] = 'o';
    u_viewpos_desc.name[8] = 's';

    u_viewpos_desc.layout = u_viewpos_layout_desc;

    u_viewpos = gs_graphics_uniform_create (&u_viewpos_desc);

    // light uniform
    // Method for batch constructing uniforms within structures (that way you can upload data as a unit, rather than individuals)
    gs_graphics_uniform_layout_desc_t u_light_uniform_layout_desc[7] = {};
    u_light_uniform_layout_desc[0] = {GS_GRAPHICS_UNIFORM_VEC3, ".position", 0};
    u_light_uniform_layout_desc[1] = {GS_GRAPHICS_UNIFORM_VEC3, ".ambient", 0};
    u_light_uniform_layout_desc[2] = {GS_GRAPHICS_UNIFORM_VEC3, ".diffuse", 0};
    u_light_uniform_layout_desc[3] = {GS_GRAPHICS_UNIFORM_VEC3, ".specular", 0};
    u_light_uniform_layout_desc[4] = {GS_GRAPHICS_UNIFORM_FLOAT, ".constant", 0};
    u_light_uniform_layout_desc[5] = {GS_GRAPHICS_UNIFORM_FLOAT, ".linear", 0};
    u_light_uniform_layout_desc[6] = {GS_GRAPHICS_UNIFORM_FLOAT, ".quadratic", 0};

    gs_graphics_uniform_desc_t u_light_uniform_desc = {};
    u_light_uniform_desc.name[0] = 'u';
    u_light_uniform_desc.name[1] = '_';
    u_light_uniform_desc.name[2] = 'l';
    u_light_uniform_desc.name[3] = 'i';
    u_light_uniform_desc.name[4] = 'g';
    u_light_uniform_desc.name[5] = 'h';
    u_light_uniform_desc.name[6] = 't';

    u_light_uniform_desc.layout = u_light_uniform_layout_desc;
    u_light_uniform_desc.layout_size = 7 * sizeof(gs_graphics_uniform_layout_desc_t);

    u_light = gs_graphics_uniform_create(&u_light_uniform_desc);

    // material uniform
    gs_graphics_uniform_layout_desc_t u_material_uniform_layout_desc[3] = {};

    u_material_uniform_layout_desc[0] = {GS_GRAPHICS_UNIFORM_SAMPLER2D, ".diffuse", 0};
    u_material_uniform_layout_desc[1] = {GS_GRAPHICS_UNIFORM_SAMPLER2D, ".specular", 0};
    u_material_uniform_layout_desc[2] = {GS_GRAPHICS_UNIFORM_FLOAT, ".shininess", 0};

    gs_graphics_uniform_desc_t u_material_uniform_desc = {};
    u_material_uniform_desc.name[0] = 'u';
    u_material_uniform_desc.name[1] = '_';
    u_material_uniform_desc.name[2] = 'm';
    u_material_uniform_desc.name[3] = 'a';
    u_material_uniform_desc.name[4] = 't';
    u_material_uniform_desc.name[5] = 'e';
    u_material_uniform_desc.name[6] = 'r';
    u_material_uniform_desc.name[7] = 'i';
    u_material_uniform_desc.name[8] = 'a';
    u_material_uniform_desc.name[9] = 'l';

    u_material_uniform_desc.layout_size = 3 * sizeof(gs_graphics_uniform_layout_desc_t);
    u_material_uniform_desc.layout = u_material_uniform_layout_desc;

    u_material = gs_graphics_uniform_create(&u_material_uniform_desc);

    // shader setup    
    gs_graphics_shader_source_desc_t srcs[2] = {
        {
            GS_GRAPHICS_SHADER_STAGE_VERTEX,
            &v_src[0]
        },
        {
            GS_GRAPHICS_SHADER_STAGE_FRAGMENT,
            &f_src[0]
        }
    };

    gs_graphics_shader_desc_t shader_description = {
        srcs,
        sizeof(srcs),
        "light_shader"
    };
    
    shader = gs_graphics_shader_create(&shader_description);

    // pipeline creation
    gs_graphics_pipeline_desc_t graphics_description = {};
    graphics_description.raster.shader = shader;

    graphics_description.blend.func = GS_GRAPHICS_BLEND_EQUATION_ADD;
    graphics_description.blend.src = GS_GRAPHICS_BLEND_MODE_SRC_ALPHA;
    graphics_description.blend.dst = GS_GRAPHICS_BLEND_MODE_ONE_MINUS_SRC_ALPHA;

    graphics_description.depth.func = GS_GRAPHICS_DEPTH_FUNC_LESS;

    gs_graphics_vertex_attribute_desc_t vertex_attrs[3] = {};
    vertex_attrs[0].format = GS_GRAPHICS_VERTEX_ATTRIBUTE_FLOAT3; // Position
    vertex_attrs[1].format = GS_GRAPHICS_VERTEX_ATTRIBUTE_FLOAT3; // Normal
    vertex_attrs[2].format = GS_GRAPHICS_VERTEX_ATTRIBUTE_FLOAT2; // TexCoord
    
    graphics_description.layout.attrs = vertex_attrs;
    graphics_description.layout.size = 3 * sizeof(gs_graphics_vertex_attribute_desc_t);

    pip = gs_graphics_pipeline_create (&graphics_description);
}

void UpdateScene(AppState *appState, gs_camera_t cam){
    
    gs_vec2 fs = gs_platform_framebuffer_sizev(gs_platform_main_window());
    gs_vec2 ws = gs_platform_window_sizev(gs_platform_main_window());
    const float t = gs_platform_elapsed_time() * 0.001f;
    const float rad = sin(t * 0.2f) * 10.f; 

    // Construct proj/view matrices
    gs_mat4 view = gs_camera_get_view(&cam);  
    gs_mat4 proj = gs_camera_get_proj(&cam, (int32_t)ws.x, (int32_t)ws.y);

    // Construct light data to submit
    light_params_t light = {
        .position = gs_v3(sin(t) * rad, 1.f, cos(t) * rad),
        .ambient = gs_v3(1.0f, 1.0f, 1.0f),
        .diffuse = gs_v3(1.0f, 0.5f, 0.5f),
        .specular = gs_v3(1.f, 1.f, 1.f),
        .constant = 1.f,
        .linear = 0.09f,
        .quadratic = 0.032f
    };

    // Construct material data to submit
    material_params_t mat = {
        .diffuse = t_diffuse,
        .specular = t_specular,
        .shininess = 100.f
    };

    gs_graphics_bind_vertex_buffer_desc_t vbo_desc = {};
    vbo_desc.buffer = vbo;

    // Uniform bindings that don't change per object
    gs_graphics_bind_uniform_desc_t uniforms[5] = {};
    uniforms[0] = {u_proj, &proj, 0};
    uniforms[1] = {u_view, &view, 0};
    uniforms[2] = {u_light, &light, 0};
    uniforms[3] = {u_viewpos, &cam.transform.position, 0};
    uniforms[4] = {u_material, &mat, 0};

    binds = {
        .vertex_buffers = {.desc = &vbo_desc},
        .uniforms = {.desc = uniforms, .size = sizeof(uniforms)}
    };

    gs_graphics_clear_action_t clear_action_attrs = {};
    clear_action_attrs.color[0] = 0.1f;
    clear_action_attrs.color[1] = 0.1f;
    clear_action_attrs.color[2] = 0.1f;
    clear_action_attrs.color[3] = 1.0f; 

    gs_graphics_clear_desc_t clear = {};
    clear.actions = &clear_action_attrs;
    
    gs_graphics_renderpass_begin(&commandBuffer, GS_GRAPHICS_RENDER_PASS_DEFAULT); 
    
    {
        gs_graphics_pipeline_bind(&commandBuffer, pip);
        gs_graphics_set_viewport(&commandBuffer, 0, 0, (uint32_t)fs.x, (uint32_t)fs.y);
        gs_graphics_clear(&commandBuffer, &clear);
        gs_graphics_apply_bindings(&commandBuffer, &binds);

        gs_vec3 test_cube_pos = {0.0f, 0.0f, 0.0f};

        int cubesCount = 3;
        for(int i = 0; i < appState->rectPrisms.size(); i++){

            RectPrism prism = appState->rectPrisms.at(i);

            gs_mat4 model = gs_mat4_translate(prism.position.x, prism.position.y, prism.position.z);
            
            model = gs_mat4_mul(model, gs_mat4_scale(prism.scale.x, prism.scale.y, prism.scale.z));

            gs_graphics_bind_uniform_desc_t uniform_bind_desc = {};
            uniform_bind_desc.uniform = u_model;
            uniform_bind_desc.data = &model;

            gs_graphics_bind_desc_t model_binds = {};
            model_binds.uniforms.desc = &uniform_bind_desc;
            
            gs_graphics_apply_bindings(&commandBuffer, &model_binds);
            gs_graphics_draw_desc_t draw_desc = {};
            draw_desc.start = 0;
            draw_desc.count = 36;
            gs_graphics_draw(&commandBuffer, &draw_desc);
        }
    }

    gs_graphics_renderpass_end(&commandBuffer);

    // Submit command buffer (syncs to GPU, MUST be done on main thread where you have your GPU context created)
    gs_graphics_command_buffer_submit(&commandBuffer);
}


void DrawRectPrism(AppState *appState,  gs_vec3 position, gs_vec3 scale, gs_vec4 color){
    appState->rectPrisms.push_back({position, scale, color});
}