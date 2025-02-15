#include "entity.h"

namespace E_{    
        
    static void AddVertexPosition(VertexData *vertex_data, float x, float y, float z){
        vertex_data->positions[vertex_data->index] = x; 
        vertex_data->positions[vertex_data->index + 1] = y;
        vertex_data->positions[vertex_data->index + 2] = z;
        vertex_data->len_positions += 3;
        vertex_data->index += 3;
    }

    static void AddVertexNormal(VertexData *vertex_data, float x, float y, float z){
        vertex_data->normals[vertex_data->index] = x;
        vertex_data->normals[vertex_data->index + 1] = y;
        vertex_data->normals[vertex_data->index + 2] = z;
        vertex_data->len_normals += 3;
        vertex_data->index += 3;
    }

    static void AddVertexIndice(VertexData *vertex_data, int x, float y, float z){
        
        vertex_data->indices[vertex_data->index] = x;
        vertex_data->indices[vertex_data->index  + 1] = y;
        vertex_data->indices[vertex_data->index  + 2] = z;
        vertex_data->len_indices += 3;
        vertex_data->index += 3;
    }
    
    static void zeta_verts_to_eq(ZMath::Vec3D *zeta_verts, VertexData *vertex_data){
        // TOP
        AddVertexPosition(vertex_data, zeta_verts[4].x, zeta_verts[4].y, zeta_verts[4].z); // front top left
        AddVertexPosition(vertex_data, zeta_verts[7].x, zeta_verts[7].y, zeta_verts[7].z); // front top right
        AddVertexPosition(vertex_data, zeta_verts[6].x, zeta_verts[6].y, zeta_verts[6].z); // back top right
        AddVertexPosition(vertex_data, zeta_verts[5].x, zeta_verts[5].y, zeta_verts[5].z); // back top left

        // BOTTOM
        AddVertexPosition(vertex_data, zeta_verts[0].x, zeta_verts[0].y, zeta_verts[0].z); // back  bottom left
        AddVertexPosition(vertex_data, zeta_verts[3].x, zeta_verts[3].y, zeta_verts[3].z); // back bottom right
        AddVertexPosition(vertex_data, zeta_verts[2].x, zeta_verts[2].y, zeta_verts[2].z); // front bottom right
        AddVertexPosition(vertex_data, zeta_verts[1].x, zeta_verts[1].y, zeta_verts[1].z); // front bottom left

        // FRONT
        AddVertexPosition(vertex_data, zeta_verts[1].x, zeta_verts[1].y, zeta_verts[1].z); // front bottom left
        AddVertexPosition(vertex_data, zeta_verts[2].x, zeta_verts[2].y, zeta_verts[2].z); // front bottom right
        AddVertexPosition(vertex_data, zeta_verts[7].x, zeta_verts[7].y, zeta_verts[7].z); // front top right
        AddVertexPosition(vertex_data, zeta_verts[4].x, zeta_verts[4].y, zeta_verts[4].z); // front top left

        // BACK
        AddVertexPosition(vertex_data, zeta_verts[3].x, zeta_verts[3].y, zeta_verts[3].z); // back bottom right
        AddVertexPosition(vertex_data, zeta_verts[0].x, zeta_verts[0].y, zeta_verts[0].z); // back bottom left
        AddVertexPosition(vertex_data, zeta_verts[5].x, zeta_verts[5].y, zeta_verts[5].z); // back top left
        AddVertexPosition(vertex_data, zeta_verts[6].x, zeta_verts[6].y, zeta_verts[6].z); // back top right

        // RIGHT
        AddVertexPosition(vertex_data, zeta_verts[3].x, zeta_verts[3].y, zeta_verts[3].z); // back bottom right
        AddVertexPosition(vertex_data, zeta_verts[6].x, zeta_verts[6].y, zeta_verts[6].z); // back top right
        AddVertexPosition(vertex_data, zeta_verts[7].x, zeta_verts[7].y, zeta_verts[7].z); // front top right
        AddVertexPosition(vertex_data, zeta_verts[2].x, zeta_verts[2].y, zeta_verts[2].z); // front bottom right

        // LEFT
        AddVertexPosition(vertex_data, zeta_verts[0].x, zeta_verts[0].y, zeta_verts[0].z); // back bottom left
        AddVertexPosition(vertex_data, zeta_verts[1].x, zeta_verts[1].y, zeta_verts[1].z); // front bottom left
        AddVertexPosition(vertex_data, zeta_verts[4].x, zeta_verts[4].y, zeta_verts[4].z); // front top left
        AddVertexPosition(vertex_data, zeta_verts[5].x, zeta_verts[5].y, zeta_verts[5].z); // back top left

        vertex_data->index = 0;
        // TOP
        AddVertexIndice(vertex_data, 0, 1, 2);
        AddVertexIndice(vertex_data, 2, 3, 0);

        // BOTTOM
        AddVertexIndice(vertex_data, 4, 5, 6);
        AddVertexIndice(vertex_data, 6, 7, 4);

        // FRONT
        AddVertexIndice(vertex_data, 8, 9, 10);
        AddVertexIndice(vertex_data, 10, 11, 8);

        // BACK
        AddVertexIndice(vertex_data, 12, 13, 14);
        AddVertexIndice(vertex_data, 14, 15, 12);

        // RIGHT
        AddVertexIndice(vertex_data, 16, 17, 18);
        AddVertexIndice(vertex_data, 18, 19, 16);

        // LEFT
        AddVertexIndice(vertex_data, 20, 21, 22);
        AddVertexIndice(vertex_data, 22, 23, 20);

        // NOTE(Lenny): Calculations may not work for other shapes besides prisms :(
        // vertex_data->index = 0;
        float *positions = vertex_data->positions;
        vertex_data->len_normals = vertex_data->len_positions;
    
        for (int i = 0; i < vertex_data->len_indices; i += 3) {
            int index_1 = vertex_data->indices[i];
            int index_2 = vertex_data->indices[i + 1];
            int index_3 = vertex_data->indices[i + 2];

            HMM_Vec3 pos_1 = {positions[index_1 * 3], positions[index_1 * 3 + 1], positions[index_1 * 3 + 2]};
            HMM_Vec3 pos_2 = {positions[index_2 * 3], positions[index_2 * 3 + 1], positions[index_2 * 3 + 2]};
            HMM_Vec3 pos_3 = {positions[index_3 * 3], positions[index_3 * 3 + 1], positions[index_3 * 3 + 2]};

            HMM_Vec3 normal = HMM_Cross(pos_2 - pos_1, pos_3 - pos_1);
            normal = HMM_NormV3(normal);

            vertex_data->normals[index_1 * 3] += normal.X;
            vertex_data->normals[index_1 * 3 + 1] += normal.Y;
            vertex_data->normals[index_1 * 3 + 2] += normal.Z;
            
            
            vertex_data->normals[index_2 * 3] += normal.X;
            vertex_data->normals[index_2 * 3 + 1] += normal.Y;
            vertex_data->normals[index_2 * 3  + 2] += normal.Z;
            
            
            vertex_data->normals[index_3 * 3] += normal.X;
            vertex_data->normals[index_3 * 3 + 1] += normal.Y;
            vertex_data->normals[index_3 * 3 + 2] += normal.Z;

            int x = 0;
        }

        vertex_data->index = 0;
    }

    void add_texture(Entity *entity, unsigned int texture){
        entity->textures[entity->textureIndex++] = texture;
    }

    Entity *create_entity(EntityManager *em, HMM_Vec3 position, float scale, 
        float rotation_x, float rotation_y, float rotation_z, Zeta::RigidBodyCollider colliderType, void *collider){
        Entity *result = &em->entities[em->index++];

        result->scale = scale;
        result->rotation_x = rotation_x;
        result->rotation_y = rotation_y;
        result->rotation_z = rotation_z;

        result->rb = new Zeta::RigidBody3D(
            {position.X, position.Y, position.Z}, 
            100.0f, 0.1f, 1.0f, colliderType, collider);

        return result;
    }

    Entity *create_entity(EntityManager *em, HMM_Vec3 position, float scale, 
        float rotation_x, float rotation_y, float rotation_z,  Zeta::StaticBodyCollider colliderType, void *collider){
        Entity *result = &em->entities[em->index++];
        result->scale = scale;
        result->rotation_x = rotation_x;
        result->rotation_y = rotation_y;
        result->rotation_z = rotation_z;

        result->sb = new Zeta::StaticBody3D(
        {position.X, position.Y, position.Z}, 
        colliderType, collider);

        return result;   
    }

    // call after `add_collider()`
    void init(Entity *entity){
                
        std::vector<float> tex_coords = {
            0.0 , 0.0, 0.0, 
            0.0 , 1.0, 0.0, 
            1.0 , 1.0, 0.0, 
            1.0 , 0.0, 0.0, 			
            0.0 , 0.0, 0.0, 
            0.0 , 1.0, 0.0, 
            1.0 , 1.0, 0.0, 
            1.0 , 0.0, 0.0, 			
            0.0 , 0.0, 0.0, 
            0.0 , 1.0, 0.0, 
            1.0 , 1.0, 0.0, 
            1.0 , 0.0, 0.0, 
            0.0 , 0.0, 0.0, 
            0.0 , 1.0, 0.0, 
            1.0 , 1.0, 0.0, 
            1.0 , 0.0, 0.0, 
            0.0 , 0.0, 0.0, 
            0.0 , 1.0, 0.0, 
            1.0 , 1.0, 0.0, 
            1.0 , 0.0, 0.0, 
            0.0 , 0.0, 0.0, 
            0.0 , 1.0, 0.0, 
            1.0 , 1.0, 0.0, 
            1.0 , 0.0, 0.0
        };

        HMM_Vec4 color = {1.0f, 1.0f, 1.0f, 1.0f};
        std::vector<float> cube_colors = {
            // TOP
            color.X, color.Y, color.Z,
            color.X, color.Y, color.Z,
            color.X, color.Y, color.Z,
            color.X, color.Y, color.Z,

            // TOP
            color.X, color.Y, color.Z,
            color.X, color.Y, color.Z,
            color.X, color.Y, color.Z,
            color.X, color.Y, color.Z,
            
            // TOP
            color.X, color.Y, color.Z,
            color.X, color.Y, color.Z,
            color.X, color.Y, color.Z,
            color.X, color.Y, color.Z,
            
            // TOP
            color.X, color.Y, color.Z,
            color.X, color.Y, color.Z,
            color.X, color.Y, color.Z,
            color.X, color.Y, color.Z,
            
            // TOP
            color.X, color.Y, color.Z,
            color.X, color.Y, color.Z,
            color.X, color.Y, color.Z,
            color.X, color.Y, color.Z,
            
            // TOP
            color.X, color.Y, color.Z,
            color.X, color.Y, color.Z,
            color.X, color.Y, color.Z,
            color.X, color.Y, color.Z,
        };

        VertexData vertex_data = {};
        vertex_data.positions = new float[3 * 4 * 6 * sizeof(float)]();
        vertex_data.normals = new float[3 * 4 * 6 * sizeof(float)]();
        vertex_data.colors = new float[3 * 4 * 6 * sizeof(float)]();
        vertex_data.indices = new int[3 * 2 * 6 * sizeof(int)]();

        vertex_data.tex_coords = new float[3 * 4 * 6 * sizeof(float)];
        vertex_data.len_tex_coords = 3 * 4 * 6;

        if(entity->sb){
            zeta_verts_to_eq(entity->sb->collider.cube.getVertices(), &vertex_data);
        }else if(entity->rb){
            zeta_verts_to_eq(entity->rb->collider.cube.getVertices(), &vertex_data);
        }else{
            Assert(!"No RigidBody or StaticBody Attached");
        }

        vertex_data.colors = &cube_colors[0];
        vertex_data.len_colors = vertex_data.len_positions;
        vertex_data.tex_coords = &tex_coords[0];
        vertex_data.len_tex_coords = 72;

        if(entity->sb){
            entity->sb->collider.cube.pos = entity->sb->pos;
        }else if(entity->rb){
            entity->rb->collider.cube.pos = entity->rb->pos;
        }
        
        entity->raw_model = load_to_VAO(&vertex_data);
    }

    void init(Entity *entity, RawModel model){
        entity->raw_model = model;
    }

    static void add_collider(Entity *entity, Zeta::RigidBodyCollider colliderType, void *collider){
        if(colliderType == Zeta::RigidBodyCollider::RIGID_CUBE_COLLIDER){
            Zeta::Cube *cube = (Zeta::Cube *)collider;

            // cube->pos = rb->pos;
            entity->rb->collider.cube = *cube;
        }
    }

    static void add_collider(Entity *entity, Zeta::StaticBodyCollider colliderType, void *collider){
        if(colliderType == Zeta::StaticBodyCollider::STATIC_CUBE_COLLIDER){
            Zeta::Cube *cube = (Zeta::Cube *)collider;

            entity->sb->collider.cube = *cube;
        }
    }

    static void increase_rotation(Entity *entity, float dx, float dy, float dz){
        entity->rotation_x += dx;
        entity->rotation_y += dy;
        entity->rotation_z += dz;
    }

};