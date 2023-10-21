#pragma once

#include <utility>
#include "primitives.h"

namespace Zeta {
    enum RigidBodyCollider {
        RIGID_SPHERE_COLLIDER,
        RIGID_AABB_COLLIDER,
        RIGID_CUBE_COLLIDER,
        RIGID_CUSTOM_COLLIDER,
        RIGID_NONE
    };

    enum StaticBodyCollider {
        STATIC_PLANE_COLLIDER,
        STATIC_SPHERE_COLLIDER,
        STATIC_AABB_COLLIDER,
        STATIC_CUBE_COLLIDER,
        STATIC_CUSTOM_COLLIDER,
        STATIC_NONE
    };

    class RigidBody3D {
        public:
            RigidBody3D() : angle(0.0f), rotationalVelocity(0.0f), rotationalInertia(0.0f) {};

            RigidBody3D(ZMath::Vec3D const &pos, float mass, float cor, float linearDamping, RigidBodyCollider colliderType, void* collider, float angle, float rotationalVelocity, float rotationalInertia) 
                    : pos(pos), mass(mass), invMass(1.0f/mass), cor(cor), linearDamping(linearDamping), 
                      colliderType(colliderType), collider(collider), angle(angle), rotationalVelocity(rotationalVelocity), rotationalInertia(rotationalInertia) {};

            RigidBody3D(RigidBody3D const &rb) {
                pos = rb.pos;
                mass = rb.mass;
                invMass = rb.invMass;
                cor = rb.cor;
                linearDamping = rb.linearDamping;
                colliderType = rb.colliderType;
                angle = rb.angle;
                rotationalVelocity = rb.rotationalVelocity;
                rotationalInertia = rb.rotationalInertia;

                switch(colliderType) {
                    case RIGID_SPHERE_COLLIDER: { collider = new Sphere(*((Sphere*) rb.collider)); break; }
                    case RIGID_AABB_COLLIDER:   { collider = new AABB(*((AABB*) rb.collider));     break; }
                    case RIGID_CUBE_COLLIDER:   { collider = new Cube(*((Cube*) rb.collider));     break; }
                    case RIGID_NONE:            { collider = nullptr;                              break; }
                }
            };

            RigidBody3D(RigidBody3D &&rb) {
                pos = std::move(rb.pos);
                mass = rb.mass;
                invMass = rb.invMass;
                cor = rb.cor;
                linearDamping = rb.linearDamping;
                colliderType = rb.colliderType;
                collider = rb.collider;
                rb.collider = nullptr;
                angle = rb.angle;
                rotationalVelocity = rb.rotationalVelocity;
                rotationalInertia = rb.rotationalInertia;
            };

            RigidBody3D& operator = (RigidBody3D const &rb) {
                if (collider) {
                    switch(colliderType) {
                        case RIGID_SPHERE_COLLIDER: { delete (Sphere*) collider; break; }
                        case RIGID_AABB_COLLIDER:   { delete (AABB*) collider;   break; }
                        case RIGID_CUBE_COLLIDER:   { delete (Cube*) collider;   break; }
                    }
                }

                pos = rb.pos;
                mass = rb.mass;
                invMass = rb.invMass;
                cor = rb.cor;
                linearDamping = rb.linearDamping;
                colliderType = rb.colliderType;
                angle = rb.angle;
                rotationalVelocity = rb.rotationalVelocity;
                rotationalInertia = rb.rotationalInertia;

                switch(colliderType) {
                    case RIGID_SPHERE_COLLIDER: { collider = new Sphere(*((Sphere*) rb.collider)); break; }
                    case RIGID_AABB_COLLIDER:   { collider = new AABB(*((AABB*) rb.collider));     break; }
                    case RIGID_CUBE_COLLIDER:   { collider = new Cube(*((Cube*) rb.collider));     break; }
                    case RIGID_NONE:            { collider = nullptr;                              break; }
                }

                return *this;
            };

            RigidBody3D& operator = (RigidBody3D &&rb) {
                if (this != &rb) {
                    pos = std::move(rb.pos);
                    mass = rb.mass;
                    invMass = rb.invMass;
                    cor = rb.cor;
                    linearDamping = rb.linearDamping;
                    colliderType = rb.colliderType;
                    collider = rb.collider;
                    rb.collider = nullptr;
                    angle = rb.angle;
                    rotationalVelocity = rb.rotationalVelocity;
                    rotationalInertia = rb.rotationalInertia;
                }

                return *this;
            };

            ~RigidBody3D() {
                switch(colliderType) {
                    case RIGID_SPHERE_COLLIDER: { delete (Sphere*) collider; break; }
                    case RIGID_AABB_COLLIDER:   { delete (AABB*) collider;   break; }
                    case RIGID_CUBE_COLLIDER:   { delete (Cube*) collider;   break; }
                }
            };

            RigidBodyCollider colliderType;
            void* collider;

            float mass;
            float invMass;
            float cor;
            float linearDamping;

            ZMath::Vec3D pos;
            ZMath::Vec3D vel = ZMath::Vec3D();
            ZMath::Vec3D netForce = ZMath::Vec3D();

            float angle;
            float rotationalVelocity;
            float rotationalInertia;

            void update(ZMath::Vec3D const &g, float dt) {
                netForce += g * mass;
                vel += (netForce * invMass) * dt;
                pos += vel * dt;

                vel *= linearDamping;
                netForce = ZMath::Vec3D();

                switch(colliderType) {
                    case RIGID_SPHERE_COLLIDER: { ((Sphere*) collider)->c = pos; break; }
                    case RIGID_AABB_COLLIDER:   { ((AABB*) collider)->pos = pos; break; }
                    case RIGID_CUBE_COLLIDER:   { ((Cube*) collider)->pos = pos; break; }
                }
            };
    };

    class StaticBody3D {
        public:
            StaticBody3D() {};

            StaticBody3D(ZMath::Vec3D const &pos, StaticBodyCollider colliderType, void* collider)
                    : pos(pos), colliderType(colliderType), collider(collider) {};

            StaticBody3D(StaticBody3D const &sb) {
                pos = sb.pos;
                colliderType = sb.colliderType;

                switch(colliderType) {
                    case STATIC_PLANE_COLLIDER:  { collider = new Plane(*((Plane*) sb.collider));   break; }
                    case STATIC_SPHERE_COLLIDER: { collider = new Sphere(*((Sphere*) sb.collider)); break; }
                    case STATIC_AABB_COLLIDER:   { collider = new AABB(*((AABB*) sb.collider));     break; }
                    case STATIC_CUBE_COLLIDER:   { collider = new Cube(*((Cube*) sb.collider));     break; }
                    case STATIC_NONE:            { collider = nullptr;                              break; }
                }
            };

            StaticBody3D(StaticBody3D &&sb) {
                pos = std::move(sb.pos);
                colliderType = sb.colliderType;
                collider = sb.collider;
                sb.collider = nullptr;
            };

            StaticBody3D& operator = (StaticBody3D const &sb) {
                if (collider) {
                    switch(colliderType) {
                        case STATIC_PLANE_COLLIDER:  { delete (Plane*) collider;  break; }
                        case STATIC_SPHERE_COLLIDER: { delete (Sphere*) collider; break; }
                        case STATIC_AABB_COLLIDER:   { delete (AABB*) collider;   break; }
                        case STATIC_CUBE_COLLIDER:   { delete (Cube*) collider;   break; }
                    }
                }

                pos = sb.pos;
                colliderType = sb.colliderType;

                switch(colliderType) {
                    case STATIC_PLANE_COLLIDER:  { collider = new Plane(*((Plane*) sb.collider));   break; }
                    case STATIC_SPHERE_COLLIDER: { collider = new Sphere(*((Sphere*) sb.collider)); break; }
                    case STATIC_AABB_COLLIDER:   { collider = new AABB(*((AABB*) sb.collider));     break; }
                    case STATIC_CUBE_COLLIDER:   { collider = new Cube(*((Cube*) sb.collider));     break; }
                    case STATIC_NONE:            { collider = nullptr;                              break; }
                }

                return *this;
            };

            StaticBody3D& operator = (StaticBody3D &&sb) {
                if (this != &sb) {
                    pos = std::move(sb.pos);
                    colliderType = sb.colliderType;
                    collider = sb.collider;
                    sb.collider = nullptr;
                }

                return *this;
            };

            ~StaticBody3D() {
                switch(colliderType) {
                    case STATIC_PLANE_COLLIDER:  { delete (Plane*) collider;  break; }
                    case STATIC_SPHERE_COLLIDER: { delete (Sphere*) collider; break; }
                    case STATIC_AABB_COLLIDER:   { delete (AABB*) collider;   break; }
                    case STATIC_CUBE_COLLIDER:   { delete (Cube*) collider;   break; }
                }
            };

            ZMath::Vec3D pos;

            StaticBodyCollider colliderType;
            void* collider;
        };
}