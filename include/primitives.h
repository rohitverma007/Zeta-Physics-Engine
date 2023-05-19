#ifndef PRIMITIVES_H
#define PRIMITIVES_H

#include "zmath.h"

// todo add masses for each of the primitives for their rigidbodies

namespace Primitives {
    class Ray3D {
        public:
            ZMath::Vec3D origin;
            ZMath::Vec3D dir; // normalized direction of the ray

            // @brief Construct a new Ray3D object
            // 
            // @param position The origin of the ray.
            // @param direction The direction of the ray as a normalized vector.
            Ray3D(ZMath::Vec3D origin, ZMath::Vec3D direction) : origin(origin), dir(direction) {};
    };

    class Line3D {
        public:
            ZMath::Vec3D start, end;

            // @brief Create a line between two points.
            //
            // @param p1 (Vec3D) Starting point.
            // @param p2 (Vec3D) Ending point.
            Line3D(ZMath::Vec3D const &p1, ZMath::Vec3D const &p2) : start(p1), end(p2) {};

            // A vector with the lowest value of x, y, and z the line segment reaches.
            ZMath::Vec3D getMin() const { return ZMath::Vec3D(ZMath::min(start.x, end.x), ZMath::min(start.y, end.y), ZMath::min(start.z, end.z)); };

            // A vector with greatest value of x, y, and z the line segment reaches.
            ZMath::Vec3D getMax() const { return ZMath::Vec3D(ZMath::max(start.x, end.x), ZMath::max(start.y, end.y), ZMath::max(start.z, end.z)); };
    };

    // Models a rectangular, finite plane in 3D space.
    // Planes should only be used as colliders for static bodies.
    // This should be used to model death planes, borders, etc. as planes are not affected by forces and impulse.
    class Plane {
        private:
            ZMath::Vec2D halfSize;

        public:
            ZMath::Vec3D pos; // Center of the plane.
            ZMath::Vec3D normal; // Normal vector to the plane in global coordinates. Cached for efficiency.
            ZMath::Mat3D rot; // Rotate anything from global space to this plane's local space. Cache this value for efficiency.

            float theta; // Angle rotated with respect to the XY plane.
            float phi; // Angle rotated with respect to the XZ plane.

            /**
             * @brief Create an unrotated plane.
             * 
             * @param min (Vec2D) The min vertex of the plane.
             * @param max (Vec2D) The max vertex of the plane.
             * @param z (float) The z-level the plane lies on.
             */
            Plane(ZMath::Vec2D const &min, ZMath::Vec2D const &max, float z) : halfSize((max - min) * 0.5f),
                    pos(ZMath::Vec3D(min.x + halfSize.x, min.y + halfSize.y, z)), theta(0.0f), phi(0.0f) {
                
                ZMath::Vec3D v1 = ZMath::Vec3D(pos.x - halfSize.x, pos.y - halfSize.y, pos.z);
                ZMath::Vec3D v2 = ZMath::Vec3D(pos.x + halfSize.x, pos.y - halfSize.y, pos.z);

                normal = (v2 - pos).cross(v1 - pos);
                rot = ZMath::Mat3D::identity(); // no rotation
            };

            /**
             * @brief Create a rotated plane.
             * 
             * @param min (Vec2D) The min vertex of the plane as if it was unrotated.
             * @param max (Vec2D) The max vertex of the plane as if it was unrotated.
             * @param z (float) The z-level the plane lies on.
             * @param angXY (float) The angle, in degrees, the plane is rotated with respect to the XY plane.
             * @param angXZ (float) The angle, in degrees, the plane is rotated with respect to the XZ plane.
             */
            Plane(ZMath::Vec2D const &min, ZMath::Vec2D const &max, float z, float angXY, float angXZ) : halfSize((max - min) * 0.5f),
                    pos(ZMath::Vec3D(min.x + halfSize.x, min.y + halfSize.y, z)), theta(angXY), phi(angXZ) {

                ZMath::Vec3D v1 = ZMath::Vec3D(pos.x - halfSize.x, pos.y - halfSize.y, pos.z);
                ZMath::Vec3D v2 = ZMath::Vec3D(pos.x + halfSize.x, pos.y - halfSize.y, pos.z);

                rot = ZMath::Mat3D::generateRotationMatrix(angXY, angXZ);

                // rotate the points to find the normal
                // // ZMath::Mat3D rotT = rot.transpose();

                v1 = rot * v1;
                v2 = rot * v2;

                normal = (v2 - pos).cross(v1 - pos);
            };

            ZMath::Vec2D getLocalMin() const { return ZMath::Vec2D(pos.x - halfSize.x, pos.y - halfSize.y); };
            ZMath::Vec2D getLocalMax() const { return ZMath::Vec2D(pos.x + halfSize.x, pos.y + halfSize.y); };
            ZMath::Vec2D getHalfSize() const { return halfSize; };

            // Get the vertices of the plane in terms of global coordinates.
            // Remember to use delete[] on the object you assign this to afterwards to free the memory.
            ZMath::Vec3D* getVertices() const {
                ZMath::Vec3D* v = new ZMath::Vec3D[4]; // 4 as it is rectangular

                v[0] = ZMath::Vec3D(-halfSize.x, -halfSize.y, pos.z);
                v[1] = ZMath::Vec3D(-halfSize.x, halfSize.y, 0);
                v[2] = ZMath::Vec3D(halfSize.x, halfSize.y, 0);
                v[3] = ZMath::Vec3D(halfSize.x, -halfSize.y, 0);

                // rotate each vertex
                for (int i = 0; i < 4; i++) { v[i] = rot*v[i] + pos; }

                return v;
            };
    };

    class Sphere {
        public:
            float r; // radius
            ZMath::Vec3D c; // centerpoint

            // @brief Create a Sphere with an arbitrary radius and center.
            //
            // @param rho (float) Radius of the sphere.
            // @param center (Vec3D) Center of the sphere.
            Sphere(float rho, ZMath::Vec3D const &center) : r(rho), c(center) {};
    };

    class AABB {
        private:
            ZMath::Vec3D halfSize;

        public:
            ZMath::Vec3D pos; // Centerpoint of the AABB.

            /** 
             * @brief Instantiate a 3D unrotated Cube.
             * 
             * @param min (Vec3D) Min vertex of the AABB.
             * @param max (Vec3D) Max vertex of the AABB.
             */
            AABB(ZMath::Vec3D const &min, ZMath::Vec3D const &max) : halfSize((max - min) * 0.5f) {
                rb.pos = min + halfSize;
                rb.theta = 0.0f;
                rb.phi = 0.0f;
            };

            ZMath::Vec3D getMin() const { return rb.pos - halfSize; };
            ZMath::Vec3D getMax() const { return rb.pos + halfSize; };
            ZMath::Vec3D getHalfSize() const { return halfSize; };

            ZMath::Vec3D* getVertices() const{
                ZMath::Vec3D* vertices = new ZMath::Vec3D[8];

                vertices[0] = {rb.pos.x - halfSize.x, rb.pos.y - halfSize.y, rb.pos.z + halfSize.z}; // bottom left
                vertices[1] = {rb.pos.x - halfSize.x, rb.pos.y + halfSize.y, rb.pos.z + halfSize.z}; // top left
                vertices[2] = {rb.pos.x + halfSize.x, rb.pos.y + halfSize.y, rb.pos.z + halfSize.z}; // top right
                vertices[3] = {rb.pos.x + halfSize.x, rb.pos.y - halfSize.y, rb.pos.z + halfSize.z};  // bottom right
                
                vertices[4] = {rb.pos.x - halfSize.x, rb.pos.y - halfSize.y, rb.pos.z - halfSize.z}; // bottom left
                vertices[5] = {rb.pos.x - halfSize.x, rb.pos.y + halfSize.y, rb.pos.z - halfSize.z}; // top left
                vertices[6] = {rb.pos.x + halfSize.x, rb.pos.y + halfSize.y, rb.pos.z - halfSize.z}; // top right
                vertices[7] = {rb.pos.x + halfSize.x, rb.pos.y - halfSize.y, rb.pos.z - halfSize.z}; // bottom right

                return vertices;
            };
    };

    class Cube {
        private:
            ZMath::Vec3D halfSize;

        public:
            ZMath::Vec3D pos; // Centerpoint of the Cube.
            ZMath::Mat3D rot; // Rotate anything from global space to this cube's local space. Cache this value for efficiency.

            // @brief Create a cube rotated by an arbitrary angle with arbitrary min and max vertices.
            //
            // @param min Min vertex of the cube as if it was not rotated.
            // @param max Max vertex of the cube as if it was not rotated.
            // @param angXY Angle the cube is rotated by with respect to the XY plane in degrees.
            // @param angXZ Angle the cube is rotated by with respect to the XZ plane in degrees.
            Cube(ZMath::Vec3D const &min, ZMath::Vec3D const &max, float angXY, float angXZ) : halfSize((max - min) * 0.5f) {
                rb.pos = min + halfSize;
                rb.theta = angXY;
                rb.phi = angXZ;
                rb.mass = 20.0f;
                rb.invMass = 0.05f;

                rot = ZMath::Mat3D::generateRotationMatrix(angXY, angXZ);
            };

            // Get the min vertex in the cube's UVW coordinates.
            ZMath::Vec3D getLocalMin() const { return rb.pos - halfSize; };

            // Get the max vertex in the cube's UVW coordinates.
            ZMath::Vec3D getLocalMax() const { return rb.pos + halfSize; };

            // Get half the distance between the cube's min and max vertices.
            ZMath::Vec3D getHalfSize() const { return halfSize; };

            // Get the vertices of the cube in terms of global coordinates.
            // Remeber to use delete[] on the variable you assign this after use to free the memory.
            ZMath::Vec3D* getVertices() const {
                ZMath::Vec3D* v = new ZMath::Vec3D[8];

                // todo maybe reorder
                // ! p sure this is a bad order for them. Will fix sometime after our presentation
                v[0] = rb.pos - halfSize;
                v[1] = ZMath::Vec3D(rb.pos.x - halfSize.x, rb.pos.y - halfSize.y, rb.pos.z +  halfSize.z);
                v[2] = ZMath::Vec3D(rb.pos.x + halfSize.x, rb.pos.y - halfSize.y, rb.pos.z + halfSize.z);
                v[3] = ZMath::Vec3D(rb.pos.x + halfSize.x, rb.pos.y - halfSize.y, rb.pos.z - halfSize.z);
                v[4] = ZMath::Vec3D(rb.pos.x - halfSize.x, rb.pos.y + halfSize.y, rb.pos.z + halfSize.z);
                v[5] = ZMath::Vec3D(rb.pos.x - halfSize.x, rb.pos.y + halfSize.y, rb.pos.z - halfSize.z);
                v[6] = ZMath::Vec3D(rb.pos.x + halfSize.x, rb.pos.y + halfSize.y, rb.pos.z - halfSize.z);
                v[7] = rb.pos + halfSize;

                for (int i = 0; i < 8; i++) {
                    ZMath::rotateXY(v[i], rb.pos, rb.theta);
                    ZMath::rotateXZ(v[i], rb.pos, rb.phi);
                }

                return v;
            }; 
    };
} // namespace Primitives

#endif // !PRIMITIVES_H
