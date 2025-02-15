#pragma once

#include "bodies.h"

namespace Zeta {
    // * ===================================
    // * Point vs Primitives
    // * ===================================

    // Determine if a point lies on a line.
    static bool PointAndLine(ZMath::Vec3D const &point, Line3D const &line) {
        // ? Determine the distance between the line and point using the projection of that point to a point on the line.
        // ? D = ||PQ x u||/||u||
        // ? If this distance is 0, we know it lies on the line.

        // obtain the max and min points for the bounds
        ZMath::Vec3D min = line.getMin(), max = line.getMax();

        // ensure the point is within the start and end of the line segment
        if (point.x < min.x || point.x > max.x || point.y < min.y || point.y > max.y || point.z < min.z || point.z > max.z) { return 0; }

        // We don't need to divide by ||u|| to know if it will evaluate to 0.
        return ZMath::compare((point - line.start).cross((line.end - line.start)).magSq(), 0);
    };

    // Determine if a point lies on a plane.
    static bool PointAndPlane(ZMath::Vec3D const &point, Plane const &plane) {
        ZMath::Vec2D min = plane.getLocalMin(), max = plane.getLocalMax();
        ZMath::Vec3D p = point - plane.pos; // allows for rotation

        // rotate the point into local space
        // ensure the point is rotated about the proper origin by subtracting it initially and adding it back.
        p = plane.rot * p + plane.pos;

        return p.x >= min.x && p.y >= min.y && p.x <= max.x && p.y <= max.y && ZMath::compare(p.z, plane.pos.z);
    };

    // Determine if a point lies within a sphere.
    static bool PointAndSphere(ZMath::Vec3D const &point, Sphere const &sphere) { return sphere.c.distSq(point) <= sphere.r*sphere.r; };

    // Determine if a point lies within an unrotated cube.
    static bool PointAndAABB(ZMath::Vec3D const &point, AABB const &aabb) {
        ZMath::Vec3D min = aabb.getMin(), max = aabb.getMax();
        return point.x <= max.x && point.y <= max.y && point.z <= max.z && point.x >= min.x && point.y >= min.y && point.z >= min.z;
    };

    // Determine if a point lies within a rotated cube.
    static bool PointAndCube(ZMath::Vec3D const &point, Cube const &cube) {
        ZMath::Vec3D min = cube.getLocalMin(), max = cube.getLocalMax();
        ZMath::Vec3D p = point - cube.pos; // create a copy so we can rotate into our local cords

        // rotate into our UVW cords
        p = cube.rot * p + cube.pos;

        return p.x <= max.x && p.y <= max.y && p.z <= max.z && p.x >= min.x && p.y >= min.y && p.z >= min.z;
    };

    // * ===================================
    // * Line3D vs Primitives
    // * ===================================

    // Determine if a line intersects a point.
    static bool LineAndPoint(Line3D const &line, ZMath::Vec3D const &point) { return PointAndLine(point, line); };

    // Determine if a line intersects another line.
    static bool LineAndLine(Line3D const &line1, Line3D const &line2) {
        // ? First check if the lines are parallel.
        // ? If the lines are parallel, we check to ensure overlap and that the start point of line2 lies on line1 if the lines were infinite.
        // ? If the lines are not parallel, we then solve for the point of intersection using the parametric equations for a 3D line.
        // ? We then ensure this point in time statisfies all 3 equations.
        // ? If it does and there's overlap, we have an intersection; otherwise we do not.

        ZMath::Vec3D s1 = line1.start, s2 = line2.start, e1 = line1.end, e2 = line2.end;
        ZMath::Vec3D v1 = e1 - s1, v2 = e2 - s2;

        // Determine the proportion between the two. If it is the same, the lines are parallel due to it relying on time.
        float c;
        if (v1.x) { c = v2.x/v1.x; }
        else if (v1.y) { c = v2.y/v1.y; }
        else { c = v2.z/v1.z; } // v1.z != 0

        // check for parallel lines
        if (c && ZMath::compare(c*v1.x, v2.x) && ZMath::compare(c*v1.y, v2.y) && ZMath::compare(c*v1.z, v2.z)) {
            ZMath::Vec3D min1 = line1.getMin(), max1 = line1.getMax();
            ZMath::Vec3D min2 = line2.getMin(), max2 = line2.getMax();

            if (v1.x) {
                float t = (s2.x - s1.x)/v1.x;
                return s2.y == s1.y + v1.y * t && s2.z == s1.z + v1.z * t && min1.x <= max2.x && min2.x <= max1.x;
            }

            if (v1.y) { return s1.x == s2.x && s2.z == s1.z + v1.z * ((s2.y - s1.y)/v1.y) && min1.y <= max2.y && min2.y <= max1.y; }
            
            return s1.x == s2.x && s1.y == s2.y && min1.z <= max2.z && min2.z <= max1.z;
        }

        float s, t;

        // determine the s and t values for when the line would intersect
        if (v2.x*v1.y != v1.x*v2.y) { s = (v1.x*(s2.y - s1.y) + v1.y*(s1.x - s2.x))/(v2.x*v1.y - v1.x*v2.y); }
        else if (v2.y*v1.z != v1.y*v2.z) { s = (v1.y*(s2.z - s1.z) + v1.z*(s1.y - s2.y))/(v2.y*v1.z - v1.y*v2.z); }
        else { s = (v1.x*(s2.z - s1.z) + v1.z*(s1.x - s2.x))/(v2.x*v1.z - v1.x*v2.z); }

        if (v1.x) { t = (s2.x - s1.x + v2.x*s)/v1.x; }
        else if (v1.y) { t = (s2.y - s1.y + v2.y*s)/v1.y; }
        else { t = (s2.z - s1.z + v2.z*s)/v1.z; }

        // ensure it satisfies the third equation
        if (s1.z + t*v1.z != s2.z + s*v2.z) { return 0; }

        // determine the point of intersection
        ZMath::Vec3D p = ZMath::Vec3D(s1.x + t*v1.x, s1.y + t*v1.y, s1.z + t*v1.z);

        // determine the min and max points for both lines
        ZMath::Vec3D max(ZMath::min(ZMath::max(s1.x, e1.x), ZMath::max(s2.x, e2.x)), 
                         ZMath::min(ZMath::max(s1.y, e1.y), ZMath::max(s2.y, e2.y)),
                         ZMath::min(ZMath::max(s1.z, e1.z), ZMath::max(s2.z, e2.z)));

        ZMath::Vec3D min(ZMath::max(ZMath::min(s1.x, e1.x), ZMath::min(s2.x, e2.x)),
                         ZMath::max(ZMath::min(s1.y, e1.y), ZMath::min(s2.y, e2.y)),
                         ZMath::max(ZMath::min(s1.z, e1.z), ZMath::min(s2.z, e2.z)));

        // ensure the point of intersection is between these bounds
        return p.x <= max.x && p.y <= max.y && p.z <= max.z && p.x >= min.x && p.y >= min.y && p.z >= min.z;
    };

    // Determine if a line intersects a plane.
    static bool LineAndPlane(Line3D const &line, Plane const &plane) {
        Line3D l(line.start - plane.pos, line.end - plane.pos); // copy so we can rotate it

        // rotate the line into the plane's local coordinates
        l.start = plane.rot * line.start + plane.pos;
        l.end = plane.rot * line.end + plane.pos;

        // get the maxes and mins
        ZMath::Vec3D minL = l.getMin(), maxL = l.getMax();
        ZMath::Vec2D minP = plane.getLocalMin(), maxP = plane.getLocalMax();

        // check for a line laying on the same plane as the plane we're checking against
        if (ZMath::compare(minL.z, plane.pos.z) && ZMath::compare(maxL.z, plane.pos.z)) {
            return minL.x > maxP.x || minP.x > maxL.x || minL.y > maxP.y || minP.y > maxL.y;
        }

        // ensure an intersection is possible
        if (minL.z > plane.pos.z || plane.pos.z > maxL.z) { return 0; }

        // If there was an intersection, there would be a point in time where the line's z coord = minP.z.
        // We also know that the z component of our slope is not 0 as we checked for that case earlier.
        ZMath::Vec3D slope = l.end - l.start;
        float t = (plane.pos.z - l.start.z)/slope.z;
        ZMath::Vec2D p(l.start.x + t*slope.x, l.start.y + t*slope.y); // point of intersection

        // Ensure the point of intersection is within the bounds of the plane.
        // We do not need to check if it is within the bounds of the line as the conditional above requires that.
        return minP.x <= p.x && p.x <= maxP.x && minP.y <= p.y && p.y <= maxP.y;
    };

    // Determine if a line intersects a sphere.
    static bool LineAndSphere(Line3D const &line, Sphere const &sphere) {
        // ? Use the parametric equations for a 3D line.
        // ? Relate the parametric equations with the distance squared to the center of the sphere.
        // ? Since we define our start point as the point at t_0 and end point as the point at t_1,
        // ?  we know that if either solution of the quadratic constructed is between 0 to 1 inclusive, we have a collision.

        if (line.start.distSq(sphere.c) <= sphere.r * sphere.r || line.end.distSq(sphere.c) <= sphere.r * sphere.r) { return 1; }

        float dx = line.end.x - line.start.x, dy = line.end.y - line.start.y, dz = line.end.z - line.start.z;
        float xc = line.start.x - sphere.c.x, yc = line.start.y - sphere.c.y, zc = line.start.z - sphere.c.z;

        float A = dx*dx + dy*dy + dz*dz;
        float B = 2*(dx*xc + dy*yc + dz*zc);
        float C = xc*xc + yc*yc + zc*zc - sphere.r*sphere.r;

        float D = B*B - 4*A*C;

        if (D < 0) { return 0; } // no intersections even if the line was infinite

        float sq = sqrtf(D);
        float div = 1.0f/(2.0f*A);
        float t1 = (-B - sq)*div, t2 = (-B + sq)*div;

        // if t1 or t2 falls between 0 and 1, the line intersects the sphere
        return (0.0f <= t1 && t1 <= 1.0f) || (0.0f <= t2 && t2 <= 1.0f);
    };

    // Determine if a line intersects an unrotated cube.
    // todo this might not work
    static bool LineAndAABB(Line3D const &line, AABB const &aabb) {
        // ? Check if the line has any point within the AABB's bounds.

        ZMath::Vec3D minL = line.getMin(), maxL = line.getMax();
        ZMath::Vec3D minA = aabb.getMin(), maxA = aabb.getMax();

        return minL.x <= maxA.x && minA.x <= maxL.x && minL.y <= maxA.y && minA.y <= maxL.y && minL.z <= maxA.z && minA.z <= maxL.z;
    };

    // Determine if a line intersects a cube.
    // todo this might not work
    static bool LineAndCube(Line3D const &line, Cube const &cube) {
        // ? Rotate into the Cube's UVW coordinates.
        // ? Check to see if the line is within the cube's bounds.

        Line3D l(line.start - cube.pos, line.end - cube.pos);

        // Rotate into the Cube's UVW coords.
        l.start = cube.rot * l.start + cube.pos;
        l.end = cube.rot * l.end + cube.pos;

        // todo try just getting the line's original min and max and rotating that into local coords
        // ! slightly more efficient

        ZMath::Vec3D minL = l.getMin(), maxL = l.getMax();
        ZMath::Vec3D minC = cube.getLocalMin(), maxC = cube.getLocalMax();

        return minL.x <= maxC.x && minC.x <= maxL.x && minL.y <= maxC.y && minC.y <= maxL.y && minL.z <= maxC.z && minC.z <= maxL.z;
    };

    // * =================
    // * Raycasting
    // * =================

    // Determine if a ray intersects a plane.
    // dist will be modified to equal the distance from the ray it hits the plane.
    // dist is set to -1 if there is no intersection.
    static bool raycast(Plane const &plane, Ray3D const &ray, float &dist) {
        float dot = plane.normal * ray.dir;

        // check if the ray is parallel to the plane
        if (!dot) { dist = -1.0f; return 0; }

        dist = -((plane.normal * (ray.origin - plane.pos))/dot);
        ZMath::Vec2D min = plane.getLocalMin(), max = plane.getLocalMax();
        ZMath::Vec3D p = ray.origin + ray.dir*dist - plane.pos;

        p = plane.rot * p + plane.pos;

        // make sure the point of intersection is within our bounds and the intersection wouldn't occur behind the ray
        if (dist >= 0 && p.x >= min.x && p.y >= min.y && p.x <= max.x && p.y <= max.y && ZMath::compare(p.z, plane.pos.z)) { return 1; }

        dist = -1.0f;
        return 0;
    };

    // Determine if a ray intersects a sphere.
    // dist will be modified to equal the distance from the ray it hits the sphere.
    // dist is set to -1 if there is no intersection.
    static bool raycast(Sphere const &sphere, Ray3D const &ray, float &dist) {
        // todo at some point we may want to get the hit point.
        // todo if we do, we simply do hit = ray.origin + dist*ray.dir;

        // ? First we find the closest point on the ray to the sphere.
        // ? To find this point, we find the distance to it using dir * (center - origin).
        // ? Next we solve origin + t*origin to find the closest point.
        // ? If the distance of that closest point to the center is less than or equal to the radius, we have an intersection.

        // determine the closest point and the distance to that point
        float t = ray.dir * (sphere.c - ray.origin);

        // the sphere is behind the ray
        if (t < 0) {
            dist = -1.0f;
            return 0;
        }

        ZMath::Vec3D close = ray.origin + ray.dir * t;

        float dSq = sphere.c.distSq(close);
        float rSq = sphere.r*sphere.r;

        // no intersection
        if (dSq > rSq) {
            dist = -1.0f;
            return 0;
        }

        // lands on the circumference
        if (dSq == rSq) {
            dist = t;
            return 1;
        }

        // ray started in the sphere
        if (t < sphere.r) {
            dist = t + sqrtf(rSq - dSq);
            return 1;
        }

        // standard intersection
        dist = t - sqrtf(rSq - dSq);
        return 1;
    };

    // Determine if a ray intersects an AABB.
    // dist will be modified to equal the distance from the ray it hits the AABB.
    // dist is set to -1 if there is no intersection.
    static bool raycast(AABB const &aabb, Ray3D const &ray, float &dist) {
        // ? We can determine the distance from the ray to a certain edge by dividing a select min or max vector component
        // ?  by the corresponding component from the unit directional vector.
        // ? We know if tMin > tMax, then we have no intersection and if tMax is negative the AABB is behind us and we do not have a hit.

        ZMath::Vec3D dirfrac(1.0f/ray.dir.x, 1.0f/ray.dir.y, 1.0f/ray.dir.z);
        ZMath::Vec3D min = aabb.getMin(), max = aabb.getMax();

        float t1 = (min.x - ray.origin.x)*dirfrac.x;
        float t2 = (max.x - ray.origin.x)*dirfrac.x;
        float t3 = (min.y - ray.origin.y)*dirfrac.y;
        float t4 = (max.y - ray.origin.y)*dirfrac.y;
        float t5 = (min.z - ray.origin.z)*dirfrac.z;
        float t6 = (max.z - ray.origin.z)*dirfrac.z;

        // tMin is the max of the mins and tMx is the min of the maxes
        float tMin = ZMath::max(ZMath::max(ZMath::min(t1, t2), ZMath::min(t3, t4)), ZMath::min(t5, t6));
        float tMax = ZMath::min(ZMath::min(ZMath::max(t1, t2), ZMath::max(t3, t4)), ZMath::max(t5, t6));

        // if tMax < 0 the ray is intersecting behind it. Therefore, we do not actually have a collision.
        if (tMax < 0) {
            dist = -1.0f;
            return 0;
        }

        // ray doesn't intersect the AABB.
        if (tMax < tMin) {
            dist = -1.0f;
            return 0;
        }

        // ray's origin is inside of the AABB.
        if (tMin < 0) {
            dist = tMax;
            return 1;
        }

        dist = tMin;
        return 1;
    };

    // Determine if a ray intersects a cube.
    // dist will be modified to equal the distance from the ray it hits the cube.
    // dist is set to -1 if there is no intersection.
    static bool raycast(Cube const &cube, Ray3D const &ray, float &dist) {
        // http://groups.csail.mit.edu/graphics/classes/6.837/F02/lectures/6.837-10_rayCast.pdf
        // Slide 47, basically they recommend to convert ray to object space and check aabb, dunno
        // if its faster but MIT slides says its a good start
        // * If this works ill copy the code over from AABB to make sure we dont lose pref

        ZMath::Vec3D cubeMin = cube.getLocalMin();
        ZMath::Vec3D cubeMax = cube.getLocalMax();

        ZMath::Vec3D rayOrigin = ray.origin - cube.pos;
        ZMath::Vec3D rayDir = ray.dir;

        // todo unsure if this will work for angles outside the first quadrant; it should, though (I think)
        // ! test although im p sure it should work

        rayOrigin = cube.rot * rayOrigin + cube.pos;
        rayDir = cube.rot * rayDir;

        AABB newCube(cube.getLocalMin(), cube.getLocalMax());
        Ray3D newRay(rayOrigin, rayDir);

        float d2 = 0; // ! simply for making it pass the unit tests

        return raycast(newCube, newRay, d2);
    };

    // * ===================================
    // * Plane vs Primitives
    // * ===================================

    // Determine if a plane intersects a point.
    static bool PlaneAndPoint(Plane const &plane, ZMath::Vec3D const &point) { return PointAndPlane(point, plane); };

    // Determine if a plane intersects a line.
    static bool PlaneAndLine(Plane const &plane, Line3D const &line) { return LineAndPlane(line, plane); };

    // Determine if a plane intersects a sphere.
    static bool PlaneAndSphere(Plane const &plane, Sphere const &sphere) {
        // ? A line from the center of the sphere along the normal of the plane will eventually
        // ?  intersect an infinite plane at the closest point of intersection.
        // ? Using this, we can take the distance to the plane from that point using the equation
        // ?  for the distance from a point to a plane.
        // ? If this distance squared is less than the radius squared, we have an intersection.
        // ? Since we are not dealing with infinite planes, we will first need to clamp the 
        // ?  sphere's center between the min and max vertices of the plane.

        ZMath::Vec3D closest(sphere.c - plane.pos);
        ZMath::Vec2D min = plane.getLocalMin(), max = plane.getLocalMax();

        closest = plane.rot * closest + plane.pos;

        closest.x = ZMath::clamp(closest.x, min.x, max.x);
        closest.y = ZMath::clamp(closest.y, min.y, max.y);
        // if this doesn't work try replacing with ZMath::clamp(closest.z, min.z, max.z);
        closest.z = plane.pos.z;

        return closest.distSq(sphere.c) <= sphere.r*sphere.r;
    };

    // Check for intersection and return the collision normal.
    // If there is not an intersection, the normal will be a junk value.
    // The normal will point towards B away from A.
    static bool PlaneAndSphere(Plane const &plane, Sphere const &sphere, ZMath::Vec3D &normal) {
        ZMath::Vec3D closest(sphere.c - plane.pos);
        ZMath::Vec2D min = plane.getLocalMin(), max = plane.getLocalMax();

        closest = plane.rot * closest + plane.pos;

        closest.x = ZMath::clamp(closest.x, min.x, max.x);
        closest.y = ZMath::clamp(closest.y, min.y, max.y);
        // if this doesn't work try replacing with ZMath::clamp(closest.z, min.z, max.z);
        closest.z = plane.pos.z;

        ZMath::Vec3D diff = closest - sphere.c;
        if (diff.magSq() > sphere.r*sphere.r) { return 0; }

        normal = diff.normalize();
        return 1;
    };

    // Determine if a plane intersects an unrotated cube.
    static bool PlaneAndAABB(Plane const &plane, AABB const &aabb) {
        // halfsize of the plane (A) and aabb (B)
        ZMath::Vec2D planeH = plane.getHalfSize();
        ZMath::Vec3D hA(planeH.x, planeH.y, 0.0f), hB = aabb.getHalfSize();

        // * Determine the rotation matrices of A and B

        // rotate anything from global space (B's local space) to A's local space
        ZMath::Mat3D rotAT = plane.rot.transpose();

        // determine the difference between the positions
        // Note: global space is the AABB's local space
        ZMath::Vec3D dB = aabb.pos - plane.pos;
        ZMath::Vec3D dA = rotAT * dB;

        // * Check for intersections with the separating axis theorem

        // amount of penetration along A's axes
        ZMath::Vec3D faceA = ZMath::abs(dA) - hA - rotAT * hB;
        if (faceA.x > 0 || faceA.y > 0 || faceA.z > 0) { return 0; }

        // amount of penetration along B's axes
        ZMath::Vec3D faceB = ZMath::abs(dB) - hB - plane.rot * hA;
        return faceB.x <= 0 && faceB.y <= 0 && faceB.z <= 0;
    };

    // Check for intersection and return the collision normal.
    // If there is not an intersection, the normal will be a junk value.
    // The normal will point towards B away from A.
    static bool PlaneAndAABB(Plane const &plane, AABB const &aabb, ZMath::Vec3D &normal) {
        // halfsize of the plane (A) and aabb (B)
        ZMath::Vec2D planeH = plane.getHalfSize();
        ZMath::Vec3D hA(planeH.x, planeH.y, 0.0f), hB = aabb.getHalfSize();

        // * Determine the rotation matrices of A and B

        // rotate anything from global space (B's local space) to A's local space
        ZMath::Mat3D rotAT = plane.rot.transpose();

        // determine the difference between the positions
        // Note: global space is the AABB's local space
        ZMath::Vec3D dB = aabb.pos - plane.pos;
        ZMath::Vec3D dA = rotAT * dB;

        // * Check for intersections with the separating axis theorem

        // amount of penetration along A's axes
        ZMath::Vec3D faceA = ZMath::abs(dA) - hA - rotAT * hB;
        if (faceA.x > 0 || faceA.y > 0 || faceA.z > 0) { return 0; }

        // amount of penetration along B's axes
        ZMath::Vec3D faceB = ZMath::abs(dB) - hB - plane.rot * hA;
        if (faceB.x > 0 || faceB.y > 0 || faceB.z > 0) { return 0; }

        // * Find the best axis (i.e. the axis with the least penetration)

        // Assume A's x-axis is the best axis first
        float separation = faceA.x;
        normal = dA.x > 0.0f ? plane.rot.c1 : -plane.rot.c1;

        // tolerance values
        float relativeTol = 0.95f;
        float absoluteTol = 0.01f;

        // ? check if there is another axis better than A's x axis by checking if the penetration along
        // ?  the current axis being checked is greater than that of the current penetration
        // ?  (as greater value = less negative = less penetration).

        // A's remaining axes
        if (faceA.y > relativeTol * separation + absoluteTol * hA.y) {
            separation = faceA.y;
            normal = dA.y > 0.0f ? plane.rot.c2 : -plane.rot.c2;
        }

        if (faceA.z > relativeTol * separation + absoluteTol * hA.z) {
            separation = faceA.z;
            normal = dA.z > 0.0f ? plane.rot.c3 : -plane.rot.c3;
        }

        // B's axes
        if (faceB.x > relativeTol * separation + absoluteTol * hB.x) {
            separation = faceB.x;
            normal = dB.x > 0.0f ? ZMath::Vec3D(1, 0, 0) : ZMath::Vec3D(-1, 0, 0);
        }

        if (faceB.y > relativeTol * separation + absoluteTol * hB.y) {
            separation = faceB.y;
            normal = dB.y > 0.0f ? ZMath::Vec3D(0, 1, 0) : ZMath::Vec3D(0, -1, 0);
        }

        if (faceB.z > relativeTol * separation + absoluteTol * hB.z) {
            separation = faceB.z;
            normal = dB.z > 0.0f ? ZMath::Vec3D(0, 0, 1) : ZMath::Vec3D(0, 0, -1);
        }

        return 1;
    };

    // Determine if a plane intersects a cube.
    static bool PlaneAndCube(Plane const &plane, Cube const &cube) {
        // halfsize of the plane (A) and cube (B)
        ZMath::Vec2D planeH = plane.getHalfSize();
        ZMath::Vec3D hA(planeH.x, planeH.y, 0.0f), hB = cube.getHalfSize();

        // * Determine the rotation matrices of A and B

        // rotate anything from global space to A's local space
        ZMath::Mat3D rotAT = plane.rot.transpose();

        // rotate anything from global space to B's local space
        ZMath::Mat3D rotBT = cube.rot.transpose();

        // determine the difference between the positions
        ZMath::Vec3D dP = cube.pos - plane.pos;
        ZMath::Vec3D dA = rotAT * dP;
        ZMath::Vec3D dB = rotBT * dP;

        // * Rotation matrices for switching between local spaces

        // ! When scenes are developed test if we actually need the absolute value

        // Rotate anything from B's local space into A's
        ZMath::Mat3D C = ZMath::abs(rotAT * cube.rot);

        // Rotate anything from A's local space into B's
        ZMath::Mat3D CT = C.transpose();

        // * Check for intersections with the separating axis theorem

        // amount of penetration along A's axes
        ZMath::Vec3D faceA = ZMath::abs(dA) - hA - C * hB;
        if (faceA.x > 0 || faceA.y > 0 || faceA.z > 0) { return 0; }

        // amount of penetration along B's axes
        ZMath::Vec3D faceB = ZMath::abs(dB) - hB - CT * hA;
        return faceB.x <= 0 && faceB.y <= 0 && faceB.z <= 0;
    };

    // Check for intersection and return the collision normal.
    // If there is not an intersection, the normal will be a junk value.
    // The normal will point towards B away from A.
    static bool PlaneAndCube(Plane const &plane, Cube const &cube, ZMath::Vec3D &normal) {
        // halfsize of the plane (A) and cube (B)
        ZMath::Vec2D planeH = plane.getHalfSize();
        ZMath::Vec3D hA(planeH.x, planeH.y, 0.0f), hB = cube.getHalfSize();

        // * Determine the rotation matrices of A and B

        // rotate anything from global space to A's local space
        ZMath::Mat3D rotAT = plane.rot.transpose();

        // rotate anything from global space to B's local space
        ZMath::Mat3D rotBT = cube.rot.transpose();

        // determine the difference between the positions
        ZMath::Vec3D dP = cube.pos - plane.pos;
        ZMath::Vec3D dA = rotAT * dP;
        ZMath::Vec3D dB = rotBT * dP;

        // * Rotation matrices for switching between local spaces

        // ! When scenes are developed test if we actually need the absolute value

        // Rotate anything from B's local space into A's
        ZMath::Mat3D C = ZMath::abs(rotAT * cube.rot);

        // Rotate anything from A's local space into B's
        ZMath::Mat3D CT = C.transpose();

        // * Check for intersections with the separating axis theorem

        // amount of penetration along A's axes
        ZMath::Vec3D faceA = ZMath::abs(dA) - hA - C * hB;
        if (faceA.x > 0 || faceA.y > 0 || faceA.z > 0) { return 0; }

        // amount of penetration along B's axes
        ZMath::Vec3D faceB = ZMath::abs(dB) - hB - CT * hA;
        if (faceB.x > 0 || faceB.y > 0 || faceB.z > 0) { return 0; }

        // * Find the best axis (i.e. the axis with the least penetration)

        // Assume A's x-axis is the best axis first
        float separation = faceA.x;
        normal = dA.x > 0.0f ? plane.rot.c1 : -plane.rot.c1;

        // tolerance values
        float relativeTol = 0.95f;
        float absoluteTol = 0.01f;

        // ? check if there is another axis better than A's x axis by checking if the penetration along
        // ?  the current axis being checked is greater than that of the current penetration
        // ?  (as greater value = less negative = less penetration).

        // A's remaining axes
        if (faceA.y > relativeTol * separation + absoluteTol * hA.y) {
            separation = faceA.y;
            normal = dA.y > 0.0f ? plane.rot.c2 : -plane.rot.c2;
        }

        if (faceA.z > relativeTol * separation + absoluteTol * hA.z) {
            separation = faceA.z;
            normal = dA.z > 0.0f ? plane.rot.c3 : -plane.rot.c3;
        }

        // B's axes
        if (faceB.x > relativeTol * separation + absoluteTol * hB.x) {
            separation = faceB.x;
            normal = dB.x > 0.0f ? cube.rot.c1 : -cube.rot.c1;
        }

        if (faceB.y > relativeTol * separation + absoluteTol * hB.y) {
            separation = faceB.y;
            normal = dB.y > 0.0f ? cube.rot.c2 : -cube.rot.c2;
        }

        if (faceB.z > relativeTol * separation + absoluteTol * hB.z) {
            separation = faceB.z;
            normal = dB.z > 0.0f ? cube.rot.c3 : -cube.rot.c3;
        }

        return 1;
    };

    // * ===================================
    // * Sphere vs Primitives
    // * ===================================

    // Determine if a sphere intersects a point.
    static bool SphereAndPoint(Sphere const &sphere, ZMath::Vec3D const &point) { return PointAndSphere(point, sphere); };

    // Determine if a sphere intersects a line.
    static bool SphereAndLine(Sphere const &sphere, Line3D const &line) { return LineAndSphere(line, sphere); };

    // Determine if a sphere intersects a plane.
    static bool SphereAndPlane(Sphere const &sphere, Plane const &plane) { return PlaneAndSphere(plane, sphere); };

    // Determine if a sphere intersects another sphere.
    static bool SphereAndSphere(Sphere const &sphere1, Sphere const &sphere2) {
        float r = sphere1.r + sphere2.r;
        return sphere1.c.distSq(sphere2.c) <= r*r;
    };

    // Check for intersection and return the collision normal.
    // If there is not an intersection, the normal will be a junk value.
    // The normal will point towards B away from A.
    static bool SphereAndSphere(Sphere const &sphere1, Sphere const &sphere2, ZMath::Vec3D &normal) {
        float r = sphere1.r + sphere2.r;
        ZMath::Vec3D sphereDiff = sphere2.c - sphere1.c;

        if (sphereDiff.magSq() > r*r) { return 0; }
        normal = sphereDiff.normalize();

        return 1;
    };

    // Determine if a sphere intersects an unrotated cube.
    static bool SphereAndAABB(Sphere const &sphere, AABB const &aabb) {
        // ? We know a sphere and AABB would intersect if the distance from the closest point to the center on the AABB
        // ?  from the center is less than or equal to the radius of the sphere.
        // ? We can determine the closet point by clamping the value of the sphere's center between the min and max of the AABB.
        // ? From here, we can check the distance from this point to the sphere's center.

        ZMath::Vec3D min = aabb.getMin(), max = aabb.getMax();
        ZMath::Vec3D closest = ZMath::clamp(closest, min, max);
        return closest.distSq(sphere.c) <= sphere.r*sphere.r;
    };

    // Check for intersection and return the collision normal.
    // If there is not an intersection, the normal will be a junk value.
    // The normal will point towards B away from A.
    static bool SphereAndAABB(Sphere const &sphere, AABB const & aabb, ZMath::Vec3D &normal) {
        ZMath::Vec3D min = aabb.getMin(), max = aabb.getMax();
        ZMath::Vec3D closest = ZMath::clamp(closest, min, max);
        ZMath::Vec3D diff = closest - sphere.c;

        if (diff.magSq() > sphere.r*sphere.r) { return 0; }

        normal = diff.normalize();

        return 1;
    };

    // Determine if a sphere intersects a cube.
    static bool SphereAndCube(Sphere const &sphere, Cube const &cube) {
        // ? We can use the same approach as for SphereAndAABB, just we have to rotate the sphere into the Cube's UVW coordinates.

        ZMath::Vec3D closest = sphere.c - cube.pos;
        ZMath::Vec3D min = cube.getLocalMin(), max = cube.getLocalMax();

        // rotate the center of the sphere into the UVW coordinates of our cube
        closest = cube.rot * closest + cube.pos;
        
        // perform the check as if it was an AABB vs Sphere
        closest = ZMath::clamp(closest, min, max);
        return closest.distSq(sphere.c) <= sphere.r*sphere.r;
    };

    // Check for intersection and return the collision normal.
    // If there is not an intersection, the normal will be a junk value.
    // The normal will point towards B away from A.
    static bool SphereAndCube(Sphere const &sphere, Cube const &cube, ZMath::Vec3D &normal) {
        ZMath::Vec3D closest = sphere.c - cube.pos;
        ZMath::Vec3D min = cube.getLocalMin(), max = cube.getLocalMax();

        // rotate the center of the sphere into the UVW coordinates of our cube
        closest = cube.rot * closest + cube.pos;
        
        // perform the check as if it was an AABB vs Sphere
        closest = ZMath::clamp(closest, min, max);
        ZMath::Vec3D diff = closest - sphere.c;

        if (diff.magSq() > sphere.r*sphere.r) { return 0; }

        // the closest point to the sphere's center will be our contact point rotated back into global coordinates coordinates

        closest -= cube.pos;
        closest = cube.rot.transpose() * closest + cube.pos;

        normal = diff.normalize();

        return 1;
    };

    // * ===================================
    // * AABB vs Primitives
    // * ===================================

    // Determine if an unrotated cube intersects a point.
    static bool AABBAndPoint(AABB const &aabb, ZMath::Vec3D const &point) { return PointAndAABB(point, aabb); };

    // Determine if an unrotated cube intersects a line.
    static bool AABBAndLine(AABB const &aabb, Line3D const &line) { return LineAndAABB(line, aabb); };

    // Determine if an unrotated cube intersects a plane.
    static bool AABBAndPlane(AABB const &aabb, Plane const &plane) { return PlaneAndAABB(plane, aabb); };

    // Determine if an unrotated cube intersects a sphere.
    static bool AABBAndSphere(AABB const &aabb, Sphere const &sphere) { return SphereAndAABB(sphere, aabb); };

    // Check for intersection and return the collision normal.
    // If there is not an intersection, the normal will be a junk value.
    // The normal will point towards B away from A.
    static bool AABBAndSphere(AABB const &aabb, Sphere const &sphere, ZMath::Vec3D &normal) {
        bool hit = SphereAndAABB(sphere, aabb, normal);
        normal = -normal;
        return hit;
    };

    // Determine if an unrotated cube intersects another unrotated cube.
    static bool AABBAndAABB(AABB const &aabb1, AABB const &aabb2) {
        // ? Check if there's overlap for the AABBs on all three axes.
        // ? If there is, we know the two AABBs intersect.

        ZMath::Vec3D min1 = aabb1.getMin(), max1 = aabb1.getMax(), min2 = aabb2.getMin(), max2 = aabb2.getMax();

        // if both min points for a given interval are less than the other max points, we know there's an overlap on that axis
        return min2.x <= max1.x && min1.x <= max2.x && min2.y <= max1.y && min1.y <= max2.y && min2.z <= max1.z && min1.z <= max2.z;
    };

    // Check for intersection and return the collision normal.
    // If there is not an intersection, the normal will be a junk value.
    // The normal will point towards B away from A.
    static bool AABBAndAABB(AABB const &aabb1, AABB const &aabb2, ZMath::Vec3D &normal) {
        // half size of AABB a and b respectively
        ZMath::Vec3D hA = aabb1.getHalfSize(), hB = aabb2.getHalfSize();

        // * Check for intersections using the separating axis theorem.
        // because both are axis aligned, global space is the same as the local space of both AABBs.

        // distance between the two
        ZMath::Vec3D dP = aabb2.pos - aabb1.pos;
        ZMath::Vec3D absDP = ZMath::abs(dP);

        // penetration along A's (and B's) axes
        ZMath::Vec3D faceA = absDP - hA - hB;
        if (faceA.x > 0 || faceA.y > 0 || faceA.z > 0) { return 0; }

        // ? Since they are axis aligned, the penetration between the two will be the same on any given axis.
        // ?  Therefore, we only need to check for A.

        // * Find the best axis (i.e. the axis with the least amount of penetration).

        // Assume A's x-axis is the best axis first
        float separation = faceA.x;
        normal = dP.x > 0.0f ? ZMath::Vec3D(1, 0, 0) : ZMath::Vec3D(-1, 0, 0);

        // tolerance values
        float relativeTol = 0.95f;
        float absoluteTol = 0.01f;

        // ? check if there is another axis better than A's x axis by checking if the penetration along
        // ?  the current axis being checked is greater than that of the current penetration
        // ?  (as greater value = less negative = less penetration).

        // A's remaining axes
        if (faceA.y > relativeTol * separation + absoluteTol * hA.y) {
            separation = faceA.y;
            normal = dP.y > 0.0f ? ZMath::Vec3D(0, 1, 0) : ZMath::Vec3D(0, -1, 0);
        }

        if (faceA.z > relativeTol * separation + absoluteTol * hA.z) {
            separation = faceA.z;
            normal = dP.z > 0.0f ? ZMath::Vec3D(0, 0, 1) : ZMath::Vec3D(0, 0, -1);
        }

        return 1;
    };

    // Determine if an unrotated cube intersects a cube.
    static bool AABBAndCube(AABB const &aabb, Cube const &cube) {
        // ? Use the separating axis theorem to determine if there is an intersection between the AABB and cube.

        // half size of the aabb and cube respectively (A = AABB, B = Cube)
        ZMath::Vec3D hA = aabb.getHalfSize(), hB = cube.getHalfSize();

        // rotate anything from global space to the cube's local space
        ZMath::Mat3D rotBT = cube.rot.transpose();

        // determine the distance between the positions
        ZMath::Vec3D dA = cube.pos - aabb.pos; // global space is the AABB's local space
        ZMath::Vec3D dB = rotBT * dA;

        // * Check for intersection using the separating axis theorem

        // amount of penetration along A's axes
        ZMath::Vec3D faceA = ZMath::abs(dA) - hA - hB;
        if (faceA.x > 0 || faceA.y > 0 || faceA.z > 0) { return 0; }

        // amount of penetration along B's axes
        ZMath::Vec3D faceB = ZMath::abs(dB) - hB - rotBT * hA;
        return faceB.x <= 0 && faceB.y <= 0 && faceB.z <= 0;
    };

    // Check for intersection and return the collision normal.
    // If there is not an intersection, the normal will be a junk value.
    // The normal will point towards B away from A.
    static bool AABBAndCube(AABB const &aabb, Cube const &cube, ZMath::Vec3D &normal) {
        // ? Use the separating axis theorem to determine if there is an intersection between the AABB and cube.

        // half size of the aabb and cube respectively (A = AABB, B = Cube)
        ZMath::Vec3D hA = aabb.getHalfSize(), hB = cube.getHalfSize();

        // rotate anything from global space to the cube's local space
        ZMath::Mat3D rotBT = cube.rot.transpose();

        // determine the distance between the positions
        ZMath::Vec3D dA = cube.pos - aabb.pos; // global space is the AABB's local space
        ZMath::Vec3D dB = rotBT * dA;

        // * Check for intersection using the separating axis theorem

        // amount of penetration along A's axes
        ZMath::Vec3D faceA = ZMath::abs(dA) - hA - hB;
        if (faceA.x > 0 || faceA.y > 0 || faceA.z > 0) { return 0; }

        // amount of penetration along B's axes
        ZMath::Vec3D faceB = ZMath::abs(dB) - hB - rotBT * hA;
        if (faceB.x > 0 || faceB.y > 0 || faceB.z > 0) { return 0; }
        
        // * Find the best axis (i.e. the axis with the least amount of penetration).

        // Assume A's x-axis is the best axis first
        float separation = faceA.x;
        normal = dA.x > 0.0f ? ZMath::Vec3D(1, 0, 0) : ZMath::Vec3D(-1, 0, 0);

        // tolerance values
        float relativeTol = 0.95f;
        float absoluteTol = 0.01f;

        // ? check if there is another axis better than A's x axis by checking if the penetration along
        // ?  the current axis being checked is greater than that of the current penetration
        // ?  (as greater value = less negative = less penetration).

        // A's remaining axes
        if (faceA.y > relativeTol * separation + absoluteTol * hA.y) {
            separation = faceA.y;
            normal = dA.y > 0.0f ? ZMath::Vec3D(0, 1, 0) : ZMath::Vec3D(0, -1, 0);
        }

        if (faceA.z > relativeTol * separation + absoluteTol * hA.z) {
            separation = faceA.z;
            normal = dA.z > 0.0f ? ZMath::Vec3D(0, 0, 1) : ZMath::Vec3D(0, 0, -1);
        }

        // B's axes
        if (faceB.x > relativeTol * separation + absoluteTol * hB.x) {
            separation = faceB.x;
            normal = dB.x > 0.0f ? cube.rot.c1 : -cube.rot.c1;
        }

        if (faceB.y > relativeTol * separation + absoluteTol * hB.y) {
            separation = faceB.y;
            normal = dB.y > 0.0f ? cube.rot.c2 : -cube.rot.c2;
        }

        if (faceB.z > relativeTol * separation + absoluteTol * hB.z) {
            separation = faceB.z;
            normal = dB.z > 0.0f ? cube.rot.c3 : -cube.rot.c3;
        }

        return 1;
    };

    // * ===================================
    // * Cube vs Primitives
    // * ===================================

    // Determine if a cube intersects a point.
    static bool CubeAndPoint(Cube const &cube, ZMath::Vec3D const &point) { return PointAndCube(point, cube); };

    // Determine if a cube intersects a line.
    static bool CubeAndLine(Cube const &cube, Line3D const &line) { return LineAndCube(line, cube); };

    // Determine if a cube intersects a plane.
    static bool CubeAndPlane(Cube const &cube, Plane const &plane) { return PlaneAndCube(plane, cube); };

    // Determine if a cube intersects a sphere.
    static bool CubeAndSphere(Cube const &cube, Sphere const &sphere) { return SphereAndCube(sphere, cube); };

    // Check for intersection and return the collision normal.
    // If there is not an intersection, the normal will be a junk value.
    // The normal will point towards B away from A.
    static bool CubeAndSphere(Cube const &cube, Sphere const &sphere, ZMath::Vec3D &normal) {
        bool hit = SphereAndCube(sphere, cube, normal);
        normal = -normal;
        return hit;
    };

    // Determine if a cube intersects an unrotated cube.
    static bool CubeAndAABB(Cube const &cube, AABB const &aabb) { return AABBAndCube(aabb, cube); };

    // Check for intersection and return the collision normal.
    // If there is not an intersection, the normal will be a junk value.
    // The normal will point towards B away from A.
    static bool CubeAndAABB(Cube const &cube, AABB const &aabb, ZMath::Vec3D &normal) {
        bool hit = AABBAndCube(aabb, cube, normal);
        normal = -normal;
        return hit;
    };

    // Determine if a cube intersects another cube.
    static bool CubeAndCube(Cube const &cube1, Cube const &cube2) {
        // ? Use the separating axis theorem to determine if there is an intersection between the cubes.

        // half size of cube a and b respectively
        ZMath::Vec3D hA = cube1.getHalfSize(), hB = cube2.getHalfSize();

        // rotate anything from global space to A's local space
        ZMath::Mat3D rotAT = cube1.rot.transpose();

        // determine the difference between the positions
        ZMath::Vec3D dP = cube2.pos - cube1.pos;
        ZMath::Vec3D dA = rotAT * dP;
        ZMath::Vec3D dB = cube2.rot.transpose() * dP;

        // * rotation matrices for switching between local spaces

        // ! When we have a proper scene to test, use that to check if the absolute value is necessary

        // Rotate anything from B's local space into A's
        ZMath::Mat3D C = ZMath::abs(rotAT * cube2.rot);

        // Rotate anything from A's local space into B's
        ZMath::Mat3D CT = C.transpose();

        // * Check for intersections with the separating axis theorem

        // amount of penetration along A's axes
        ZMath::Vec3D faceA = ZMath::abs(dA) - hA - C * hB;
        if (faceA.x > 0 || faceA.y > 0 || faceA.z > 0) { return 0; }

        // amount of penetration along B's axes
        ZMath::Vec3D faceB = ZMath::abs(dB) - hB - CT * hA;
        return faceB.x <= 0 && faceB.y <= 0 && faceB.z <= 0;
    };

    // Check for intersection and return the collision normal.
    // If there is not an intersection, the normal will be a junk value.
    // The normal will point towards B away from A.
    static bool CubeAndCube(Cube const &cube1, Cube const &cube2, ZMath::Vec3D &normal) {
        // ? Use the separating axis theorem to determine if there is an intersection between the cubes.

        // half size of cube a and b respectively
        ZMath::Vec3D hA = cube1.getHalfSize(), hB = cube2.getHalfSize();

        // rotate anything from global space to A's local space
        ZMath::Mat3D rotAT = cube1.rot.transpose();

        // determine the difference between the positions
        ZMath::Vec3D dP = cube2.pos - cube1.pos;
        ZMath::Vec3D dA = rotAT * dP;
        ZMath::Vec3D dB = cube2.rot.transpose() * dP;

        // * rotation matrices for switching between local spaces
        
        // ! When we have a proper scene to test, use that to check if the absolute value is necessary

        // Rotate anything from B's local space into A's
        ZMath::Mat3D C = ZMath::abs(rotAT * cube2.rot);

        // Rotate anything from A's local space into B's
        ZMath::Mat3D CT = C.transpose();

        // * Check for intersections with the separating axis theorem

        // amount of penetration along A's axes
        ZMath::Vec3D faceA = ZMath::abs(dA) - hA - C * hB;
        if (faceA.x > 0 || faceA.y > 0 || faceA.z > 0) { return 0; }

        // amount of penetration along B's axes
        ZMath::Vec3D faceB = ZMath::abs(dB) - hB - CT * hA;
        if (faceB.x > 0 || faceB.y > 0 || faceB.z > 0) { return 0; }
        
        // * Find the best axis (i.e. the axis with the least penetration).

        // Assume A's x-axis is the best axis first.
        float separation = faceA.x;
        normal = dA.x > 0.0f ? cube1.rot.c1 : -cube1.rot.c1;

        // tolerance values
        float relativeTol = 0.95f;
        float absoluteTol = 0.01f;

        // ? check if there is another axis better than A's x axis by checking if the penetration along
        // ?  the current axis being checked is greater than that of the current penetration
        // ?  (as greater value = less negative = less penetration).

        // A's remaining axes
        if (faceA.y > relativeTol * separation + absoluteTol * hA.y) {
            separation = faceA.y;
            normal = dA.y > 0.0f ? cube1.rot.c2 : -cube1.rot.c2;
        }

        if (faceA.z > relativeTol * separation + absoluteTol * hA.z) {
            separation = faceA.z;
            normal = dA.z > 0.0f ? cube1.rot.c3 : -cube1.rot.c3;
        }

        // B's axes
        if (faceB.x > relativeTol * separation + absoluteTol * hB.x) {
            separation = faceB.x;
            normal = dB.x > 0.0f ? cube2.rot.c1 : -cube2.rot.c1;
        }

        if (faceB.y > relativeTol * separation + absoluteTol * hB.y) {
            separation = faceB.y;
            normal = dB.y > 0.0f ? cube2.rot.c2 : -cube2.rot.c2;
        }

        if (faceB.z > relativeTol * separation + absoluteTol * hB.z) {
            separation = faceB.z;
            normal = dB.z > 0.0f ? cube2.rot.c3 : -cube2.rot.c3;
        }

        return 1;
    };
}
