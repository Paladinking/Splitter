#include "polygon.h"
#include <stdio.h>
#include <math.h>

bool line_intersects(b2Vec2 pa1, b2Vec2 pa2, b2Vec2 pb1, b2Vec2 pb2, b2Vec2* intersect) {
    double denom = (pa1.x - pa2.x) * (pb1.y - pb2.y) - (pa1.y - pa2.y) * (pb1.x - pb2.x);
    if (denom == 0.0) {
        return false;
    }

    double t = ((pa1.x - pb1.x) * (pb1.y - pb2.y) - (pa1.y - pb1.y) * (pb1.x - pb2.x)) / denom;
    double u = -((pa1.x - pa2.x) * (pa1.y - pb1.y) - (pa1.y - pa2.y) * (pa1.x - pb1.x)) / denom;
    if (0.0 <= t && t <= 1.0 && 0.0 <= u && u <= 1.0) {
        intersect->x = pa1.x + t * (pa2.x - pa1.x);
        intersect->y = pa1.y + t * (pa2.y - pa1.y);
        return true;
    }
    return false;
}

bool point_eq(b2Vec2 p1, b2Vec2 p2) {
    return b2AbsFloat(p1.x - p2.x) + b2AbsFloat(p1.y - p2.y) < 0.01;
}


// Split polygon by line from p1 to p2
bool Polygon_split(Polygon* p, b2WorldId world, b2Vec2 p1, b2Vec2 p2, Polygon** a, Polygon** b) {
    b2Transform t = b2Body_GetTransform(p->body);
    p1 = b2InvTransformPoint(t, p1);
    p2 = b2InvTransformPoint(t, p2);

    b2Vec2 start = p->points[p->count - 1];
    for (int i = 0; i < p->count; ++i) {
        b2Vec2 end = p->points[i];
        b2Vec2 intersect;
        if (line_intersects(start, end, p1, p2, &intersect)) {
            b2Vec2 start2 = end;
            for (int j = i + 1; j < p->count; ++j) {
                b2Vec2 end2 = p->points[j];
                b2Vec2 intersect2;
                if (line_intersects(start2, end2, p1, p2, &intersect2)) {
                    if (point_eq(intersect, intersect2)) {
                        // Intersects were found in shared corner of two lines, keep looking
                        start2 = end2;
                        continue;
                    }
                    int l1 = i + p->count - j + 2;
                    int l2 = j - i + 2;

                    b2Vec2* v = alloca(l1 * sizeof(b2Vec2));
                    b2Vec2* v2 = alloca(l2 * sizeof(b2Vec2));

                    v2[0] = intersect;
                    for (int ix = 0; ix < i; ++ix) {
                        v[ix] = p->points[ix];
                    }
                    for (int ix = i; ix < j; ++ix) {
                        v2[ix - i + 1] = p->points[ix];
                    }
                    v2[j - i + 1] = intersect2; 
                    v[i] = intersect;
                    v[i + 1] = intersect2;
                    for (int ix = j; ix < p->count; ++ix) {
                        v[i + ix + 2 - j] = p->points[ix];
                    }

                    Polygon_free(p);

                    *b = Polygon_create(world, v2, l2, t);
                    *a = Polygon_create(world, v, l1, t);
                    if (*a == NULL || *b == NULL) {
                        return true;
                    }

                    b2Vec2 aPos = b2Body_GetPosition((*a)->body);
                    b2Vec2 bPos = b2Body_GetPosition((*b)->body);

                    b2Vec2 diff = {aPos.x - bPos.x, aPos.y - bPos.y};
                    diff = b2Normalize(diff);

                    const float FORCE_VAL = 10 * 50000.0f;
                    
                    b2Vec2 aForce = {diff.x * FORCE_VAL, diff.y * FORCE_VAL};
                    b2Vec2 bForce = {diff.x * -FORCE_VAL, diff.y * -FORCE_VAL};
                    b2Body_ApplyForceToCenter((*a)->body, aForce, true);
                    b2Body_ApplyForceToCenter((*b)->body, bForce, true);
                    return true;
                }
                start2 = end2;
            }

        }
        start = end;
    }
    return false;
}

// This is simplified version of b2ComputePolygonMass that works for more than B2_MAX_POLYGON_VERTICES points.
b2Vec2 get_center_of_mass( const b2Vec2* points, int count, float density) {
    b2Vec2 center = { 0.0f, 0.0f };
    float area = 0.0f;
    

    // Get a reference point for forming triangles.
    // Use the first vertex to reduce round-off errors.
    b2Vec2 r = points[0];

    const float inv3 = 1.0f / 3.0f;

    for ( int i = 1; i < count - 1; ++i) {
            // Triangle edges
            b2Vec2 e1 = b2Sub(points[i], r );
            b2Vec2 e2 = b2Sub(points[i + 1], r);

            float D = b2Cross( e1, e2 );

            float triangleArea = 0.5f * D;
            area += triangleArea;

            // Area weighted centroid, r at origin
            center = b2MulAdd( center, triangleArea * inv3, b2Add( e1, e2 ) );
    }
    // Center of mass, shift back from origin at r
    float invArea = 1.0f / area;
    center.x = center.x * invArea + r.x;
    center.y = center.y * invArea + r.y;
    return center;
}

int needed_verts(int n_points) {
    int count = n_points;
    while (count > B2_MAX_POLYGON_VERTICES) {
        n_points += 2;
        count -= B2_MAX_POLYGON_VERTICES - 2;
    }
    return n_points;
}


Polygon* Polygon_create(b2WorldId world, const b2Vec2* points, int n_points, b2Transform t) {
    b2Vec2 center = get_center_of_mass(points, n_points, 1.0f);
    Polygon* poly = SDL_malloc(sizeof(Polygon) + n_points * sizeof(b2Vec2));
    SDL_assert_release(poly != NULL);
    SDL_memcpy(poly->points, points, n_points * sizeof(b2Vec2));

    for (int i = 0; i < n_points; ++i) {
        poly->points[i].x -= center.x;
        poly->points[i].y -= center.y;
    }

    b2BodyDef bodyDef = b2DefaultBodyDef();
    bodyDef.type = b2_dynamicBody;
    bodyDef.position = t.p;
    b2Vec2 wold_center = b2RotateVector(t.q, center);

    bodyDef.position.x += wold_center.x;
    bodyDef.position.y += wold_center.y;
    bodyDef.rotation = t.q;
    b2BodyId body = b2CreateBody(world, &bodyDef);
    poly->count = n_points;

    int total_verts = 0;
    if (n_points <= B2_MAX_POLYGON_VERTICES) {
        b2Hull hull = b2ComputeHull(poly->points, n_points);
        total_verts += hull.count;
        if (hull.count != 0) {
            b2Polygon dynPoly = b2MakePolygon(&hull, 0.0f);
            b2ShapeDef shapeDef = b2DefaultShapeDef();
            shapeDef.density = 1.0f;
            b2CreatePolygonShape(body, &shapeDef, &dynPoly);
        }
    } else {
        int tot_verts = needed_verts(n_points);
        int parts = SDL_ceil((double)tot_verts / (double) B2_MAX_POLYGON_VERTICES);
        int verts_per = tot_verts / parts;
        int ix = 1;
        for (int i = 0; i < parts; ++i) {
            b2Vec2 verts[B2_MAX_POLYGON_VERTICES];
            verts[0] = poly->points[0];
            int part_count = verts_per;
            if (i == parts - 1) {
                part_count = tot_verts - verts_per * (parts - 1);
            }
            for (int j = 1; j < part_count; ++j) {
                verts[j] = poly->points[ix];
                ++ix;
            }
            --ix;
            b2Hull hull = b2ComputeHull(verts, part_count);
            total_verts += hull.count;
            if (hull.count != 0) {
                b2Polygon dynPoly = b2MakePolygon(&hull, 0.0f);
                b2ShapeDef shapeDef = b2DefaultShapeDef();
                shapeDef.density = 1.0f;
                b2CreatePolygonShape(body, &shapeDef, &dynPoly);
            }
        }
    }

    if (total_verts < n_points) {
        SDL_free(poly);
        b2DestroyBody(body);
        return NULL;
    }

    poly->body = body;

    return poly;
}

void Polygon_free(Polygon* p) {
    b2DestroyBody(p->body);

    SDL_free(p);
}


float area(const SDL_FPoint* point, int n)  {
    float sum = 0.0f;
    SDL_FPoint start = point[n -1];
    for (int i = 0; i < n; ++i) {
        SDL_FPoint end = point[i];
            sum += start.x * end.y - start.y * end.x;
        start = end;
    }
    return 0.5 * SDL_abs(sum);
}



/*bool contains_point(Polygon* poly, SDL_FPoint p) {
    if (poly->count < 3) {
        return false;
    }

    bool inside = false;
    for (int i = 0, j = poly->count - 1; i < poly->count; j = i++) {
        SDL_FPoint pi = poly->points[i];
        SDL_FPoint pj = poly->points[j];

        if (((pi.y > p.y) != (pj.y > p.y)) &&
            (p.x < (pj.x - pi.x) * (p.y - pi.y) / (pj.y - pi.y) + pi.x)) {
            inside = !inside;
        }
    }

    return inside;
}


float dot_prod(SDL_FPoint a, SDL_FPoint b) {
    return a.x * b.x + a.y * b.y;
}

SDL_FPoint normalized(SDL_FPoint p) {
    float len = SDL_sqrtf(p.x * p.x + p.y * p.y);

    if (len == 0) {
        p.x = 0.0f;
        p.y = 0.0f;
        return p;
    }
    p.x /= len;
    p.y /= len;
    return p;
}



struct PolygonProjection {
    float low;
    float high;
};

struct PolygonProjection project_poly(Polygon* p, SDL_FPoint normal) {
    float low = INFINITY;
    float high = -INFINITY;

    for (int i = 0; i < p->count; ++i) {
        float d = dot_prod(normal, p->points[i]);
        if (d > high) {
            high = d;
        }
        if (d < low) {
            low = d;
        }
    }
    struct PolygonProjection proj = {low, high};
    return proj;
}

bool check_collision2(Polygon* a, Polygon* b, SDL_FPoint* normal, float* min_penetration) {
    SDL_FPoint start = a->points[a->count - 1];
    for (int i = 0; i < a->count; ++i) {
        SDL_FPoint end = a->points[i];
        SDL_FPoint delta = {end.x - start.x, end.y - start.y};
        delta = normalized(delta);
        SDL_FPoint normal = {-delta.y, delta.x};
        struct PolygonProjection proj_a = project_poly(a, normal);
        struct PolygonProjection proj_b = project_poly(b, normal);   

        if (proj_a.high > proj_b.low && proj_b.high > proj_a.low) {

            start = end;
            continue;
        }
        return false;

    }
    return true;
}


bool Polygon_collision2(Polygon* a, Polygon* b, SDL_FPoint* colp, SDL_FPoint* delta, SDL_FPoint* normal) {
    float min_penetration = 99999999999999.0f;
    return check_collision2(a, b, normal, &min_penetration) && check_collision2(b, a, normal, &min_penetration);
}*/



/*void Polygon_solve_collision(Polygon *a, Polygon* b, SDL_FPoint p, SDL_FPoint shift, SDL_FPoint normal) {
    const double ELASTICITY = 0.5;
    Polygon_move(a, shift.x / 2.0, shift.y / 2.0);
    Polygon_move(b, -shift.x / 2.0, -shift.y / 2.0);
    SDL_FPoint center_a = a->center;
    SDL_FPoint ra = {p.x - center_a.x, p.y - center_a.y};

    SDL_FPoint center_b = b->center;
    SDL_FPoint rb = {p.x - center_b.x, p.y - center_b.y};

    SDL_FPoint vrel_vec = {a->vx - a->vrot * ra.y - (b->vx - b->vrot * rb.y),
                           a->vy + a->vrot * ra.x - (b->vy + b->vrot * rb.x)};
    float vrel = dot_prod(vrel_vec, normal);
    if (vrel > 0) {
        return;
    }

    float ra_x_n = ra.x * normal.y - ra.y * normal.x;
    float rb_x_n = rb.x * normal.y - rb.y * normal.x;

    float denom = 1.0 / a->mass + 1.0 / b->mass +
                  (ra_x_n * ra_x_n) / a->inertia +
                  (rb_x_n * rb_x_n) / b->inertia;

    float j = -(ELASTICITY + 1.0) * vrel / denom;
    SDL_FPoint vel_a = {a->vx + j * normal.x / a->mass, a->vy + j * normal.y / a->mass};
    SDL_FPoint vel_b = {b->vx - j * normal.x / b->mass, b->vy - j * normal.y / b->mass};

    a->vrot = a->vrot + j * ra_x_n / a->inertia;
    a->vx = vel_a.x;
    a->vy = vel_a.y;

    b->vrot = b->vrot - j * rb_x_n / b->inertia;
    b->vx = vel_b.x;
    b->vy = vel_b.y;
}
        const ELASTICITY: f64 = 0.5;

        self.shape.shift(shift.x, shift.y);
        let centre_a = self.shape.centre;
        let ra = p - centre_a;

        let centre_b = other.shape.centre;
        let rb = p - centre_b;

        let vrel = Point::new(self.dx - self.rot * ra.y -  (other.dx - other.rot * rb.y),
                              self.dy + self.rot * ra.x - (other.dy + other.rot * rb.x)).dot(normal);

        let ra_x_n = ra.x * normal.y - ra.y * normal.x;
        let rb_x_n = rb.x * normal.y - rb.y * normal.x;
        let cross_thing_a = Point::new(ra.y * -ra_x_n, ra.x * ra_x_n);
        let cross_thing_b = Point::new(rb.y * -rb_x_n, rb.x * rb_x_n);

        let norm_part = normal.dot(cross_thing_a / self.inertia + cross_thing_b / other.inertia);

        let j = -(ELASTICITY + 1.0) * vrel / (1.0 / self.mass + 1.0 / other.mass + norm_part);
        let vel_a = Point::new(self.dx, self.dy) + j * normal / self.mass;
        let vel_b = Point::new(other.dx, other.dy) - j * normal / other.mass;
        self.rot = self.rot + j * ra_x_n / self.inertia;
        self.dx = vel_a.x;
        self.dy = vel_a.y;

        other.rot = other.rot - j * rb_x_n / other.inertia;
        other.dx = vel_b.x;
        other.dy = vel_b.y;





*/

b2Vec2 rotated(b2Vec2 p, double rad, b2Vec2 center) {
    b2Vec2 pd = {p.x - center.x, p.y - center.y};

    double sina = SDL_sin(rad);
    double cosa = SDL_cos(rad);

    float x = pd.x * cosa - pd.y * sina;
    float y = pd.y * cosa + pd.x * sina;

    pd.x = x + center.x;
    pd.y = y + center.y;
    return pd;
}


/*float polygon_radius(const SDL_FPoint* points, int count, SDL_FPoint center) {
    float radius = 0.0;
    for (int i = 0; i < count; ++i) {
        float len_sqrd = (points[i].x - center.x) * (points[i].x - center.x) + 
                         (points[i].y - center.y) * (points[i].y - center.y);
        if (radius < len_sqrd) {
            radius = len_sqrd;
        }
    }

    return SDL_sqrtf(radius);
}

SDL_FPoint polygon_center(const SDL_FPoint* points, int count) {
    SDL_FPoint p = {0.0f, 0.0f};
    for (uint32_t ix = 0; ix < count; ++ix) {
        p.x += points[ix].x;
        p.y += points[ix].y;
    }

    p.x /= count;
    p.y /= count;
    
    return p;
}


void Polygon_move(Polygon* p, float dx, float dy) {
    for (int i = 0; i < p->count; ++i) {
        p->points[i].x += dx;
        p->points[i].y += dy;
    }
    p->center.x += dx;
    p->center.y += dy;
}

void Polygon_rotate(Polygon* poly, double rad) {
    double sina = SDL_sin(rad);
    double cosa = SDL_cos(rad);
    for (int i = 0; i < poly->count; ++i) {
        SDL_FPoint p = {poly->points[i].x - poly->center.x,
                        poly->points[i].y - poly->center.y};

        float x = p.x * cosa - p.y * sina;
        float y = p.y * cosa + p.x * sina;

        poly->points[i].x = x + poly->center.x;
        poly->points[i].y = y + poly->center.y;
    }
}

void solve_wall_collision(Polygon* pol, float dx, float dy, SDL_FPoint p, float normal_x, float normal_y) {
    SDL_FPoint normal = {normal_x, normal_y};
    const float ELASTICITY = 0.9f;
    Polygon_move(pol, dx, dy);

    p.x += dx;
    p.y += dy;

    SDL_FPoint center_a = pol->center;
    SDL_FPoint ra = {p.x - center_a.x, p.y - center_a.y};

    SDL_FPoint vrel_vec = {pol->vx - pol->vrot * ra.y,
                           pol->vy + pol->vrot * ra.x};
    float vrel = dot_prod(vrel_vec, normal);
    if (vrel > 0) {
        return;
    }

    float ra_x_n = ra.x * normal.y - ra.y * normal.x;

    float denom = 1.0 / pol->mass + (ra_x_n * ra_x_n) / pol->inertia;

    float j = -(ELASTICITY + 1.0) * vrel / denom;
    SDL_FPoint vel_a = {pol->vx + j * normal.x / pol->mass, pol->vy + j * normal.y / pol->mass};

    pol->vrot = pol->vrot + j * ra_x_n / pol->inertia;
    pol->vx = vel_a.x;
    pol->vy = vel_a.y;
}

void Polygon_resolve_walls(Polygon* pol, float max_x, float max_y) {
    for (int i = 0; i < pol->count; ++i) {
        if (pol->points[i].x < 0) {
            solve_wall_collision(pol, -pol->points[i].x, 0.0, pol->points[i], 1.0, 0.0);
        }
        if (pol->points[i].x >= max_x) {
            solve_wall_collision(pol, max_x - pol->points[i].x, 0.0, pol->points[i], 1.0, 0.0);
        }
        if (pol->points[i].y < 0) {
            solve_wall_collision(pol, 0.0, -pol->points[i].y, pol->points[i], 0.0, 1.0);
        } 
        if (pol->points[i].y >= max_y) {
            solve_wall_collision(pol, 0.0, max_y - pol->points[i].y, pol->points[i], 0.0, -.0);
        }
    }
}*/
