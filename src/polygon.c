#include "polygon.h"
#include <stdio.h>

    /*pub fn get_collision(&self, other: &Polygon) -> Option<(Point, Point, Point)> {
        if let Some(res) = self.check_collision(other) {
            return Some(res);
        }
        if let Some((col, offset, normal)) = other.check_collision(self) {
            return Some((col, -offset, normal));
        }
        return None;
    }*/
    
    /*
        for p in &other.points {
            if self.contains_point(*p) {
                let centre = other.centre;
                if let Some((col, normal)) = self.get_intersect(*p, centre) {
                    return Some((col, col -*p, normal));
                } else {
                    return None;
                }
            }
        }
        return None;

     * */

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


bool line_intersects(SDL_FPoint pa1, SDL_FPoint pa2, SDL_FPoint pb1, SDL_FPoint pb2, SDL_FPoint* intersect) {
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

bool contains_point(Polygon* poly, SDL_FPoint p) {
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

// point and normal
// p2 should be iside, p1 outside
bool get_intersect(Polygon* p, SDL_FPoint p1, SDL_FPoint p2, SDL_FPoint* point, SDL_FPoint* normal) {
    SDL_FPoint p3 = p->points[p->count - 1];
    for (int i = 0; i < p->count; ++i) {
        SDL_FPoint p4 = p->points[i];
        if (line_intersects(p1, p2, p3, p4, point)) {
            float dx = p4.x - p3.x;
            float dy = p4.y - p3.y;
            SDL_FPoint rotated = {-dy, dx};
            SDL_FPoint out = {p1.x - point->x, p1.y - point->y};
        
            float scale = dot_prod(rotated, out) / dot_prod(rotated, rotated);

            normal->x = scale * rotated.x;
            normal->y = scale * rotated.y;
            *normal = normalized(*normal);

            return true;
        }
        p3 = p4;
    }
    return false;
}


bool check_collision(Polygon* a, Polygon* b, SDL_FPoint* colp, SDL_FPoint* delta, SDL_FPoint* normal) {
    for (int i = 0; i < b->count; ++i) {
        if (contains_point(a, b->points[i])) {
            SDL_FPoint center = b->center;
            if (get_intersect(a, b->points[i], center, colp, normal)) {
                delta->x = colp->x - b->points[i].x;
                delta->y = colp->y - b->points[i].y;
                return true;
            }
            return false;
        }

    }
    return false;
}

bool Polygon_collision(Polygon* a, Polygon* b, SDL_FPoint* colp, SDL_FPoint* delta, SDL_FPoint* normal) {
    float dx = a->center.x - b->center.x;
    float dy = a->center.y - b->center.y;
    float rad = a->radius + b->radius;
    if (dx * dx + dy * dy > rad * rad) {
        return false;
    }
    if (check_collision(a, b, colp, delta, normal)) {
        delta->x = -delta->x;
        delta->y = -delta->y;
        return true;
    }
    if (check_collision(b, a, colp, delta, normal)) {
        return true;
    }
    return false;
}


void Polygon_solve_collision(Polygon *a, Polygon* b, SDL_FPoint p, SDL_FPoint shift, SDL_FPoint normal) {
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
/*
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
        other.dy = vel_b.y;*/




bool point_eq(SDL_FPoint p1, SDL_FPoint p2) {
    return SDL_abs(p1.x - p2.x) + SDL_abs(p1.y - p2.y) < 0.01;
}


// Split polygon by line from p1 to p2
bool Polygon_split(Polygon* p, SDL_FPoint p1, SDL_FPoint p2, Polygon** a, Polygon** b) {
    SDL_FPoint start = p->points[p->count - 1];
    for (int i = 0; i < p->count; ++i) {
        SDL_FPoint end = p->points[i];
        SDL_FPoint intersect;
        if (line_intersects(start, end, p1, p2, &intersect)) {
            SDL_FPoint start2 = end;
            for (int j = i + 1; j < p->count; ++j) {
                SDL_FPoint end2 = p->points[j];
                SDL_FPoint intersect2;
                if (line_intersects(start2, end2, p1, p2, &intersect2)) {
                    if (point_eq(intersect, intersect2)) {
                        // Intersects were found in shared corner of two lines, keep looking
                        start2 = end2;
                        continue;
                    }
                    int l1 = i + p->count - j + 2;
                    int l2 = j - i + 2;

                    SDL_FPoint* v = alloca(l1 * sizeof(SDL_FPoint));
                    SDL_FPoint* v2 = alloca(l2 * sizeof(SDL_FPoint));

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

                     *a = Polygon_create(v, l1);
                     *b = Polygon_create(v2, l2);

                     SDL_FPoint diff = {(*a)->center.x - (*b)->center.x,
                                        (*a)->center.y - (*b)->center.y};
                     diff = normalized(diff);
                     (*a)->vx = p->vx + 50.0f * diff.x;
                     (*a)->vy = p->vy + 50.0f * diff.y;
                     (*b)->vy = p->vx - 50.0f * diff.x;
                     (*b)->vy = p->vy - 50.0f * diff.y;
                     return true;
                }
                start2 = end2;
            }

        }
        start = end;
    }
    return false;
}


SDL_FPoint rotated(SDL_FPoint p, double rad, SDL_FPoint center) {
    SDL_FPoint pd = {p.x - center.x, p.y - center.y};

    double sina = SDL_sin(rad);
    double cosa = SDL_cos(rad);

    float x = pd.x * cosa - pd.y * sina;
    float y = pd.y * cosa + pd.x * sina;

    pd.x = x + center.x;
    pd.y = y + center.y;
    return pd;
}


float polygon_radius(const SDL_FPoint* points, int count, SDL_FPoint center) {
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

Polygon* Polygon_create(const SDL_FPoint* points, int n_points) {
    Polygon* poly = SDL_malloc(sizeof(Polygon) + n_points * sizeof(SDL_FPoint));
    SDL_assert_release(poly != NULL);


    SDL_memcpy(poly->points, points, n_points * sizeof(SDL_FPoint));
    poly->count = n_points;
    poly->center = polygon_center(points, n_points);
    poly->radius = polygon_radius(points, n_points, poly->center);
    poly->mass = area(points, n_points);
    poly->inertia = poly->mass * SDL_sqrtf(poly->mass) * 50.0f;
    poly->vx = 0.0f;
    poly->vy = 0.0f;
    poly->vrot = 0.0f;

    return poly;
}

void Polygon_free(Polygon* p) {
    SDL_free(p);
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
}
