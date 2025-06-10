#ifndef POLYGON_H
#define POLYGON_H
#include <SDL3/SDL.h>

typedef struct Polygon {
    SDL_FPoint center;
    int count;
    float radius;
    float vx;
    float vy;
    float vrot;
    float mass;
    float inertia;
    SDL_FPoint points[];
} Polygon;

extern Polygon* Polygon_create(const SDL_FPoint* points, int n_points);

extern void Polygon_move(Polygon* p, float dx, float dy);

extern bool Polygon_split(Polygon* p, SDL_FPoint p1, SDL_FPoint p2, Polygon** a, Polygon** b);

extern void Polygon_rotate(Polygon* poly, double rad);

extern void Polygon_free(Polygon* poly);

extern SDL_FPoint rotated(SDL_FPoint p, double rad, SDL_FPoint center);

extern bool Polygon_collision(Polygon* a, Polygon* b, SDL_FPoint* colp, SDL_FPoint* delta, SDL_FPoint* normal);

extern void Polygon_solve_collision(Polygon *a, Polygon* b, SDL_FPoint p, SDL_FPoint shift, SDL_FPoint normal);

extern void Polygon_resolve_walls(Polygon* pol, float max_x, float max_y);

#endif
