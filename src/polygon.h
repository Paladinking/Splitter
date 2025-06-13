#ifndef POLYGON_H
#define POLYGON_H
#include <SDL3/SDL.h>
#include <box2d/box2d.h>

typedef struct Polygon {
    b2BodyId body;
    int count;
    b2Vec2 points[];
} Polygon;

extern Polygon* Polygon_create(b2WorldId world, const b2Vec2* points, int n_points, b2Transform t);

//extern void Polygon_move(Polygon* p, float dx, float dy);

extern bool Polygon_split(Polygon* p, b2WorldId world, b2Vec2 p1, b2Vec2 p2, Polygon** a, Polygon** b);

//extern void Polygon_rotate(Polygon* poly, double rad);

extern void Polygon_free(Polygon* poly);

extern b2Vec2 rotated(b2Vec2 p, double rad, b2Vec2 center);

//extern bool Polygon_collision(Polygon* a, Polygon* b, SDL_FPoint* colp, SDL_FPoint* delta, SDL_FPoint* normal);

//extern bool Polygon_collision2(Polygon* a, Polygon* b, SDL_FPoint* colp, SDL_FPoint* delta, SDL_FPoint* normal);

//extern void Polygon_solve_collision(Polygon *a, Polygon* b, SDL_FPoint p, SDL_FPoint shift, SDL_FPoint normal);

//extern void Polygon_resolve_walls(Polygon* pol, float max_x, float max_y);

#endif
