#define SDL_ASSERT_LEVEL 2

#include <SDL3/SDL.h>
#include <SDL3_image/SDL_image.h>
#include <SDL3_ttf/SDL_ttf.h>
#include <SDL3_gfx/SDL3_gfxPrimitives.h>
#include <stdio.h>

#include "polygon.h"

SDL_Renderer* gRenderer = NULL;



void render_polygon(Polygon* poly, SDL_Color color) {
    Sint16* vx = alloca(poly->count * sizeof(Sint16));
    Sint16* vy = alloca(poly->count * sizeof(Sint16));

    for (int i = 0; i < poly->count; ++i) {
        vx[i] = poly->points[i].x;
        vy[i] = poly->points[i].y;
    }

    filledPolygonRGBA(gRenderer, vx, vy, poly->count, color.r, color.g, color.b, color.a);
    return;

    SDL_assert(poly->count >= 3);
    SDL_Vertex* verts = alloca(poly->count * sizeof(SDL_Vertex));
    int* indicies = alloca(3 * (poly->count - 2) * sizeof(int));

    for (uint32_t ix = 0; ix < poly->count; ++ix) {
        verts[ix].color.r = color.r / 255.0;
        verts[ix].color.g = color.g / 255.0;
        verts[ix].color.b = color.b / 255.0;
        verts[ix].color.a = color.a / 255.0;
        verts[ix].position = poly->points[ix];
    }

    for (uint32_t ix = 1; ix < poly->count - 1; ++ix) {
        indicies[(ix - 1) * 3] = 0;
        indicies[(ix - 1) * 3 + 1] = ix;
        indicies[(ix - 1) * 3 + 2] = ix + 1;
    }
    SDL_RenderGeometry(gRenderer, NULL, verts, poly->count, indicies, 3 * (poly->count - 2));
    return;
}


int main() {
    SDL_Init(SDL_INIT_VIDEO);
    TTF_Init();

    SDL_Window* window;
    SDL_Renderer* renderer;

    if (!SDL_CreateWindowAndRenderer("Polygon", 800, 800, 0, &window, &renderer)) {
        SDL_LogCritical(SDL_LOG_CATEGORY_APPLICATION, "Failed creating window: %s\n", SDL_GetError());
    }

    gRenderer = renderer;

    bool running = true;

    SDL_FPoint points[5] = {{10.0f, 10.0f}, {10.0f, 200.0f}, {200.0f, 290.0f}, {450.0f, 100.0f}, {200.0f, 25.0f}};
    Polygon* poly = Polygon_create(points, 5);
    Polygon_move(poly, 100, 50);

    Polygon** polygons = SDL_malloc(400 * sizeof(Polygon*));
    polygons[0] = poly;
    int poly_count = 1;


    SDL_FPoint points2[4] = {{10.0f, 200.0f}, {200.0f, 290.0f}, {450.0f, 100.0f}, {200.0f, 25.0f}};
    poly = Polygon_create(points2, 4);
    Polygon_move(poly, 0, 300);
    polygons[1] = poly;
    ++poly_count;

    Polygon* a, *b;
    SDL_FPoint p1 = {0.0, 0.0}, p2 = {500.0f, 600.0f};
    
    uint64_t stamp = SDL_GetTicks();

    double line_rot = 1.2;

    uint64_t last_click = 0;
    bool click = false;

    while (running) {
        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            switch (e.type) {
                case SDL_EVENT_QUIT:
                    running = false;
                    break;
                case SDL_EVENT_KEY_DOWN:
                    if (e.key.key == SDLK_ESCAPE) {
                        running = false;
                    }
                    break;
                case SDL_EVENT_MOUSE_WHEEL:
                    line_rot += e.wheel.y * 0.1;
                    break;
                case SDL_EVENT_MOUSE_BUTTON_DOWN:
                    if (e.button.button == SDL_BUTTON_LEFT) {
                        SDL_FPoint pos, delta, norm;
                        if (Polygon_collision(polygons[0], polygons[1], &pos, &delta, &norm)) {
                            printf("Collision: (%f, %f), (%f, %f), (%f, %f)\n", pos.x, pos.y, delta.x, delta.y, norm.x, norm.y);
                        }

                        uint64_t now = SDL_GetTicks();
                        if (now - last_click > 1000) {
                            last_click = now;
                            click = true;
                        }
                    }
                    break;

            }
        }
        uint64_t now = SDL_GetTicks();
        double delta = (now - stamp) / 1000.0;
        stamp = now;

        for (int i = 0; i < poly_count; ++i) {
            //Polygon_rotate(polygons[i], delta);
            Polygon_move(polygons[i], polygons[i]->vx * delta, polygons[i]->vy * delta);
            Polygon_rotate(polygons[i], polygons[i]->vrot * delta);
        }

        float mx, my;
        SDL_MouseButtonFlags mbutton = SDL_GetMouseState(&mx, &my);
        if (mx < 0) {
            mx = 0;
        } else if (mx > 800) {
            mx = 800;
        }
        if (my < 0) {
            my = 0;
        } else if (my > 800) {
            my = 800;
        }

        p1.x = mx - 1200;
        p2.x = mx + 1200;
        p1.y = my;
        p2.y = my;

        //Polygon_move(polygons[0], mx - polygons[0]->center.x, my - polygons[0]->center.y);

        SDL_FPoint center = {mx, my};
        p1 = rotated(p1, line_rot, center);
        p2 = rotated(p2, line_rot, center);

        if (click) {
            click = false;
            printf("click\n");
            uint32_t cur_poly_count = poly_count;
            for (uint32_t ix = 0; ix < cur_poly_count; ++ix) {
                Polygon* a, *b;
                if (Polygon_split(polygons[ix], p1, p2, &a, &b)) {
                    Polygon_free(polygons[ix]);
                    polygons[ix] = a;
                    polygons[poly_count] = b;
                    ++poly_count;
                    printf("Split\n");
                }
            }

            for (uint32_t ix = 0; ix < poly_count; ++ix) {
                if (polygons[ix]->mass < 100) {
                    Polygon_free(polygons[ix]);
                    polygons[ix] = polygons[poly_count - 1];
                    --poly_count;
                    --ix;
                }
            }
        }


        bool collision = false;

        for (int i = 0; i < poly_count; ++i) {
            for (int j = i + 1; j < poly_count; ++j) {
                SDL_FPoint col, delta, norm;
                if (Polygon_collision(polygons[i], polygons[j], &col, &delta, &norm)) {
                    collision = true;
                    printf("Collision\n");
                    Polygon_solve_collision(polygons[i], polygons[j], col, delta, norm);
                }
            }
        }

        for (int i = 0; i < poly_count; ++i) {
            Polygon_resolve_walls(polygons[i], 800, 800);
        }


        SDL_SetRenderDrawColor(gRenderer, 0x10, 0x10, 0x10, 0xff);
        SDL_RenderClear(gRenderer);


        SDL_Color color = {150, 150, 150, 255};
        if (collision) {
            color.b = 200;
        }
        for (int i = 0; i < poly_count; ++i) {
            render_polygon(polygons[i], color);
            color.r += 40;
        }
        

        
        SDL_SetRenderDrawColor(gRenderer, 0x80, 0x10, 0x10, 0xff);
        SDL_RenderLine(gRenderer, p1.x, p1.y, p2.x, p2.y);
        


       //SDL_RenderGeometry(renderer, NULL, verts, 4, indicies, 6);

        SDL_RenderPresent(renderer);
    }
    
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    
    SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION, "Shutting down...");

    TTF_Quit();
    SDL_Quit();

    return 0;
}
