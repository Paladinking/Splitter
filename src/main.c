#define SDL_ASSERT_LEVEL 2

#include <SDL3/SDL.h>
#include <SDL3_image/SDL_image.h>
#include <SDL3_ttf/SDL_ttf.h>
#include <box2d/box2d.h>
#include <stdio.h>

#include "polygon.h"

SDL_Renderer* gRenderer = NULL;


void render_polygon(Polygon* poly, SDL_Color color) {
    SDL_assert(poly->count >= 3);
    SDL_Vertex* verts = alloca(poly->count * sizeof(SDL_Vertex));
    int* indicies = alloca(3 * (poly->count - 2) * sizeof(int));

    b2Transform t = b2Body_GetTransform(poly->body);

    for (uint32_t ix = 0; ix < poly->count; ++ix) {
        verts[ix].color.r = color.r / 255.0f;
        verts[ix].color.g = color.g / 255.0f;
        verts[ix].color.b = color.b / 255.0f;
        verts[ix].color.a = color.a / 255.0f;

        b2Vec2 v = b2TransformPoint(t, poly->points[ix]);
        verts[ix].position.x = v.x * 10.0f;
        verts[ix].position.y = v.y * 10.0f;
    }

    for (uint32_t ix = 1; ix < poly->count - 1; ++ix) {
        indicies[(ix - 1) * 3] = 0;
        indicies[(ix - 1) * 3 + 1] = ix;
        indicies[(ix - 1) * 3 + 2] = ix + 1;
    }
    SDL_RenderGeometry(gRenderer, NULL, verts, poly->count, indicies, 3 * (poly->count - 2));
}

void create_bounds(b2WorldId world) {
    b2BodyDef groundBodyDef = b2DefaultBodyDef();
    groundBodyDef.type = b2_staticBody;
    groundBodyDef.position = (b2Vec2){40.0f, 40.0f};
    b2BodyId groundId = b2CreateBody(world, &groundBodyDef);

    b2Polygon groundBox = b2MakeBox(40.0f, 5.0f);
    for (int i = 0; i < groundBox.count; ++i) {
        groundBox.vertices[i].y += 45.0f;
    }
    b2ShapeDef groundShapeDef = b2DefaultShapeDef();
    groundShapeDef.material.restitution = 0.9f;
    b2CreatePolygonShape(groundId, &groundShapeDef, &groundBox);

    groundBox = b2MakeBox(40.0f, 5.0f);
    for (int i = 0; i < groundBox.count; ++i) {
        groundBox.vertices[i].y -= 45.0f;
    }
    groundShapeDef = b2DefaultShapeDef();
    groundShapeDef.material.restitution = 0.9f;
    b2CreatePolygonShape(groundId, &groundShapeDef, &groundBox);

    groundBox = b2MakeBox(5.0f, 40.0f);
    for (int i = 0; i < groundBox.count; ++i) {
        groundBox.vertices[i].x += 45.0f;
    }
    groundShapeDef = b2DefaultShapeDef();
    groundShapeDef.material.restitution = 0.9f;
    b2CreatePolygonShape(groundId, &groundShapeDef, &groundBox);

    groundBox = b2MakeBox(5.0f, 40.0f);
    for (int i = 0; i < groundBox.count; ++i) {
        groundBox.vertices[i].x -= 45.0f;
    }
    groundShapeDef = b2DefaultShapeDef();
    groundShapeDef.material.restitution = 0.9f;
    b2CreatePolygonShape(groundId, &groundShapeDef, &groundBox);
}

void draw_solid_polygon(b2Transform t, const b2Vec2* vertices, int count, float radius, b2HexColor color, void* context ) {
    SDL_Vertex* verts = alloca(count * sizeof(SDL_Vertex));
    int* indicies = alloca(3 * (count - 2) * sizeof(int));

    for (uint32_t ix = 0; ix < count; ++ix) {
        verts[ix].color.r = 230 / 255.0f;
        verts[ix].color.g = 110 / 255.0f;
        verts[ix].color.b = 110 / 255.0f;
        verts[ix].color.a = 255 / 255.0f;

        b2Vec2 v = b2TransformPoint(t, vertices[ix]);
        verts[ix].position.x = v.x * 10.0f;
        verts[ix].position.y = v.y * 10.0f;
    }

    for (uint32_t ix = 1; ix < count - 1; ++ix) {
        indicies[(ix - 1) * 3] = 0;
        indicies[(ix - 1) * 3 + 1] = ix;
        indicies[(ix - 1) * 3 + 2] = ix + 1;
    }
    SDL_RenderGeometry(gRenderer, NULL, verts, count, indicies, 3 * (count - 2));
}

int main() {
    SDL_Init(SDL_INIT_VIDEO);

    SDL_Window* window;
    SDL_Renderer* renderer;

    if (!SDL_CreateWindowAndRenderer("Polygon", 800, 800, 0, &window, &renderer)) {
        SDL_LogCritical(SDL_LOG_CATEGORY_APPLICATION, "Failed creating window: %s\n", SDL_GetError());
    }

    SDL_Texture* player_tex = IMG_LoadTexture(renderer, "assets/player.png");



    b2WorldDef worldDef = b2DefaultWorldDef();
    worldDef.gravity.y = 0.0f;
    b2WorldId world = b2CreateWorld(&worldDef);
    create_bounds(world);
    b2DebugDraw draw = b2DefaultDebugDraw();
    draw.drawShapes = true;
    draw.DrawSolidPolygonFcn = draw_solid_polygon;

    gRenderer = renderer;

    bool running = true;

    b2Vec2 player = {40.0f, 40.0f};


    b2Vec2 points[5] = {{1.0f, 1.0f}, {1.0f, 20.0f}, {20.0f, 29.0f}, {45.0f, 10.0f}, {20.0f, 2.5f}};
    b2Transform t = {{60.0f, 60.0f}, {-1.0f, 0.0f}};
    Polygon* poly = Polygon_create(world, points, 5, t);

    b2Vec2 points2[4] = {{20.0f, 29.0f}, {20.0f, 2.5f}, {35.0f, 10.0f}, {40.0f, 20.0f}};
    t.p.x += 20.0f;
    t.p.y -= 25.0f;

    Polygon** polygons = SDL_malloc(200 * sizeof(Polygon*));
    int poly_count = 2;
    polygons[0] = poly;
    poly = Polygon_create(world, points2, 4, t);
    polygons[1] = poly;


    uint64_t stamp = SDL_GetTicks();

    double line_rot = 1.2;

    uint64_t last_click = 0;
    bool click = false;

    bool left = false, right = false, up = false, down = false;

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
                    } else if (e.key.key == SDLK_W) {
                        up = true;
                    } else if (e.key.key == SDLK_S) {
                        down = true;
                    } else if (e.key.key == SDLK_A) {
                        left = true;
                    } else if (e.key.key == SDLK_D) {
                        right = true;
                    }
                    break;
                case SDL_EVENT_KEY_UP:
                    if (e.key.key == SDLK_W) {
                        up = false;
                    } else if (e.key.key == SDLK_S) {
                        down = false;
                    } else if (e.key.key == SDLK_A) {
                        left = false;
                    } else if (e.key.key == SDLK_D) {
                        right = false;
                    }
                    break;
                case SDL_EVENT_MOUSE_WHEEL:
                    line_rot += e.wheel.y * 0.1;
                    break;
                case SDL_EVENT_MOUSE_BUTTON_DOWN:
                    if (e.button.button == SDL_BUTTON_LEFT) {
                        uint64_t now = SDL_GetTicks();
                        if (now - last_click > 1000) {
                            last_click = now;
                            click = true;
                        }
                    }
                    break;

            }
        }
        b2Vec2 vel = {0.0f, 0.0f};
        if (up) {
            vel.y -= 1.0f;
        } 
        if (down) {
            vel.y += 1.0f;
        }
        if (left) {
            vel.x -= 1.0f;
        } 
        if (right) {
            vel.x += 1.0f;
        }
        vel = b2Normalize(vel);

        uint64_t now = SDL_GetTicks();
        double delta = (now - stamp) / 1000.0;
        stamp = now;

        player.x += vel.x * delta * 25.0f;
        player.y += vel.y * delta * 25.0f;

        float mx, my;
        SDL_GetMouseState(&mx, &my);
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
        b2Vec2 p1, p2;

        p1.x = mx - 1200;
        p2.x = mx + 1200;
        p1.y = my;
        p2.y = my;

        b2Vec2 center = {mx, my};
        p1 = rotated(p1, line_rot, center);
        p2 = rotated(p2, line_rot, center);
        p1.x = p1.x / 10.0f;
        p1.y = p1.y / 10.0f;
        p2.x = p2.x / 10.0f;
        p2.y = p2.y / 10.0f;

        if (click) {
            click = false;
            printf("click\n");
            uint32_t cur_poly_count = poly_count;
            for (uint32_t ix = 0; ix < cur_poly_count; ++ix) {
                Polygon* a, *b;
                if (Polygon_split(polygons[ix], world, p1, p2, &a, &b)) {
                    polygons[ix] = a;
                    polygons[poly_count] = b;
                    ++poly_count;
                    printf("Split\n");
                }
            }

            for (uint32_t ix = 0; ix < poly_count;) {
                if (polygons[ix] == NULL) {
                    polygons[ix] = polygons[poly_count - 1];
                    --poly_count;
                } else {
                    ++ix;
                }
            }
        }


        bool collision = false;


        b2World_Step(world, delta, 4);

        SDL_SetRenderDrawColor(gRenderer, 0x10, 0x10, 0x10, 0xff);
        SDL_RenderClear(gRenderer);


        SDL_Color color = {150, 150, 150, 255};
        if (collision) {
            color.b = 200;
        }

        for (int i = 0; i < poly_count; ++i) {
            color.r += 10;
            color.b += 25;
            render_polygon(polygons[i], color);
        }

        //b2World_Draw(world, &draw);
        
        SDL_FRect dest = {player.x * 10.0f - 20.0f, player.y * 10.0f - 20.0f, 40.0f, 40.0f};
        SDL_RenderTexture(gRenderer, player_tex, NULL, &dest);
        
        SDL_SetRenderDrawColor(gRenderer, 0x80, 0x10, 0x10, 0xff);
        SDL_RenderLine(gRenderer, p1.x * 10.0f, p1.y * 10.0f, p2.x * 10.0f, p2.y * 10.0f);
        

        SDL_RenderPresent(renderer);
    }

    b2DestroyWorld(world);
    
    SDL_DestroyTexture(player_tex);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    
    SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION, "Shutting down...");

    TTF_Quit();
    SDL_Quit();

    return 0;
}
