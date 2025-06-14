#include <SDL3/SDL.h>
#include <box2d/box2d.h>

SDL_Renderer* gRenderer;


void render_polygon(b2Vec2* points, int count, SDL_Color color, b2Transform t) {
    SDL_assert(count >= 3);
    SDL_Vertex* verts = alloca(count * sizeof(SDL_Vertex));
    int* indicies = alloca(3 * (count - 2) * sizeof(int));

    for (uint32_t ix = 0; ix < count; ++ix) {
        verts[ix].color.r = color.r / 255.0f;
        verts[ix].color.g = color.g / 255.0f;
        verts[ix].color.b = color.b / 255.0f;
        verts[ix].color.a = color.a / 255.0f;

        b2Vec2 v = b2TransformPoint(t, points[ix]);
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

    gRenderer = renderer;

    bool running = true;

    //b2Vec2 points[5] = {{1.0f, 1.0f}, {1.0f, 20.0f}, {20.0f, 29.0f}, {45.0f, 10.0f}, {20.0f, 2.5f}};
    b2Vec2 points2[] = {
        {.x = -15.1954899, .y = -12.4078617},
        {.x = 15.5248156, .y = 6.68470621},
        {.x = 16.8995457, .y = 0.781459987},
        {.x = 16.0796413, .y = -3.97224855},
        {.x = 14.1255598, .y = -7.81154633},
        {.x = 3.80451012, .y = -10.9078617}
    };
    b2Vec2 points[] = {
        {.x = -15.1954899, .y = -12.4078617},
        {.x = -15.1954899, .y = 6.59213829},
        {.x = -4.11816454, .y = 11.8392925},
        {.x = 7.20963955,  .y = 13.00424},
        {.x = 15.5248156,  .y = 6.68470621}
    };
    int count = sizeof(points) / sizeof(b2Vec2);
    int count2 = sizeof(points2) / sizeof(b2Vec2);
    //b2Transform t = {{60.0f, 60.0f}, {-1.0f, 0.0f}};
    b2Transform t = {.p = {.x = 50.0619125, .y = 44.5697632}, .q = {.c = -0.996549964, .s = -0.0829958841}};


    uint64_t stamp = SDL_GetTicks();

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

            }
        }

        uint64_t now = SDL_GetTicks();
        double delta = (now - stamp) / 1000.0;
        stamp = now;

        SDL_SetRenderDrawColor(gRenderer, 0x10, 0x10, 0x10, 0xff);
        SDL_RenderClear(gRenderer);


        SDL_Color color = {150, 150, 150, 255};


        render_polygon(points2, count2, color, t);
        color.r = 255;
        render_polygon(points, count, color, t);
        

        SDL_RenderPresent(renderer);
    }

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    
    SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION, "Shutting down...");

    SDL_Quit();

    return 0;
}
