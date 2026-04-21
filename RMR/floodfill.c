#include "floodfill.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int Xstart = 0;
int Ystart = 0;
int Xfinal = 0;
int Yfinal = 0;

static char mapData[MAP_HEIGHT][MAP_WIDTH];
static int  floodData[MAP_HEIGHT][MAP_WIDTH]; 

typedef struct {
    int x;
    int y;
} Point;

static Point queue[MAP_WIDTH * MAP_HEIGHT];
static int head = 0;
static int tail = 0;

static void queue_push(int x, int y) {
    if (tail < (MAP_WIDTH * MAP_HEIGHT)) {
        queue[tail].x = x;
        queue[tail].y = y;
        tail++;
    }
}

static Point queue_pop() {
    Point p = queue[head++];
    return p;
}

static int is_queue_empty() {
    return head == tail;
}

static int do_floodfill(const char* map_filename) {
    FILE* file = fopen(map_filename, "r");
    if (!file) {
        printf("Error: Could not open map file %s\n", map_filename);
        return 0;
    }

    for (int y = 0; y < MAP_HEIGHT; y++) {
        for (int x = 0; x < MAP_WIDTH; x++) {
            int ch = fgetc(file);
            while (ch == '\n' || ch == '\r') {
                ch = fgetc(file); 
            }
            if (ch == EOF) break;
            
            mapData[y][x] = (char)ch;
            floodData[y][x] = -1;
        }
    }
    fclose(file);

    if (Xfinal >= 0 && Xfinal < MAP_WIDTH && Yfinal >= 0 && Yfinal < MAP_HEIGHT) {
        if (mapData[Yfinal][Xfinal] == '0') {
            printf("Target destination is on an empty space ('0'). Returning 1.\n");
        } else {
            printf("Target destination is NOT an empty space ('0'). Returning 0.\n");
            return 0;
        }
    } else {
        printf("Target coordinates are out of bounds. Returning 0.\n");
        return 0;
    }

    if (Xstart >= 0 && Xstart < MAP_WIDTH && Ystart >= 0 && Ystart < MAP_HEIGHT) {
        mapData[Ystart][Xstart] = 'S';
    }

    head = 0;
    tail = 0;

    floodData[Yfinal][Xfinal] = 1;
    queue_push(Xfinal, Yfinal);

    int dx[] = { 0,  0, -1,  1};
    int dy[] = {-1,  1,  0,  0};

    while (!is_queue_empty()) {
        Point current = queue_pop();

        if (current.x == Xstart && current.y == Ystart) {
            break;
        }

        for (int i = 0; i < 4; i++) {
            int nx = current.x + dx[i];
            int ny = current.y + dy[i];

            if (nx >= 0 && nx < MAP_WIDTH && ny >= 0 && ny < MAP_HEIGHT) {
                if ((mapData[ny][nx] == '0' || mapData[ny][nx] == 'S') && floodData[ny][nx] == -1) {
                    floodData[ny][nx] = floodData[current.y][current.x] + 1;
                    queue_push(nx, ny);
                }
            }
        }
    }
    
    return 1;
}

static void find_path(float* x_target_position, float* y_target_position, int* num_targets, int max_targets) {
    *num_targets = 0;
    if (floodData[Ystart][Xstart] == -1) {
        printf("No path found!\n");
        return;
    }

    int current_x = Xstart;
    int current_y = Ystart;
    int last_dx = 0;
    int last_dy = 0;

    int dx[] = { 0,  0, -1,  1};
    int dy[] = {-1,  1,  0,  0};
    
    while ((current_x != Xfinal || current_y != Yfinal) && *num_targets < max_targets - 1) {
        int current_dist = floodData[current_y][current_x];
        int next_x = -1, next_y = -1;
        int next_dx = 0, next_dy = 0;
        
        for (int i = 0; i < 4; i++) {
            int nx = current_x + dx[i];
            int ny = current_y + dy[i];
            
            if (nx >= 0 && nx < MAP_WIDTH && ny >= 0 && ny < MAP_HEIGHT) {
                if (floodData[ny][nx] == current_dist - 1) {
                    next_x = nx;
                    next_y = ny;
                    next_dx = dx[i];
                    next_dy = dy[i];
                    break;
                }
            }
        }
        
        if (next_x == -1) {
            printf("Error backtracking path!\n");
            break;
        }
        
        if ((last_dx != 0 || last_dy != 0) && (last_dx != next_dx || last_dy != next_dy)) {
            x_target_position[*num_targets] = (float)current_x;
            y_target_position[*num_targets] = (float)current_y;
            (*num_targets)++;
        }
        
        last_dx = next_dx;
        last_dy = next_dy;
        current_x = next_x;
        current_y = next_y;
    }
    
    if (*num_targets < max_targets) {
        x_target_position[*num_targets] = (float)Xfinal;
        y_target_position[*num_targets] = (float)Yfinal;
        (*num_targets)++;
    }

    for (int i = *num_targets; i < max_targets; i++) {
        x_target_position[i] = (float)Xfinal;
        y_target_position[i] = (float)Yfinal;
    }
}

int run_floodfill(const char* map_filename, float* x_target_position, float* y_target_position, int* num_targets) {
    if (do_floodfill(map_filename)) {
        // Assuming 500 is the size defined in robot.h
        find_path(x_target_position, y_target_position, num_targets, 500);
        return 1;
    }
    *num_targets = 0;
    return 0;
}