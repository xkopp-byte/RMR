#include "floodfill.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int Xstart = 210;
int Ystart = 50;
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

static void restore_map() {
    FILE* src = fopen("maps/finalMap_copy.txt", "r");
    if (!src) {
        printf("Error: Could not open source map file maps/finalMap_copy.txt\n");
        return;
    }
    FILE* dest = fopen("maps/finalMap.txt", "w");
    if (!dest) {
        printf("Error: Could not open destination map file maps/finalMap.txt\n");
        fclose(src);
        return;
    }
    int ch;
    while ((ch = fgetc(src)) != EOF) {
        fputc(ch, dest);
    }
    fclose(src);
    fclose(dest);
}

static int do_floodfill(const char* map_filename) {
    
    restore_map();

    FILE* file = fopen(map_filename, "r");
    if (!file) {
        printf("Error: Could not open map file %s\n", map_filename);
        return 0;
    }

    for (int y = 0; y < MAP_HEIGHT; y++) {
        for (int x = 0; x < MAP_WIDTH; x++) {
            int ch = fgetc(file);
            while (ch == '\n' || ch == '\r' || ch == ' ') {
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

static float get_global_x(int map_x) {
    // X length is 521cm (5.21m), Xstart is 210
    return (map_x - 210) * (5.21f / MAP_WIDTH);
}

static float get_global_y(int map_y) {
    // Y length is 602cm (6.02m), Ystart is 50 from bottom left
    // If map_y=0 is the top of the array, the distance from bottom is:
    int y_from_bottom = (MAP_HEIGHT - 1) - map_y;
    return (y_from_bottom - 50) * (6.02f / MAP_HEIGHT);
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
            x_target_position[*num_targets] = get_global_x(current_x);
            y_target_position[*num_targets] = get_global_y(current_y);
            (*num_targets)++;
        }
        
        last_dx = next_dx;
        last_dy = next_dy;
        current_x = next_x;
        current_y = next_y;
    }
    
    if (*num_targets < max_targets) {
        x_target_position[*num_targets] = get_global_x(Xfinal);
        y_target_position[*num_targets] = get_global_y(Yfinal);
        (*num_targets)++;
    }

    for (int i = *num_targets; i < max_targets; i++) {
        x_target_position[i] = get_global_x(Xfinal);
        y_target_position[i] = get_global_y(Yfinal);
    }
    printf("Path updated with %d target points.\n", *num_targets);
    // cout << "Path updated with " << *num_targets << " target points.\n";
}

int run_floodfill(const char* map_filename, int x_final, int y_final, float* x_target_position, float* y_target_position, int* num_targets) {
    Xfinal = x_final;
    Yfinal = y_final;
    
    if (do_floodfill(map_filename)) {
        find_path(x_target_position, y_target_position, num_targets, 50);
        return 1;
    }
    *num_targets = 0;
    return 0;
}