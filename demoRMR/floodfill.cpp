#include "floodfill.h"
#include <iostream>
#include <vector>
#include <queue>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

using namespace std;

//int Ystart = 210;
int Ystart = 470;
int Xstart = 50;
int Xfinal = 0;
int Yfinal = 0;

static char mapData[MAP_HEIGHT][MAP_WIDTH];
static int  floodData[MAP_HEIGHT][MAP_WIDTH]; 

static float get_global_x(int map_x);
static float get_global_y(int map_y);

static void debug_print_point(const char* label, int map_x, int map_y) {
    cout << label
         << " map=(" << map_x << ", " << map_y << ")"
         << " global=(" << get_global_x(map_x)
         << ", " << get_global_y(map_y) << ")"
         << endl;
}

struct Point {
    int x;
    int y;
};

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

    cout << "[floodfill] loading map: " << map_filename << endl;
    cout << "[floodfill] start input map=(" << Xstart << ", " << Ystart << ")"
         << " final input map=(" << Xfinal << ", " << Yfinal << ")" << endl;
    
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
            cout << "[floodfill] target cell is open at map=(" << Xfinal
                 << ", " << Yfinal << ")" << endl;
        } else {
            cout << "[floodfill] target cell blocked at map=(" << Xfinal
                 << ", " << Yfinal << ") value='" << mapData[Yfinal][Xfinal]
                 << "'" << endl;
            return 0;
        }
    } else {
        cout << "[floodfill] target coordinates out of bounds" << endl;
        return 0;
    }

    if (Xstart >= 0 && Xstart < MAP_WIDTH && Ystart >= 0 && Ystart < MAP_HEIGHT) {
        mapData[Ystart][Xstart] = 'S';
    }

    debug_print_point("[floodfill] start point", Xstart, Ystart);
    debug_print_point("[floodfill] target point", Xfinal, Yfinal);

    queue<Point> q;

    floodData[Ystart][Xstart] = 0;
    q.push({Xstart, Ystart});

    cout << "[floodfill] BFS initialized from start cell" << endl;

    int dx[] = {1,-1,0,0};
    int dy[] = {0,0,1,-1};
    /*
    why this?
    // Check Right
    int nx = current.x + 1;
    int ny = current.y;
    if (nx >= 0 && nx < MAP_WIDTH ...) { ... }

    // Check Left
    nx = current.x - 1;
    ny = current.y;
    if (nx >= 0 && nx < MAP_WIDTH ...) { ... }

    // Check Down
    nx = current.x;
    ny = current.y + 1;
    if (nx >= 0 && nx < MAP_WIDTH ...) { ... }

    // Check Up
    nx = current.x;
    ny = current.y - 1;
    if (nx >= 0 && nx < MAP_WIDTH ...) { ... }
    this is why
    */
    bool found = false;

    while (!q.empty()) {
        Point current = q.front();
        q.pop();

        if (current.x == Xfinal && current.y == Yfinal) {
            found = true;
            break;
        }

        for (int i = 0; i < 4; i++) {
            int nx = current.x + dx[i];
            int ny = current.y + dy[i];
            if (nx < 0 || ny < 0 || nx >= MAP_WIDTH || ny >= MAP_HEIGHT)
                continue;
            if (mapData[ny][nx] == '1' || mapData[ny][nx] == 'X')
                continue;
            if (floodData[ny][nx] != -1)
                continue;
            floodData[ny][nx] = floodData[current.y][current.x] + 1;
            q.push({nx, ny});
        }
    }
    
    return found ? 1 : 0;
}

static float get_global_x(int map_x) {
    return (map_x - 50) * (5.21f / MAP_WIDTH);
}
static float get_global_y(int map_y) {
    return (map_y - 470) * (6.02f / MAP_HEIGHT);
}

static void find_path(float* x_target_position,
                      float* y_target_position,
                      int* num_targets,
                      int max_targets)
{
    *num_targets = 0;

    cout << "[floodfill] backtracking path" << endl;

    if (floodData[Yfinal][Xfinal] == -1) {
        cout << "[floodfill] no path found" << endl;
        return;
    }

    cout << "[floodfill] path exists, start backtrack from final cell" << endl;
    debug_print_point("[floodfill] backtrack final", Xfinal, Yfinal);

    int current_x = Xfinal;
    int current_y = Yfinal;

    int last_dx = 0;
    int last_dy = 0;

    int dx[] = {0, 0, -1, 1};
    int dy[] = {-1, 1, 0, 0};

    // collect turn points in reverse order
    while (!(current_x == Xstart &&
             current_y == Ystart))
    {
        int current_dist =
            floodData[current_y][current_x];

        int next_x = -1;
        int next_y = -1;

        int next_dx = 0;
        int next_dy = 0;

        for (int i = 0; i < 4; i++)
        {
            int nx = current_x + dx[i];
            int ny = current_y + dy[i];

            if (nx < 0 || ny < 0 ||
                nx >= MAP_WIDTH ||
                ny >= MAP_HEIGHT)
                continue;

            if (floodData[ny][nx] ==
                current_dist - 1)
            {
                next_x = nx;
                next_y = ny;

                next_dx = dx[i];
                next_dy = dy[i];

                break;
            }
        }

        if (next_x == -1)
        {
            cout << "[floodfill] backtrack error at map=(" << current_x
                 << ", " << current_y << ")" << endl;
            return;
        }

        // direction changed -> save waypoint
        if ((last_dx != 0 || last_dy != 0) &&
            (last_dx != next_dx ||
             last_dy != next_dy))
        {
            if (*num_targets < max_targets)
            {
                cout << "[floodfill] waypoint " << *num_targets
                     << " at map=(" << current_x << ", " << current_y << ")"
                     << " global=(" << get_global_x(current_x)
                     << ", " << get_global_y(current_y) << ")" << endl;

                x_target_position[*num_targets] =
                    get_global_x(current_x);

                y_target_position[*num_targets] =
                    get_global_y(current_y);

                (*num_targets)++;
            }
        }

        last_dx = next_dx;
        last_dy = next_dy;

        current_x = next_x;
        current_y = next_y;
    }

    // reverse turn points
    for (int i = 0; i < *num_targets / 2; i++)
    {
        std::swap(
            x_target_position[i],
            x_target_position[*num_targets - 1 - i]);

        std::swap(
            y_target_position[i],
            y_target_position[*num_targets - 1 - i]);
    }

    if (*num_targets < max_targets)
    {
        cout << "[floodfill] final waypoint global=(" << get_global_x(Xfinal)
             << ", " << get_global_y(Yfinal) << ")" << endl;
        x_target_position[*num_targets] =
            get_global_x(Xfinal);

        y_target_position[*num_targets] =
            get_global_y(Yfinal);

        (*num_targets)++;
    }

    cout << "[floodfill] generated " << *num_targets << " waypoints" << endl;

    for (int i = 0; i < *num_targets; i++) {
        cout << "[floodfill] target[" << i << "] = ("
             << x_target_position[i] << ", "
             << y_target_position[i] << ")" << endl;
    }
}
int run_floodfill(const char* map_filename, int x_final, int y_final, float* x_target_position, float* y_target_position, int* num_targets) {
    Xfinal = x_final;
    Yfinal = y_final;

    cout << "[floodfill] run_floodfill request final=(" << Xfinal
         << ", " << Yfinal << ")" << endl;
    
    if (do_floodfill(map_filename)) {
        find_path(x_target_position, y_target_position, num_targets, 50);
        cout << "[floodfill] run_floodfill success" << endl;
        return 1;
    }
    *num_targets = 0;
    cout << "[floodfill] run_floodfill failed" << endl;
    return 0;
}