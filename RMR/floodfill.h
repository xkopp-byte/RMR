#ifndef FLOODFILL_H
#define FLOODFILL_H

#ifdef __cplusplus
extern "C" {
#endif

// Map dimensions
#define MAP_WIDTH 604
#define MAP_HEIGHT 682

// Starting and final coordinates
extern int Xstart;
extern int Ystart;
extern int Xfinal;
extern int Yfinal;

/**
 * Initializes the floodfill map and runs the floodfill algorithm from 
 * the (Xfinal, Yfinal) to (Xstart, Ystart). 
 * Replaces '0' with distance increments.
 *
 * @param map_filename The path to the 'finalMap.txt' file 
 * @param x_target_position Array to store computed path breaking points (X)
 * @param y_target_position Array to store computed path breaking points (Y)
 * @param num_targets Pointer to integer to store the number of computed points
 */
int run_floodfill(const char* map_filename, float* x_target_position, float* y_target_position, int* num_targets);

// Make room for breaking points calculation later
// Calculates the path with the least amount of breaks based on the floodfilled map data
// void calculate_breaking_points_from_floodfill(float* x_targets, float* y_targets, int* num_targets);

#ifdef __cplusplus
}
#endif

#endif // FLOODFILL_H
