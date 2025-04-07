#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

typedef struct TreeNode {
    int row, col;
    float cost;
    struct TreeNode* parent;
} TreeNode;


#define MAX_VERTICES 10000
TreeNode* vertices[MAX_VERTICES];
int vertex_count = 0;

// Euclidean distance
float euclidean(TreeNode* a, TreeNode* b) {
    return sqrtf((a->row - b->row) * (a->row - b->row) + (a->col - b->col) * (a->col - b->col));
}

// Get random sample point
void get_sample(TreeNode* result, int rows, int cols, int goal_row, int goal_col, int goal_bias) {
    if (rand() % 100 < goal_bias) {
        result->row = goal_row;
        result->col = goal_col;
    } else {
        result->row = rand() % rows;
        result->col = rand() % cols;
    }
}

// Collision check (simple 8-connectivity)
int check_collision(unsigned char* map, int rows, int cols, TreeNode* a, TreeNode* b) {
    int steps = 10;
    float dr = (b->row - a->row) / (float)steps;
    float dc = (b->col - a->col) / (float)steps;

    for (int i = 0; i <= steps; i++) {
        int r = (int)(a->row + i * dr);
        int c = (int)(a->col + i * dc);
        if (r < 0 || r >= rows || c < 0 || c >= cols || map[r * cols + c] == 0)
            return 0;
    }
    return 1;
}

// Extend toward sample point
void extend(TreeNode* nearest, TreeNode* sample, TreeNode* result) {
    float theta = atan2f(sample->col - nearest->col, sample->row - nearest->row);
    int step = 10;
    result->row = nearest->row + step * cosf(theta);
    result->col = nearest->col + step * sinf(theta);
    result->cost = 0.0;
    result->parent = NULL;
}

// Nearest neighbor
TreeNode* get_nearest(TreeNode* node) {
    float min_dist = 1e9;
    TreeNode* nearest = NULL;
    for (int i = 0; i < vertex_count; i++) {
        float dist = euclidean(vertices[i], node);
        if (dist < min_dist) {
            min_dist = dist;
            nearest = vertices[i];
        }
    }
    return nearest;
}

// Main RRT* function
int run_rrt_star(unsigned char* map, int rows, int cols,
                 int start_row, int start_col,
                 int goal_row, int goal_col,
                 int n_pts, int neighbor_size, const char* name) {

    printf("Running RRT* in C from (%d, %d) to (%d, %d)\n", start_row, start_col, goal_row, goal_col);

    srand(time(NULL));
    clock_t start_time = clock();
    vertex_count = 0;

    // Start and Goal nodes
    TreeNode* start = malloc(sizeof(TreeNode));
    start->row = start_row;
    start->col = start_col;
    start->cost = 0;
    start->parent = NULL;
    vertices[vertex_count++] = start;

    TreeNode goal = {goal_row, goal_col, 0, NULL};
    int goal_found = 0;

    for (int i = 0; i < n_pts && vertex_count < MAX_VERTICES; i++) {
        TreeNode sample;
        get_sample(&sample, rows, cols, goal_row, goal_col, 4);
        if (map[sample.row * cols + sample.col] == 0) continue;

        TreeNode* nearest = get_nearest(&sample);
        TreeNode* new_node = malloc(sizeof(TreeNode));
        extend(nearest, &sample, new_node);

        if (!check_collision(map, rows, cols, nearest, new_node)) {
            free(new_node);
            continue;
        }

        new_node->cost = nearest->cost + euclidean(nearest, new_node);
        new_node->parent = nearest;
        vertices[vertex_count++] = new_node;

        float dist_to_goal = euclidean(new_node, &goal);
        if (dist_to_goal <= 10 && check_collision(map, rows, cols, new_node, &goal)) {
            goal.cost = new_node->cost + dist_to_goal;
            goal.parent = new_node;
            goal_found = 1;
            break;
        }
    }

    printf("finishloop\n");
    clock_t end_time = clock();
    double elapsed_time = (double)(end_time - start_time) / CLOCKS_PER_SEC;

    printf("%s\n", name);

    if (goal_found) {
        // Count steps from goal to start
        int steps = 0;
        TreeNode* current = goal.parent;
        while (current && current->parent) {
            steps++;
            current = current->parent;
        }

        printf("It took %d nodes to find the path using RRT*\n", steps);
        printf("The length of path is %.2f\n", goal.cost);
        printf("Elapsed time: %.4f seconds\n", elapsed_time);
        return 1;
    } else {
        printf("No path found\n");
        printf("Elapsed time: %.4f seconds\n", elapsed_time);
        return 0;
    }
}
