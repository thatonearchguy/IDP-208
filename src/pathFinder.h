#ifndef PATHFINDER_H
#define PATHFINDER_H

#include <Arduino.h>
// Number of vertices in the graph
#define numVert 20
#define numBlocks 4
#define srcParent -1
#define MAX_DIST 500

enum direction : uint8_t
{
    RIGHT,
    UP,
    LEFT,
    DOWN,
    REVERSE,
    FINISH,
    UNREACHABLE,
};
typedef struct
{
    uint8_t distance;
    direction direction;
} node;

enum status: uint8_t {
    NOTCOMPLETED,
    COMPLETED,
};

// ------- HELPER FUNCTIONS -------
// returns vertex (not yet completed) that is the shortest distance from source
int minDistance(int distance[numVert], bool completedSet[numVert]);

// initialises adjacency matrix
void initialise(node graph[numVert][numVert]);

// returns optimum path from source to destination.
void returnPath(int parent[numVert], int j, int bestPath[numVert], int *index);
void returnDirection(node graph[numVert][numVert], int bestPath[numVert], direction bestPathDirections[numVert]);
void getOptimalPath(int parent[numVert], int destination, int bestPath[numVert], node graph[numVert][numVert], direction bestPathDirections[numVert]);

// ------- PRINTING FUNCTIONS -------
// replace with Serial.write()

// prints enum direction
// void printDirection(direction dir);

// prints optimum path from source to destination.
// void printPath(int bestPath[numVert], direction bestPathDirections[numVert]);

// prints minimum distance from source to every vertex
// void printDistance(int distance[numVert]);

// ------- PRIMARY FUNCTIONS -------
// Implements dijkstra algorithm (to be improved if need be)
void dijkstra(node graph[numVert][numVert], int source, int bestPath[numVert], direction bestPathDirections[numVert], int distance[numVert], int parent[numVert]);

#endif