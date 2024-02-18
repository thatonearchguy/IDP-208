// Number of vertices in the graph
#define numVert 18
#define srcParent -1
#define MAX_DIST 500

enum direction
{
    RIGHT,
    UP,
    LEFT,
    DOWN,
    REVERSE,
    UNREACHABLE
};
typedef struct
{
    int distance;
    direction direction;
} node;

// ------- HELPER FUNCTIONS -------
// returns vertex (not yet completed) that is the shortest distance from source
int minDistance(int distance[numVert], bool completedSet[numVert]);

// initialises adjacency matrix
void initialise(node graph[numVert][numVert]);

// returns optimum path from source to destination.
void returnPath(int parent[numVert], int j, int bestPath[numVert], int *index);
void returnDirection(node graph[numVert][numVert], int bestPath[numVert], direction bestPathDirections[numVert]);

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
void dijkstra(node graph[numVert][numVert], int source, int destination, int bestPath[numVert], direction bestPathDirections[numVert], int distance[numVert]);