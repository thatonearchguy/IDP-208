#include "pathFinder.h"

// main
int example()
{
    node graph[numVert][numVert];
    initialise(graph);

    // declare best path 
    int bestPath[numVert];

    // contains directions to take in path order
    direction bestPathDirections[numVert];

    // declare parents
    int parent[numVert];

    // distance[i] will hold the shortest distance from source to i
    int distance[numVert]; 

    dijkstra(graph, 0, bestPath, bestPathDirections, distance, parent);
    getOptimalPath(parent, 7, bestPath, graph, bestPathDirections);

    // printDistance(distance);
    // printPath(bestPath, bestPathDirections);
    return 0;
}

void returnPath(int parent[numVert], int j, int bestPath[numVert], int *index)
{
    // Base Case : If j is source
    if (parent[j] == srcParent){
        bestPath[*index] = j;
        return;
    }
    // recurse if j is not parent
    returnPath(parent, parent[j], bestPath, index);
    (*index)++;
    bestPath[*index] = j;
    return;
}

void returnDirection(node graph[numVert][numVert], int bestPath[numVert], direction bestPathDirections[numVert]) {
    
    for (int i = 0; i < numVert-1; i++) {
        bestPathDirections[i] = graph[bestPath[i]][bestPath[i+1]].direction;
    }
}

int minDistance(int distance[numVert], bool completedSet[numVert])
{
    // Initialize min value to be infinite
    int min = MAX_DIST;
    int min_index;
 
    for (int vert = 0; vert < numVert; vert++)
        if (completedSet[vert] == false && distance[vert] <= min) {
            min = distance[vert];
            min_index = vert;
        }
    // return vertex with min distance (vertices are represented by indices)
    return min_index;
}

void getOptimalPath(int parent[numVert], int destination, int bestPath[numVert], node graph[numVert][numVert], direction bestPathDirections[numVert]) {
    int index = 0;
    returnPath(parent, destination, bestPath, &index);
    returnDirection(graph, bestPath, bestPathDirections);
}

void dijkstra(node graph[numVert][numVert], int source, int bestPath[numVert], direction bestPathDirections[numVert], int distance[numVert], int parent[numVert]) {
 
    bool completedSet[numVert]; // completedSet[i] will be true if vertex i's distance to 
    // source can no longer be decreased.
 
    // Initialize all distances as INFINITE and completedSet[] as false
    for (int i = 0; i < numVert; i++)
        distance[i] = MAX_DIST, completedSet[i] = false;
 
    // Distance of source vertex from itself is always 0
    distance[source] = 0;

    // initialise best path 
    for (int i = 0; i < numVert; i++) {
            bestPath[i] = -1;
        }
    // initialise parent set
    for (int i = 0; i < numVert; i++) {
        parent[i] = srcParent;
    }
 
    // Find shortest path for all vertices
    for (int count = 0; count < numVert - 1; count++) {
        // Pick the minimum distance vertex from the set of
        // vertices not yet processed. u is always equal to
        // source in the first iteration.
        int u = minDistance(distance, completedSet);
 
        // Mark the picked vertex as processed
        completedSet[u] = true;
 
        // Update distance value of the adjacent vertices of the
        // picked vertex.
        for (int vert = 0; vert < numVert; vert++)
 
            // Update distance[numVert] only if is not in completedSet,
            // there is an edge from u to numVert, and total
            // weight of path from source to  numVert through u is
            // smaller than current value of distance[numVert]
            if (!completedSet[vert] && graph[u][vert].distance && distance[u] != MAX_DIST && distance[u] + graph[u][vert].distance < distance[vert]) {
                    // set parent of u to be previous spt vertex
                    parent[vert] = u;
                    distance[vert] = distance[u] + graph[u][vert].distance;
                }
    }
}


void initialise(node graph[numVert][numVert]) {
    for (int i = 0; i < numVert; i++) {
        for (int j = 0; j < numVert; j++) {
            graph[i][j].direction = UNREACHABLE;
            graph[i][j].distance = 0;
        }
    }
    graph[0][3].distance = 46; graph[3][0].distance = 46;
    graph[1][2].distance = 71; graph[2][1].distance = 71; 
    graph[1][5].distance = 46; graph[5][1].distance = 46;
    graph[1][8].distance = 85; graph[8][1].distance = 85;
    graph[2][3].distance = 33; graph[3][2].distance = 33;
    graph[2][7].distance = 34; graph[7][2].distance = 34;
    graph[3][4].distance = 104; graph[4][3].distance = 104;
    graph[4][6].distance = 46; graph[6][4].distance = 46;
    graph[4][11].distance = 85; graph[11][4].distance = 85;
    graph[8][9].distance = 102; graph[9][8].distance = 102;
    graph[8][18].distance = 76; graph[18][8].distance = 76;
    graph[18][15].distance = 103; graph[15][18].distance = 103; // bend
    graph[9][10].distance = 35; graph[10][9].distance = 35;
    graph[9][13].distance = 37; graph[13][9].distance = 37;
    graph[10][11].distance = 72; graph[11][10].distance = 72;
    graph[10][12].distance = 30; graph[12][10].distance = 30;
    graph[19][16].distance = 63; graph[16][19].distance = 63;// bend
    graph[11][19].distance = 76; graph[19][11].distance = 76;
    graph[13][14].distance = 45; graph[14][13].distance = 45;
    graph[13][15].distance = 39; graph[15][13].distance = 39;
    graph[15][16].distance = 42; graph[16][15].distance = 42;
    graph[16][17].distance = 38; graph[17][16].distance = 38;

    graph[0][3].direction = UP; graph[3][0].direction = FINISH;
    graph[1][2].direction = RIGHT; graph[2][1].direction = LEFT; 
    graph[1][5].direction = DOWN; graph[5][1].direction = REVERSE;
    graph[1][8].direction = UP; graph[8][1].direction = DOWN;
    graph[2][3].direction = RIGHT; graph[3][2].direction = LEFT;
    graph[2][7].direction = UP; graph[7][2].direction = REVERSE;
    graph[3][4].direction = RIGHT; graph[4][3].direction = LEFT;
    graph[4][6].direction = DOWN; graph[6][4].direction = REVERSE;
    graph[4][11].direction = UP; graph[11][4].direction = DOWN;
    graph[8][9].direction = RIGHT; graph[9][8].direction = LEFT;
    graph[8][18].direction = UP; graph[18][8].direction = DOWN;
    graph[18][15].direction = RIGHT; graph[15][18].direction = LEFT; // bend
    graph[9][10].direction = RIGHT; graph[10][9].direction = LEFT;
    graph[9][13].direction = UP; graph[13][9].direction = DOWN;
    graph[10][11].direction = RIGHT; graph[11][10].direction = LEFT;
    graph[10][12].direction = DOWN; graph[12][10].direction = REVERSE;
    graph[19][16].direction = LEFT; graph[16][19].direction = RIGHT;// bend
    graph[11][19].direction = UP; graph[19][11].direction = DOWN;
    graph[13][14].direction = LEFT; graph[14][13].direction = REVERSE;
    graph[13][15].direction = UP; graph[15][13].direction = DOWN;
    graph[15][16].direction = RIGHT; graph[16][15].direction = LEFT;
    graph[16][17].direction = DOWN; graph[17][16].direction = REVERSE;
} 

// replace with Serial.println()
/*
void printDirection(direction dir) {
    switch (dir) {
        case LEFT:
            cout << "LEFT ";
            return;
        case RIGHT:
            cout << "RIGHT ";
            return;
        case UP:
            cout << "UP ";
            return;
        case DOWN:
            cout << "DOWN ";
            return;
        case UNREACHABLE:
            return;
    }
}

void printPath(int bestPath[numVert], direction bestPathDirections[numVert])
{
    // for printing best path
    cout << endl;
    cout << "Optimum path: ";
    for (int i = 0; i < numVert; i++) {
        if (bestPath[i] != -1) {
            cout << bestPath[i] << ' ';
            printDirection(bestPathDirections[i]);
        } 
    }
    cout << endl;
}

void printDistance(int distance[numVert])
{
    cout << endl;
    cout << "Vertex \t Distance from Source" << endl;
    for (int i = 0; i < numVert; i++) {
        cout << i << " \t\t\t\t" << distance[i] << endl;
    }
}
*/