// Number of vertices in the graph
#define numVert 9
#define srcParent -1
#define INT_MAX 500

// ------- HELPER FUNCTIONS -------
// returns vertex (not yet completed) that is the shortest distance from source 
int minDistance(int distance[], bool completedSet[]);

// returns optimum path from source to destination.
void returnPath(int parent[], int j, int bestPath[], int *index);

// ------- PRINTING FUNCTIONS -------
// replace with Serial.write()

// prints optimum path from source to destination.
//void printPath(int bestPath[]);
 
// prints minimum distance from source to every vertex
//void printDistance(int distance[]);

// ------- PRIMARY FUNCTIONS -------
// Implements dijkstra algorithm (to be improved if need be)
void dijkstra(int graph[numVert][numVert], int source, int destination, int bestPath[numVert], int distance[numVert]);
 
// main
int main()
{
 
    /* example graph */
    // can be changed to a float
    int graph[numVert][numVert] = { 
        { 0, 4, 0, 0, 0, 0, 0, 8, 0 },
        { 4, 0, 8, 0, 0, 0, 0, 11, 0 },
        { 0, 8, 0, 7, 0, 4, 0, 0, 2 },
        { 0, 0, 7, 0, 9, 14, 0, 0, 0 },
        { 0, 0, 0, 9, 0, 10, 0, 0, 0 },
        { 0, 0, 4, 14, 10, 0, 2, 0, 0 },
        { 0, 0, 0, 0, 0, 2, 0, 1, 6 },
        { 8, 11, 0, 0, 0, 0, 1, 0, 7 },
        { 0, 0, 2, 0, 0, 0, 6, 7, 0 } 
    };
    // intialise best path 
    int bestPath[numVert];
    for (int i = 0; i < numVert; i++) {
        bestPath[i] = -1;
    }

    // distance[i] will hold the shortest distance from source to i
    int distance[numVert];  

    // fill bestPath with the best path
    dijkstra(graph, 0, 3, bestPath, distance);

    
    // for printing best path
    // printPath(bestPath);
    // printDistance(distance);
    
 
    return 0;
}


void returnPath(int parent[], int j, int bestPath[], int *index)
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

int minDistance(int distance[], bool completedSet[])
{
    // Initialize min value to be infinite
    int min = INT_MAX;
    int min_index;
 
    for (int vert = 0; vert < numVert; vert++)
        if (completedSet[vert] == false && distance[vert] <= min) {
            min = distance[vert];
            min_index = vert;
        }
    // return vertex with min distance (vertices are represented by indices)
    return min_index;
}

void dijkstra(int graph[numVert][numVert], int source, int destination, int bestPath[numVert], int distance[numVert]) {
 
    bool completedSet[numVert]; // completedSet[i] will be true if vertex i's distance to 
    // source can no longer be decreased.
 
    // Initialize all distances as INFINITE and completedSet[] as false
    for (int i = 0; i < numVert; i++)
        distance[i] = INT_MAX, completedSet[i] = false;
 
    // Distance of source vertex from itself is always 0
    distance[source] = 0;

    // initialise parent set
    int parent[numVert] = {srcParent};
 
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
            if (!completedSet[vert] && graph[u][vert] && distance[u] != INT_MAX && distance[u] + graph[u][vert] < distance[vert]) {
                    // set parent of u to be previous spt vertex
                    parent[vert] = u;
                    distance[vert] = distance[u] + graph[u][vert];
                }
    }
 
    // print the constructed distance array

    // return best path as an array
    int index = 0;
    returnPath(parent, destination, bestPath, &index);
}
 
/*
void printPath(int bestPath[])
{
    // for printing best path
    cout << endl;
    cout << "Optimum path: ";
    for (int i = 0; i < numVert; i++) {
        if (bestPath[i] != -1) {
            cout << bestPath[i] << ' ';
        } 
    }
    cout << endl;
}

void printDistance(int distance[])
{
    cout << endl;
    cout << "Vertex \t Distance from Source" << endl;
    for (int i = 0; i < numVert; i++) {
        cout << i << " \t\t\t\t" << distance[i] << endl;
    }
}
*/