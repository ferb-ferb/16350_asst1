#include "planner.h"
#include "pqueue.h"
#include <math.h>
#include <stdio.h>
#include <vector>

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))
#define GET3DINDEX(X, Y, T, XSIZE, YSIZE) ((T)*(XSIZE*YSIZE) + (Y-1)*XSIZE + (X-1))

#if !defined(MAX)
#define MAX(A, B) ((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define MIN(A, B) ((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 9 // Added 9th direction for "Wait"

 void backwardsA(

  int target_steps,

  int* target_traj,

  int x_size,

  int y_size,

  int** h_table,

  int* map,

  int collision_thresh

  )

{

  int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};

  int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};

  pqueue openSet(x_size*y_size);

  bool* closed = new bool[x_size*y_size];

  for(int i = 0; i < target_steps; i++){

    node* goal = new node(target_traj[i], target_traj[target_steps+i]);

    openSet.insertNode(goal);

  }

  node* curr;

  while(curr = openSet.pop()){

    int idx = GETMAPINDEX(curr->x, curr->y, x_size, y_size);

    if (closed[idx] == 1) {

      delete curr;

      continue;

    }

    closed[idx] = true;

    h_table[curr->y-1][curr->x-1] = curr->g;

    for(int dir = 0; dir < NUMOFDIRS; dir++)

    {

      //Try each newx and newy

      int newx = curr->x + dX[dir];

      int newy = curr->y + dY[dir];


      //If this pose is in the map

      if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size)

      {

        //if not a collision 

        if ((map[GETMAPINDEX(newx,newy,x_size,y_size)] >= 0) && (map[GETMAPINDEX(newx,newy,x_size,y_size)] < collision_thresh))  //if free

        {

          //

          node* n = new node(newx,newy,curr->g + map[GETMAPINDEX(newx,newy,x_size,y_size)]);

          // if(){

          openSet.insertNode(n);

          // }

        }

      }

    }

    delete curr;

  }

  delete[] closed;

} 

void planner(
    int* map,
    int collision_thresh,
    int x_size,
    int y_size,
    int robotposeX,
    int robotposeY,
    int target_steps,
    int* target_traj,
    int targetposeX,
    int targetposeY,
    int curr_time,
    int* action_ptr
)
{
    // --- 1. Static Heuristic Table (Persistent) ---
    static int** h_table = nullptr;
    if(curr_time == 0 || h_table == nullptr){
        if(h_table) { // Cleanup if resizing is needed
            for(int j=0; j<y_size; j++) delete[] h_table[j];
            delete[] h_table;
        }
        h_table = new int*[y_size];
        for(int j = 0; j < y_size; j++) {
            h_table[j] = new int[x_size];
            for(int i = 0; i < x_size; i++) h_table[j][i] = 1000000;
        }
        backwardsA(target_steps, target_traj, x_size, y_size, h_table, map, collision_thresh);
    }

    // --- 2. Space-Time A* Setup ---
    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1, 0};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1, 0};

    // Use a G-table to prune paths. Initialize with "infinity"
    // This is much faster than a bool closed list.
    int total_states = x_size * y_size * target_steps;
    std::vector<int> g_table(total_states, 1000000);

    pqueue openSet(50000); 

    // Start state
    int start_g = 0;
    int start_h = h_table[robotposeY-1][robotposeX-1];
    node* start = new node(robotposeX, robotposeY, start_g, start_h, nullptr, curr_time);
    
    g_table[GET3DINDEX(robotposeX, robotposeY, curr_time, x_size, y_size)] = 0;
    openSet.insertNode(start);

    node* goal_node = nullptr;
    std::vector<node*> all_nodes; // To manage memory cleanup
    all_nodes.push_back(start);

    // --- 3. Main Search Loop ---
    while(node* current = openSet.pop()) {
        
        // Goal Check: Did we intercept the target at this time?
        int targetX = target_traj[current->t];
        int targetY = target_traj[current->t + target_steps];

        if(current->x == targetX && current->y == targetY) {
            goal_node = current;
            break;
        }

        if(current->t >= target_steps - 1) continue;

        for(int dir=0; dir<NUMOFDIRS; dir++) {
            int newx = current->x + dX[dir];
            int newy = current->y + dY[dir];
            int newt = current->t + 1;

            if(newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size) {
                int map_cost = map[GETMAPINDEX(newx, newy, x_size, y_size)];
                
                if(map_cost >= 0 && map_cost < collision_thresh) {
                    int new_g = current->g + map_cost + 1; // +1 for time step
                    int state_idx = GET3DINDEX(newx, newy, newt, x_size, y_size);

                    // PRUNING: Only expand if this is the best path to this (x,y,t)
                    if(new_g < g_table[state_idx]) {
                        g_table[state_idx] = new_g;
                        int new_h = h_table[newy-1][newx-1];
                        
                        node* n = new node(newx, newy, new_g, new_h, current, newt);
                        openSet.insertNode(n);
                        all_nodes.push_back(n); 
                    }
                }
            }
        }
    }

    // --- 4. Extract Action ---
    if(goal_node) {
        node* curr = goal_node;
        // Trace back to the first move after the current state
        while(curr->parent && curr->parent != start) {
            curr = curr->parent;
        }
        action_ptr[0] = curr->x;
        action_ptr[1] = curr->y;
    } else {
        // Fallback: stay put if no path found
        action_ptr[0] = robotposeX;
        action_ptr[1] = robotposeY;
    }

    // --- 5. Cleanup ---
    for(node* n : all_nodes) delete n;
}