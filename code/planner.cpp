/*=================================================================
 *
 * planner.cpp
 *
 *=================================================================*/
#include "planner.h"
#include "pqueue.h"
#include <math.h>
#include <stdio.h>

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

#if !defined(MAX)
#define MAX(A, B) ((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define MIN(A, B) ((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 8

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
  bool* closed = new bool[x_size*y_size]();

  for(int i = 0; i < target_steps; i++){
    node* goal = new node(target_traj[i], target_traj[target_steps+i], 0, 0, nullptr);
    openSet.insertNode(goal);
  }

  node* curr;

  while(curr = openSet.pop()){
    int idx = GETMAPINDEX(curr->x, curr->y, x_size, y_size);

    if (closed[idx]) {
      delete curr;
      continue;
    }

    closed[idx] = true;
    h_table[curr->y-1][curr->x-1] = curr->g;

    for(int dir = 0; dir < NUMOFDIRS; dir++)
    {
      int newx = curr->x + dX[dir];
      int newy = curr->y + dY[dir];

      if (newx >= 1 && newx <= x_size &&
          newy >= 1 && newy <= y_size)
      {
        if ((map[GETMAPINDEX(newx,newy,x_size,y_size)] >= 0) &&
            (map[GETMAPINDEX(newx,newy,x_size,y_size)] < collision_thresh))
        {
          node* n = new node(newx,
                             newy,
                             curr->g + map[GETMAPINDEX(newx,newy,x_size,y_size)],
                             0,
                             nullptr);
          openSet.insertNode(n);
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
  static int** h_table = nullptr;

  if(curr_time == 0){
    h_table = new int*[y_size];
    for(int j = 0; j < y_size; j++){
      h_table[j] = new int[x_size];
    }
    for(int i = 0; i < y_size; i++){
      for(int j = 0; j < x_size; j++){
        h_table[i][j] = 10000;
      }
    }
    backwardsA(target_steps,target_traj,x_size,y_size,h_table,map,collision_thresh);
  }

  int dX[NUMOFDIRS] = {-1,-1,-1,0,0,1,1,1};
  int dY[NUMOFDIRS] = {-1,0,1,-1,1,-1,0,1};

  pqueue openSet(x_size*y_size);
  bool* closed = new bool[x_size*y_size]();

  node* start = new node(robotposeX, robotposeY, 0, 0, nullptr);
  openSet.insertNode(start);

  node* goal = nullptr;
  double lambda = 1.5;

  while(node* current = openSet.pop()){
    int idx = GETMAPINDEX(current->x, current->y, x_size, y_size);

    if(closed[idx]){
        delete current;
        continue;
    }

    closed[idx] = true;

    int target_idx = MIN(curr_time + current->g, target_steps-1);
    int targetX = target_traj[target_idx];
    int targetY = target_traj[target_idx + target_steps];

    if(current->x == targetX && current->y == targetY){
        goal = current;
        break;
    }

    for(int dir=0; dir<NUMOFDIRS; dir++){
        int newx = current->x + dX[dir];
        int newy = current->y + dY[dir];

        if(newx<1 || newx>x_size || newy<1 || newy>y_size) continue;

        int cost_here = map[GETMAPINDEX(newx,newy,x_size,y_size)];
        if(cost_here < 0 || cost_here >= collision_thresh) continue;

        int g_new = current->g + cost_here;

        double dist_to_target = sqrt((newx-targetX)*(newx-targetX) +
                                     (newy-targetY)*(newy-targetY));

        if(dist_to_target >= target_steps-curr_time+10){
          lambda = 800;
        }

        double f_new = g_new +
                       h_table[newy-1][newx-1] +
                       lambda * dist_to_target;

        node* n = new node(newx,
                           newy,
                           g_new,
                           f_new - g_new,
                           current);

        openSet.insertNode(n);
    }
  }

  node* next_step = goal;

  while(next_step &&
        next_step->parent &&
        next_step->parent != start)
      next_step = next_step->parent;

  if(next_step){
      action_ptr[0] = next_step->x;
      action_ptr[1] = next_step->y;
  } else {
      action_ptr[0] = robotposeX;
      action_ptr[1] = robotposeY;
  }

  delete[] closed;
}
