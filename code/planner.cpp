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
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
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
    // 8-connected grid
    // Create moves 
    //auto* a = new pqueue_element(1,2,3);
    //printf("%d\n", a->priority);
    /*
    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};
    
    // for now greedily move towards the final target position,
    // but this is where you can put your planner

    //goalx comes from last X value, Y is last Y value, traj goes {all x step followed by all y steps }
    int goalposeX = target_traj[target_steps-1];
    int goalposeY = target_traj[target_steps-1+target_steps];
    // printf("robot: %d %d;\n", robotposeX, robotposeY);
    //printf("goal: %d ", target_traj[target_steps-1+target_steps]);

    //Initialize movements to no movement so that if collision robot stays still 
    int bestX = 0, bestY = 0; // robot will not move if greedy action leads to collision
    //Euclidian Distance 
    double olddisttotarget = (double)sqrt(((robotposeX-goalposeX)*(robotposeX-goalposeX) + (robotposeY-goalposeY)*(robotposeY-goalposeY)));
    double disttotarget;
    //For all of the possible directions
    for(int dir = 0; dir < NUMOFDIRS; dir++)
    {
        //Try each newx and newy
        int newx = robotposeX + dX[dir];
        int newy = robotposeY + dY[dir];

        //If this pose is in the map
        if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size)
        {
            //if not a collision 
            if ((map[GETMAPINDEX(newx,newy,x_size,y_size)] >= 0) && (map[GETMAPINDEX(newx,newy,x_size,y_size)] < collision_thresh))  //if free
            {
                //calculate the distance to target from new possible pose
                disttotarget = (double)sqrt(((newx-goalposeX)*(newx-goalposeX) + (newy-goalposeY)*(newy-goalposeY)));
                //If its better update all the shits
                if(disttotarget < olddisttotarget)
                {
                    olddisttotarget = disttotarget;
                    bestX = dX[dir];
                    bestY = dY[dir];
                }
            }
        }
    }
    robotposeX = robotposeX + bestX;
    robotposeY = robotposeY + bestY;
    action_ptr[0] = robotposeX;
    action_ptr[1] = robotposeY;
    */
    /*pqueue openSet(10);
    node* n1 = new node(0,0,0,10);
    openSet.insertNode(n1);
    node* neighbor = new node(0,1,1,8);
    openSet.insertNode(neighbor);
    openSet.printHeap();
    */
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
    for(int i = 0; i < y_size; i++){
      printf("Row %d: h=%d\n",i,h_table[i][0]);
    }
  }
    //int goalposeX = target_traj[target_steps-1];
  //int goalposeY = target_traj[target_steps-1+target_steps];
  // node* start = new node(robotposeX, robotposeY, map[GETMAPINDEX(robotposeX,robotposeY,x_size,y_size)], 0, nullptr);
  // openSet.insertNode(start);
  // int disttotarget;
  // while(node* current = openSet.pop()){
  //   for(int dir = 0; dir < NUMOFDIRS; dir++)
  //   {
  //     //Try each newx and newy
  //     int newx = robotposeX + dX[dir];
  //     int newy = robotposeY + dY[dir];
  //
  //     //If this pose is in the map
  //     if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size)
  //     {
  //       //if not a collision 
  //       if ((map[GETMAPINDEX(newx,newy,x_size,y_size)] >= 0) && (map[GETMAPINDEX(newx,newy,x_size,y_size)] < collision_thresh))  //if free
  //       {
  //         //calculate the distance to target from new possible pose
  //         disttotarget = (double)sqrt(((newx-goalposeX)*(newx-goalposeX) + (newy-goalposeY)*(newy-goalposeY)));
  //         //If its better update all the shits
  //         node* n = new node(newx,newy,map[GETMAPINDEX(newx,newy,x_size,y_size)], disttotarget);
  //         openSet.insertNode(n);
  //       }
  //     }
  //   }
  // }
  // action_ptr[0] = openSet.pop()->x;
  // action_ptr[1] = openSet.pop()->y;
  return;
}
