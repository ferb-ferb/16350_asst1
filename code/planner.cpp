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
    pqueue openSet(100);
    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};
    for(int t = curr_time; t < target_steps; t++){
      node* g = new node(target_traj[t], target_traj[t+target_steps]);
      openSet.insertNode(g);
    }
    while (node* current = openSet.pop()){
      for(int dir = 0; dir < NUMOFDIRS; dir++)
      {
        //Try each newx and newy
        int newx = current->x + dX[dir];
        int newy = current->y + dY[dir];

        //If this pose is in the map
        if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size)
        {
          //if not a collision 
          if ((map[GETMAPINDEX(newx,newy,x_size,y_size)] >= 0) && (map[GETMAPINDEX(newx,newy,x_size,y_size)] < collision_thresh))  //if free
          {

            node* n = new node(newx,newy,current->g + map[GETMAPINDEX(newx,newy,x_size,y_size)]);
            openSet.insertNode(n);
          }
        }
      }
    }
    
    //int goalposeX = target_traj[target_steps-1];
    //int goalposeY = target_traj[target_steps-1+target_steps];
    node start = new node(robotposeX, robotposeY, map[GETMAPINDEX(robotposeX,robotposeY,x_size,y_size)], 0, nullptr);
    openSet.insertNode(start);
    int disttotarget;
    while(node* current = openSet.pop()){
      if(){}
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
            node* n = new node(newx,newy,map[GETMAPINDEX(newx,newy,x_size,y_size)], disttotarget);
            openSet.insertNode(n);
          }
        }
      }
    }
    action_ptr[0] = openSet.pop()->x;
    action_ptr[1] = openSet.pop()->y;
    return;
}
