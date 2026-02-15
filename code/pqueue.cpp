#include "pqueue.h"
#include <iostream>
#include <utility>

using namespace std;

node::node(int x_, int y_, int g_, int h_, node* parent_)
  :x(x_), y(y_), g(g_), h(h_), f(g_+h_), parent(parent_){}

pqueue::pqueue(int cap){
  size = 0;
  capacity = cap;
  array.resize(cap);
}

void pqueue::sort(int i){
  int smallest = i;
  int left = 2*i+1;
  int right = 2*i+2;
  if(left<size && array[left]->f < array[smallest]->f){
    smallest = left;
  }
  if(right<size && array[right]->f < array[smallest]->f){
    smallest = right;
  }
  if(smallest != i){
    swap(array[i],array[smallest]);
    sort(smallest);
  }
}

void pqueue::insertNode(node* val){
  if(size == capacity){
    capacity*=2;
    array.resize(capacity);
  }
  size++;
  int i = size -1;
  array[i] = val;
  while(i != 0 && array[(i-1)/2]->f > array[i]->f){
    swap(array[i],array[(i-1)/2]);
    i = (i-1)/2;
  }
}
node* pqueue::pop(){
  if(size <= 0){
    return NULL;
  }
  if(size == 1){
    size --;
    return array[0];
  }
  node* root = array[0];
  array[0] = array[size-1];
  size--;
  sort(0);
  return root;
}

void pqueue::printHeap() const{
  for(int i = 0; i < size; i++){
    cout << array[i]->f << " ";
  }
  cout << endl;
}


