#ifndef PQUEUE_H
#define PQUEUE_H

#include <vector>

struct node {
    int x, y;
    int g, h, f;
    node* parent;

    node(int x_, int y_, int g_ = 0, int h_ = 0, node* parent_ = nullptr);
};

class pqueue {
private:
    std::vector<node*> array;
    int size;
    int capacity;

    void sort(int i);

public:
    pqueue(int cap);
    void insertNode(node* val);
    node* pop();
    void printHeap() const;
};

#endif
