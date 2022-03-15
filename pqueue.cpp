#ifndef _PQUEUE_CPP
#define _PQUEUE_CPP

#include <queue>
#include <iostream>
#include "pqueue.h"

using namespace std;

template <class T, class N>
pqueue<T, N>::pqueue() {
}

template <class T, class N>
bool pqueue<T, N>::empty()
{
    return the_queue.empty();
}

template <class T, class N>
size_t pqueue<T, N>::size()
{
    return the_queue.size();
}

template <class T, class N>
void pqueue<T, N>::enqueue(const T& item, N pri)
{
    the_queue.push(pqueue_item<T, N>(item, pri));
}

template <class T, class N>
T pqueue<T, N>::dequeue()
{
    if (empty())
    {
        cerr << "Error in pqueue::dequeue: priority queue is empty." << endl;
        exit(1);
    }
    pqueue_item<T, N> item = the_queue.top();
    the_queue.pop();
    return item.item;
}


#endif
