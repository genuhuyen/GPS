#ifndef _PQUEUE_H
#define _PQUEUE_H

#include <queue>

using namespace std;

template <class T, class N>
struct pqueue_item
{
    public:
    T item;
    N priority;

    pqueue_item(T i, N p) : item(i), priority(p) {}
};

template <class T, class N>
struct pqueue_item_comp
{
    bool operator()(pqueue_item<T, N> const& item1, pqueue_item<T, N> const& item2) {
        return item1.priority > item2.priority;
    }
};

template <class T, class N>
class pqueue
{
    public:

    pqueue();

    bool empty();
    size_t size();

    void enqueue(const T& item, N pri);
    T dequeue();

    private:

    priority_queue<pqueue_item<T, N>, vector<pqueue_item<T, N> >, pqueue_item_comp<T, N> > the_queue;
};

#include "pqueue.cpp"

#endif
