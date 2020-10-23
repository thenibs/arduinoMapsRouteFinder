/* Name: Nibras Alam & Al Hisham Anik
# ID: 1617818 & 1585385
# CMPUT 275, Winter 2020
#
# Assignment 2: Driving Route Finder Part 1
#
# Consulted C++ Documentation; Used Lecture Slides
*/
#include "dijkstra.h"
#include "heap.h"
#include <list>  // for list()

// for testing
#include <iostream>
using namespace std;

void dijkstra(const WDigraph& graph, int startVertex,
                        unordered_map<int, PIL>& tree) {
    BinaryHeap<PIL, int> fires;
    fires.insert(PIL(startVertex, startVertex), 0);

    while (fires.size() > 0) {
        HeapItem<PIL, int> hItem = fires.min();
        int edgeU = hItem.item.first, edgeV = hItem.item.second;
        int time = hItem.key;

        fires.popMin();
        if (tree.find(edgeV) != tree.end()) {
            continue;
        }
        
        tree.insert(PIPIL(edgeV, PIL(edgeU, time)));
        for (auto iter = graph.neighbours(edgeV); 
                    iter != graph.endIterator(edgeV); iter++) {
            int neighbour = *iter;
            int burn = time + graph.getCost(edgeV, neighbour);
            fires.insert(PIL(edgeV, neighbour), burn);
        }       
    }
}
