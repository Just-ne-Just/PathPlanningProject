//#ifndef SEQSEARCH_H
//#define SEQSEARCH_H
//#include "ilogger.h"
//#include "searchresult.h"
//#include "environmentoptions.h"
//#include <list>
//#include <vector>
//#include <set>
//#include <unordered_set>
//#include <math.h>
//#include <limits>
//#include <chrono>
//#include <iterator>
//#include <iostream>
//#include <iomanip>
//
//
//class SeqSearch
//{
//public:
//    SeqSearch();
//    ~SeqSearch(void);
//
//    SearchResult startSeqSearch(ILogger *Logger, const Map &Map, const EnvironmentOptions &options);
//    SearchResult searchStep(ILogger *Logger, const Map &Map, const EnvironmentOptions &options, Node startNode);
//
//    double ComputeHeuristic(int i_current, int j_current,
//                            int i_finish, int j_finish,
//                            const EnvironmentOptions &options);
//
//    void makePrimaryPath(const Node* curNode, const Map &map);
//
//    void makeSecondaryPath();
//    void localClear();
//
//protected:
//    //CODE HERE
//
//    //Hint 1. You definitely need class variables for OPEN and CLOSE
//
//    //Hint 2. It's a good idea to define a heuristic calculation function, that will simply return 0
//    //for non-heuristic search methods like Dijkstra
//
//    //Hint 3. It's a good idea to define function that given a node (and other stuff needed)
//    //will return it's successors, e.g. unordered list of nodes
//
//    //Hint 4. working with OPEN and CLOSE is the core
//    //so think of the data structures that needed to be used, about the wrap-up classes (if needed)
//    //Start with very simple (and ineffective) structures like list or vector and make it work first
//    //and only then begin enhancement!
//    std::set<Node*, Comparator>     open_to_get;
//    std::set<Node*>                 open_to_get_all;
//    std::unordered_map<std::pair<int, int>, Node*, MyHash>  open_to_find;
//    std::unordered_map<std::pair<int, int>, Node*, MyHash>  close;
//    SearchResult                                            sresult; //This will store the search result
//    std::list<Node>                                         lppath, hppath;
//
//    //CODE HERE to define other members of the class
//};
//#endif
