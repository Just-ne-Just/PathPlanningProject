#ifndef SEARCH_H
#define SEARCH_H
#include "ilogger.h"
#include "searchresult.h"
#include "environmentoptions.h"
#include <list>
#include <vector>
#include <set>
#include <unordered_set>
#include <math.h>
#include <limits>
#include <chrono>
#include <iterator>
#include <iostream>
#include <iomanip>
#include <iostream>
#include <fstream>

struct Comparator {
    bool operator() (const Node* a, const Node* b) const {
        if (a->F == b->F) {
            if (a->i == b->i) {
                return a->j < b->j;
            }
            return a->i < b->i;
        }
        return a->F < b->F;
    }
};

struct DLiteComparator {
    bool operator() (const Node* a, const Node* b) const {
        if (a->key == b->key) {
            if (a->i == b->i) {
                return a->j < b->j;
            }
            return a->i < b->i;
        }
        return a->key < b->key;
    }
};

struct MyHash {
    std::size_t operator()(const std::pair<int, int> &x) const {
        return std::hash<int>()(x.first) ^ std::hash<int>()(x.second);
    }
};


class Search
{
    public:
        Search();
        ~Search(void);
        SearchResult startSearch(ILogger *Logger, const Map &Map, const EnvironmentOptions &options);

        double ComputeHeuristic(int i_current, int j_current,
                                int i_finish, int j_finish,
                                const EnvironmentOptions &options);

        void makePrimaryPath(const Node* curNode, const Map &map);

        void makeSecondaryPath();

    protected:
        std::set<Node*, Comparator>     open_to_get;
        std::set<Node*>                 open_to_get_all;
        std::unordered_map<std::pair<int, int>, Node*, MyHash>  open_to_find;
        std::unordered_map<std::pair<int, int>, Node*, MyHash>  close;
        SearchResult                                            sresult; //This will store the search result
        std::list<Node>                                         lppath, hppath;

};

class SeqSearch
{
public:
    SeqSearch();
    ~SeqSearch(void);

    SearchResult startSeqSearch(ILogger *Logger, Map &Map, const EnvironmentOptions &options);
    std::pair<Node*, Node*> searchStep(ILogger *Logger, Map &Map, const EnvironmentOptions &options, Node startNode);

    double ComputeHeuristic(int i_current, int j_current,
                            int i_finish, int j_finish,
                            const EnvironmentOptions &options);

    void makePrimaryPath(const Node* curNode, Map &map);

    void makeSecondaryPath();
    void localClear();
    void ExpandVisibility(Map& map);
    void PrintInFile(Map& map, std::pair<Node*, Node*> i_path);
    void NormalizePath();

protected:
    std::set<Node*, Comparator>                             open_to_get;
    std::unordered_map<std::pair<int, int>, Node*, MyHash>  open_to_find;
    std::unordered_map<std::pair<int, int>, Node*, MyHash>  close;
    SearchResult                                            sresult; //This will store the search result
    std::list<Node>                                         lppath, hppath;
};

class DLiteSearch
{
    DLiteSearch();
    ~DLiteSearch(void);

    SearchResult StartDLiteSearch(ILogger *Logger, Map &Map, const EnvironmentOptions &options);

    double ComputeHeuristic(int i_current, int j_current,
                     int i_finish, int j_finish,
                     const EnvironmentOptions &options);

    bool FirstStupidCheck(int i, int j, Node* v);

    bool SecondStupidCheck(int i, int j);

    void ComputeShortestPath(const Map &map, const EnvironmentOptions &options);

    void UpdateVertex(Node* v,  const Map &map, const EnvironmentOptions &options);

    void Initialize(const Map &map, const EnvironmentOptions &options);

    std::pair<double, double> CalculateKey(Node* v);

    std::set<Node*, DLiteComparator>                        open_to_get;
    std::unordered_map<std::pair<int, int>, Node*, MyHash>  open_to_find;
    std::unordered_map<std::pair<int, int>, Node*, MyHash>  close;
    SearchResult                                            sresult; //This will store the search result
    std::list<Node>                                         lppath, hppath;
};
#endif
