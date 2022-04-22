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
    bool operator() (const DLiteNode* a, const DLiteNode* b) const {
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
public:
    DLiteSearch();
    ~DLiteSearch(void);

    SearchResult StartDLiteSearch(ILogger *Logger, Map &Map, const EnvironmentOptions &options);

    double ComputeHeuristic(int i_current, int j_current,
                     int i_finish, int j_finish,
                     const EnvironmentOptions &options);


    int ComputeShortestPath(const Map &map, const EnvironmentOptions &options, double k);

    void UpdateVertex(DLiteNode* v,  const Map &map, const EnvironmentOptions &options, double k);

    DLiteNode* Argmin(const DLiteNode* v, const Map &map, const EnvironmentOptions &options);

    std::pair<double, double> CalculateKey(DLiteNode* v, double k);

    std::vector<std::pair<int, int>> ExpandVisibility(Map &map);

    double GetRhs(DLiteNode* v, const Map& map, const EnvironmentOptions& options) {
        if (std::pair<int, int>(v->i, v->j) == map.getFinish()) {
            return 0;
        }
        if (v->rhs == 1e9) {
            return ComputeHeuristic(v->i, v->j, map.getFinish().first, map.getFinish().second, options);
        }
        if (v->cost < 0) {
            return 1e9;
        }
        return v->rhs;
    }

    double GetG(DLiteNode* v, const Map& map, const EnvironmentOptions& options) {
        if (v->g == 1e9) {
            return ComputeHeuristic(v->i, v->j, map.getFinish().first, map.getFinish().second, options);
        }
        return v->g;
    }

    double Distance(DLiteNode* v1, DLiteNode* v2) {
        return sqrt(std::pow(v1->i - v2->i, 2) + std::pow(v1->j - v2->j, 2));
    }

    std::list<DLiteNode*> GetNeigh(DLiteNode* v, const Map& map, const EnvironmentOptions& options) {
        std::list<DLiteNode*> to_return;
        for (int i = -1; i <= 1; ++i) {
            for (int j = -1; j <= 1; ++j) {
                if (i == 0 && j == 0) { continue; }
                if (map.CellOnGrid(v->i + i, v->j + j) &&
                    (map.CellIsTraversable(v->i + i, v->j + j) || !map.CellIsVisible(v->i + i, v->j + j))) {
                    if (abs(i) == abs(j)) {
                        if (!options.allowdiagonal) {
                            continue;
                        }
                        if ((!map.CellOnGrid(v->i, v->j + j) ||
                             !map.CellIsTraversable(v->i, v->j + j)) &&
                            map.CellIsVisible(v->i, v->j + j)) {
                            if (!options.cutcorners) {
                                continue;
                            }
                        }
                        if ((!map.CellOnGrid(v->i + i, v->j) ||
                             !map.CellIsTraversable(v->i + i, v->j)) &&
                            map.CellIsVisible(v->i + i, v->j)) {
                            if (!options.cutcorners) {
                                continue;
                            }
                        }
                        if (((!map.CellOnGrid(v->i, v->j + j) ||
                              !map.CellIsTraversable(v->i, v->j + j)) &&
                             map.CellIsVisible(v->i, v->j + j)) &&
                            ((!map.CellOnGrid(v->i + i, v->j) ||
                              !map.CellIsTraversable(v->i + i, v->j)) &&
                             map.CellIsVisible(v->i + i, v->j))) {
                            if (!options.allowsqueeze) {
                                continue;
                            }
                        }
                    }
                    DLiteNode* new_node;
                    if (open_to_find.find({ v->i + i, v->j + j }) == open_to_find.end()) {
                        new_node = new DLiteNode(v->i + i, v->j + j, { -1, -1 });
                    } else {
                        new_node = open_to_find[{ v->i + i, v->j + j }];
                    }
                    to_return.emplace_back(new_node);
                }
            }
        }
        return to_return;
    }

    void PrintInFile(Map& map, std::pair<Node*, Node*> i_path);

    bool IsValid(DLiteNode* v) {
        if (openHash.find({ v->i, v->j }) == openHash.end()) {
            return false;
        }
        MyHash hash;
        if (hash({ v->i, v->j }) != openHash[{ v->i, v->j }]) {
            return false;
        }
        return true;
    }

protected:

    std::set<DLiteNode*, DLiteComparator>                        open_to_get;
    std::unordered_map<std::pair<int, int>, DLiteNode*, MyHash>  open_to_find;
    std::unordered_map<std::pair<int, int>, DLiteNode*, MyHash>  close;
    std::unordered_map<std::pair<int, int>, double, MyHash>       openHash;
    SearchResult                                                 sresult; //This will store the search result
    std::list<DLiteNode>                                         lppath, hppath;
};
#endif
