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
    bool operator() (const DLiteNode& l, const DLiteNode& r) const {
        if (l.key.first != r.key.first)
            return l.key.first < r.key.first;
        if (l.key.second != r.key.second)
            return l.key.second < r.key.second;
        if (l.i != r.i) {
            return l.i < r.i;
        }
        return l.j < r.j;
    }
};

struct MyHash {
    double operator()(const DLiteNode &x) const {
        return std::hash<double>()(x.i) ^ std::hash<double>()(x.j);
    }
};

struct MySimpleHash {
    double operator()(std::pair<int, int> x) const {
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
        std::unordered_map<std::pair<int, int>, Node*, MySimpleHash>  open_to_find;
        std::unordered_map<std::pair<int, int>, Node*, MySimpleHash>  close;
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
    std::unordered_map<std::pair<int, int>, Node*, MySimpleHash>  open_to_find;
    std::unordered_map<std::pair<int, int>, Node*, MySimpleHash>  close;
    SearchResult                                            sresult; //This will store the search result
    std::list<Node>                                         lppath, hppath;
};

class DLiteSearch
{
public:
    DLiteSearch();
    ~DLiteSearch(void);

    DLiteSearchResult StartDLiteSearch(ILogger *Logger, Map &Map, const EnvironmentOptions &options);


    double ComputeHeuristic(int i_current, int j_current,
                     int i_finish, int j_finish,
                     const EnvironmentOptions &options);


    int ComputeShortestPath(const Map &map, const EnvironmentOptions &options);

    void UpdateVertex(DLiteNode& v,  const Map &map, const EnvironmentOptions &options);

    std::pair<double, double> CalculateKey(const DLiteNode& v, const Map& map, const EnvironmentOptions& options);
    std::vector<std::pair<int, int>>
    ExpandVisibility(Map &map, const DLiteNode& node);


    double Distance(const DLiteNode& v1, const DLiteNode& v2) {
        return sqrt(std::pow(v1.i - v2.i, 2) + std::pow(v1.j - v2.j, 2));
    }

    std::list<DLiteNode> GetNeigh(const DLiteNode& v, const Map& map, const EnvironmentOptions& options) {
        std::list<DLiteNode> to_return;
        for (int i = -1; i <= 1; ++i) {
            for (int j = -1; j <= 1; ++j) {
                if (i == 0 && j == 0) { continue; }
                if (map.CellOnGrid(v.i + i, v.j + j) &&
                    (map.CellIsTraversable(v.i + i, v.j + j) || !map.CellIsVisible(v.i + i, v.j + j))) {
                    if (abs(i) == abs(j)) {
                        if (!options.allowdiagonal) {
                            continue;
                        }
                        if ((!map.CellOnGrid(v.i, v.j + j) ||
                             !map.CellIsTraversable(v.i, v.j + j)) &&
                            map.CellIsVisible(v.i, v.j + j)) {
                            if (!options.cutcorners) {
                                continue;
                            }
                        }
                        if ((!map.CellOnGrid(v.i + i, v.j) ||
                             !map.CellIsTraversable(v.i + i, v.j)) &&
                            map.CellIsVisible(v.i + i, v.j)) {
                            if (!options.cutcorners) {
                                continue;
                            }
                        }
                        if (((!map.CellOnGrid(v.i, v.j + j) ||
                              !map.CellIsTraversable(v.i, v.j + j)) &&
                             map.CellIsVisible(v.i, v.j + j)) &&
                            ((!map.CellOnGrid(v.i + i, v.j) ||
                              !map.CellIsTraversable(v.i + i, v.j)) &&
                             map.CellIsVisible(v.i + i, v.j))) {
                            if (!options.allowsqueeze) {
                                continue;
                            }
                        }
                    }
                    DLiteNode new_node(v.i + i, v.j + j, { std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity() });
                    if (CreatedNodes.find(new_node) == CreatedNodes.end()) {
                        NodesParam[new_node].g = std::numeric_limits<double>::infinity();
                        NodesParam[new_node].rhs = std::numeric_limits<double>::infinity();
                        MyHash hash;
                        CreatedNodes[new_node] = hash(new_node);
                        sresult.nodescreated++;
                    }
                    to_return.push_back(new_node);
                }
            }
        }
        return to_return;
    }

    void PrintInFile(Map& map, std::pair<Node*, Node*> i_path);

    void NormalizePath();

    std::pair<DLiteNode, double> GetMinNeigh(const DLiteNode& node,  const Map& map, const EnvironmentOptions& options) {
//        auto TBEGIN = std::chrono::system_clock::now();
        auto node_neigh = GetNeigh(node, map, options);
        DLiteNode min_node_parent(-1, -1, {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()});
        double min_rhs = std::numeric_limits<double>::infinity();
        for (auto& node1 : node_neigh) {
            if (min_rhs >= NodesParam[node1].g + ((abs(node1.i - node.i) + abs(node1.j - node.j)) == 2 ? CN_SQRT_TWO : 1)) {
                min_rhs = NodesParam[node1].g + ((abs(node1.i - node.i) + abs(node1.j - node.j)) == 2 ? CN_SQRT_TWO : 1);
                min_node_parent = node1;
            }
        }
//        auto TEND = std::chrono::system_clock::now();
//        std::cout << (TEND - TBEGIN).count() << '\n';
        return { min_node_parent, min_rhs };
    }

    std::deque<DLiteNode> GetAllNotObst(const DLiteNode& current, Map &map, const EnvironmentOptions& options) {
//        auto TBEGIN = std::chrono::system_clock::now();
        std::deque<DLiteNode> result1;
        int i = current.i;
        int j = current.j;
        if(options.allowdiagonal) {
            for (int k = i - 1; k <= i + 1; ++k) {
                for (int l = j - 1; l <= j + 1; ++l) {
                    if (!(k == i && l == j) && map.CellOnGrid(k, l) && map.CellIsTraversable(k, l)) {
                        result1.push_front(DLiteNode(k, l, {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()}));
                    }
                }
            }
        } else {
            for (int k = j - 1; k <= j + 1; ++k)
                if (k != j && map.CellOnGrid(i, k) && map.CellIsTraversable(i, k))
                    result1.push_front(DLiteNode(i, k, {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()}));
            for (int l = i - 1; l <= i + 1; ++l)
                if (l != i && map.CellOnGrid(l, j) && map.CellIsTraversable(l, j))
                    result1.push_front(DLiteNode(l, j, {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()}));
        }
        std::deque<DLiteNode> result;
        for(auto elem : result1) {
            if(CreatedNodes.find(elem) == CreatedNodes.end()) {
                continue;
            } else {
                result.push_back(elem);
            }
        }
//        auto TEND = std::chrono::system_clock::now();
//        std::cout << (TEND - TBEGIN).count() << '\n';
        return result;
    }

    inline bool SomeCheck(const DLiteNode& to, const DLiteNode& from) {
        if (to.j == from.j) {
            return (NodesParent[to] == DLiteNode(from.i, from.j - 1, {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()}) ||
                    NodesParent[to] == DLiteNode(from.i, from.j + 1, {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()}));
        } else if (to.i == from.i) {
            return (NodesParent[to] == DLiteNode(from.i - 1, from.j, {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()}) ||
                    NodesParent[to] == DLiteNode(from.i + 1, from.j, {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()}));
        } else {
            return false;
        }
    }

    void makeSecondaryPath();

protected:

    struct NodeInfo {
        double g = 0;
        double rhs = 0;

        NodeInfo() = default;

        NodeInfo(double other_g, double other_rhs) {
            g = other_g;
            rhs = other_rhs;
        }
    };
    double k_m = 0;
    DLiteNode s_start;
    DLiteNode s_goal;
    DLiteNode s_last;
    DLiteNode s_current;
    std::set<DLiteNode, DLiteComparator>                         open_to_get;
    std::unordered_map<DLiteNode, const DLiteNode*, MyHash>      open_to_find;
    std::unordered_map<DLiteNode, double, MyHash>                CreatedNodes;
    std::unordered_map<DLiteNode, NodeInfo, MyHash>              NodesParam;
    std::unordered_map<DLiteNode, DLiteNode, MyHash>             NodesParent;
    DLiteSearchResult                                                 sresult; //This will store the search result
    std::list<DLiteNode>                                         lppath, hppath;
};
#endif
