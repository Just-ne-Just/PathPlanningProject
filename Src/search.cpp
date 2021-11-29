#include "search.h"

Search::Search()
{
//set defaults here
}

Search::~Search() {
    for (auto x : open) {
        delete x;
    }
    for (auto x : close) {
        delete x;
    }
}

template <typename It>
It Search::findInSet(const It begin, const It end, int search_i, int search_j) {
    for (It it = begin; it != end; ++it) {
        if ((**it).i == search_i && (**it).j == search_j) {
            return it;
        }
    }
    return end;
}

SearchResult Search::startSearch(ILogger *Logger, const Map &map, const EnvironmentOptions &options)
{
    auto TBEGIN = std::chrono::system_clock::now();
    auto begin = new Node{map.getStart().first, map.getStart().second, 0, 0, 0, nullptr};
    open.insert(begin);
    sresult.nodescreated++;
    const Node* lastnode = nullptr;
    while (!open.empty()) {
        sresult.numberofsteps++;
        auto v = *open.begin();
        open.erase(open.begin());
        close.insert(v);
        sresult.nodescreated++;
        if (v->i == map.getFinish().first && v->j == map.getFinish().second) {
            lastnode = v;
            break;
        }
        for (int i = -1; i <= 1; ++i) {
            for (int j = -1; j <= 1; ++j) {
                if (i == 0 && j == 0) { continue; }
                // Diagonal
                if (abs(i) == abs(j)) {
                    if (map.CellOnGrid(v->i + i, v->j + j) && map.CellIsTraversable(v->i + i, v->j + j)) {
                        if (!options.allowdiagonal) {
                            continue;
                        }
                        if (!map.CellOnGrid(v->i, v->j + j) || !map.CellIsTraversable(v->i, v->j + j)) {
                            if (!options.cutcorners) {
                                continue;
                            }
                        }
                        if (!map.CellOnGrid(v->i + i, v->j) || !map.CellIsTraversable(v->i + i, v->j)) {
                            if (!options.cutcorners) {
                                continue;
                            }
                        }
                        if ((!map.CellOnGrid(v->i, v->j + j) || !map.CellIsTraversable(v->i, v->j + j)) &&
                            (!map.CellOnGrid(v->i + i, v->j) || !map.CellIsTraversable(v->i + i, v->j))) {
                            if (!options.allowsqueeze) {
                                continue;
                            }
                        }
                    }
                    auto f1 = findInSet(open.begin(), open.end(), v->i + i, v->j + j);
                    auto f2 = findInSet(close.begin(), close.end(), v->i + i, v->j + j);
                    if (f1 == open.end() && f2 == close.end()) {
                        sresult.nodescreated++;
                        auto neigh = new Node{v->i + i, v->j + j, v->F + 1.4, v->g + 1.4, 0, v};
                        open.insert(neigh);
                    } else {
                        if (f1 != open.end()) {
                            if ((**f1).g > v->g + 1.4) {
                                (**f1).g = v->g + 1.4;
                                (**f1).F = (**f1).g + (**f1).H;
                            }
                        } else if (f2 != close.end()) {
                            continue;
                        }
                    }
                } else {
                    if (map.CellOnGrid(v->i + i, v->j + j) && map.CellIsTraversable(v->i + i, v->j + j)) {
                        auto f1 = findInSet(open.begin(), open.end(), v->i + i, v->j + j);
                        auto f2 = findInSet(close.begin(), close.end(), v->i + i, v->j + j);
                        if (f1 == open.end() && f2 == close.end()) {
                            sresult.nodescreated++;
                            auto neigh = new Node{v->i + i, v->j + j, v->F + 1, v->g + 1, 0, v};
                            open.insert(neigh);
                        } else {
                            if (f1 != open.end()) {
                                if ((**f1).g > v->g + 1) {
                                    (**f1).g = v->g + 1;
                                    (**f1).F = (**f1).g + (**f1).H;
                                    (**f1).parent = v;
                                }
                            } else if (f2 != close.end()) {
                                continue;
                            }
                        }
                    }
                }
            }
        }
    }
    makePrimaryPath(lastnode);
    makeSecondaryPath();
    sresult.pathfound = lastnode;
    sresult.hppath = &hppath; //Here is a constant pointer
    sresult.lppath = &lppath;
    auto TEND = std::chrono::system_clock::now();
    std::chrono::duration<double> time = (TEND - TBEGIN);
    sresult.time = time.count();
    return sresult;
}

void Search::makePrimaryPath(const Node* curNode) {
    while (curNode) {
        ++sresult.pathlength;
        lppath.push_front(Node(*curNode));
        if (lppath.size() >= 2) {
            (++lppath.begin())->parent = &(*hppath.begin());
        }
        curNode = curNode->parent;
    }
}

void Search::makeSecondaryPath() {
    if (lppath.empty()) { return; }
    hppath.push_back(*lppath.begin());
    int direction1 = 0;
    int direction2 = 0;
    for (auto it = ++lppath.begin(); it != lppath.end(); ++it) {
        auto helpIt1 = it;
        auto helpIt2 = --it;
        ++it;
        if (helpIt1->i - helpIt2->i != direction1 || helpIt1->j - helpIt2->j != direction2) {
            direction1 = helpIt1->i - helpIt2->i;
            direction2 = helpIt1->j - helpIt2->j;
            hppath.push_back(*it);
        } else {
            hppath.pop_back();
            hppath.push_back(*it);
        }
    }
}
