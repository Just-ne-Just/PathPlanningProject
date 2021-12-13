#include "search.h"

Search::Search()
{
//set defaults here
}

Search::~Search() {
    for (auto& x : open_to_get) {
        delete x.second;
    }
    for (auto& x : close) {
        delete x.second;
    }
}

double Search::ComputeHeuristic(int i_current, int j_current,
                                int i_finish, int j_finish,
                                const EnvironmentOptions &options) {
    if (options.metrictype == 2) {
        return std::sqrt(std::pow(i_current - i_finish, 2) + std::pow(j_current - j_finish, 2));
    } else if (options.metrictype == 1) {
        return abs(i_current - i_finish) + abs(j_current - j_finish);
    } else if (options.metrictype == 3) {
        return std::max(abs(i_current - i_finish), abs(j_current - j_finish));
    } else if (options.metrictype == 0){
        return (abs(i_current - i_finish) + abs(j_current - j_finish)) -
        0.6 * std::min(abs(i_current - i_finish), abs(j_current - j_finish));
    }
    return 0;
}

SearchResult Search::startSearch(ILogger *Logger, const Map &map, const EnvironmentOptions &options)
{
//    std::cout << map.getStart().first << ' ' << map.getStart().second << '\n';
//    std::cout << map.getFinish().first << ' ' << map.getFinish().second << '\n';
    auto H = ComputeHeuristic(map.getStart().first,
                              map.getStart().second,
                              map.getFinish().first,
                              map.getFinish().second,
                              options);
//    std::cout << H << '\n';
    auto TBEGIN = std::chrono::system_clock::now();
    auto begin = new Node{map.getStart().first, map.getStart().second, H, 0, H, nullptr};
    open_to_get.insert({std::to_string(begin->i) + " " + std::to_string(begin->j), begin});
    open_to_find.insert({std::to_string(begin->i) + " " + std::to_string(begin->j), begin});
    sresult.nodescreated++;
    const Node* lastnode = nullptr;
    while (!open_to_get.empty()) {
        sresult.numberofsteps++;
        auto v = *open_to_get.begin();
        //std::cout << open_to_find.size() << ' ' << open_to_get.size() << '\n';

        open_to_get.erase(open_to_get.begin());
        open_to_find.erase(open_to_find.find(std::to_string(v.second->i) + " " + std::to_string(v.second->j)));
        //std::cout << open_to_find.size() << ' ' << open_to_get.size() << '\n';

        close.insert({std::to_string(v.second->i) + " " + std::to_string(v.second->j), v.second});
        if (v.second->i == map.getFinish().first && v.second->j == map.getFinish().second) {
            lastnode = v.second;
            break;
        }
        for (int i = -1; i <= 1; ++i) {
            for (int j = -1; j <= 1; ++j) {
                if (i == 0 && j == 0) { continue; }
                // Diagonal
                if (abs(i) == abs(j)) {
                    if (map.CellOnGrid(v.second->i + i, v.second->j + j) &&
                        map.CellIsTraversable(v.second->i + i, v.second->j + j)) {
                        if (!options.allowdiagonal) {
                            continue;
                        }
                        if (!map.CellOnGrid(v.second->i, v.second->j + j) ||
                            !map.CellIsTraversable(v.second->i, v.second->j + j)) {
                            if (!options.cutcorners) {
                                continue;
                            }
                        }
                        if (!map.CellOnGrid(v.second->i + i, v.second->j) ||
                            !map.CellIsTraversable(v.second->i + i, v.second->j)) {
                            if (!options.cutcorners) {
                                continue;
                            }
                        }
                        if ((!map.CellOnGrid(v.second->i, v.second->j + j) ||
                            !map.CellIsTraversable(v.second->i, v.second->j + j)) &&
                            (!map.CellOnGrid(v.second->i + i, v.second->j) ||
                            !map.CellIsTraversable(v.second->i + i, v.second->j))) {
                            if (!options.allowsqueeze) {
                                continue;
                            }
                        }
                        std::string to_find = std::to_string(v.second->i + i) + " " + std::to_string(v.second->j + j);
                        auto f1 = open_to_find.find(to_find);
                        auto f2 = close.find(to_find);
                        if (f1 == open_to_find.end() && f2 == close.end()) {
                            H = ComputeHeuristic(v.second->i + i, v.second->j + j,
                                                      map.getFinish().first,
                                                      map.getFinish().second,
                                                      options);
                            auto neigh = new Node{v.second->i + i, v.second->j + j, v.second->g + sqrt(2) + H, v.second->g + CN_SQRT_TWO, H, v.second};
                            open_to_find.insert({to_find, neigh});
                            open_to_get.insert({to_find, neigh});
                        } else {

                            if (f1 != open_to_find.end()) {
                                if (f1->second->g > v.second->g + sqrt(2)) {
                                    f1->second->g = v.second->g + sqrt(2);
                                    f1->second->F = f1->second->g + f1->second->H;
                                    f1->second->parent = v.second;
                                }
                            } else if (f2 != close.end()) {
                                continue;
                            }
                        }
                    }
                } else {
                    if (map.CellOnGrid(v.second->i + i, v.second->j + j) && map.CellIsTraversable(v.second->i + i, v.second->j + j)) {
                        std::string to_find = std::to_string(v.second->i + i) + " " + std::to_string(v.second->j + j);
                        auto f1 = open_to_find.find(to_find);
                        auto f2 = close.find(to_find);
                        if (f1 == open_to_find.end() && f2 == close.end()) {
                            H = ComputeHeuristic(v.second->i + i, v.second->j + j,
                                                      map.getFinish().first,
                                                      map.getFinish().second,
                                                      options);
                            auto neigh = new Node{v.second->i + i, v.second->j + j, v.second->g + 1 + H, v.second->g + 1, H, v.second};
                            //std::cout << v.second->i + i << ' ' << v.second->j + j << '\n';
                            //std::cout << open_to_find.size() << ' ' << open_to_get.size() << '\n';

                            open_to_find[to_find] = neigh;
                            if (open_to_get.find({to_find, neigh}) != open_to_get.end()) {
                                std::cout << open_to_get.find({to_find, neigh})->first << ' ' << open_to_get.find({to_find, neigh})->second << '\n';
                            }
                            open_to_get.insert({to_find, neigh});
                            //std::cout << open_to_find.size() << ' ' << open_to_get.size() << '\n';

                        } else {
                            if (f1 != open_to_find.end()) {
                                if (f1->second->g > v.second->g + 1) {
                                    f1->second->g = v.second->g + 1;
                                    f1->second->F = f1->second->g + f1->second->H;
                                    f1->second->parent = v.second;
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
    makePrimaryPath(lastnode, map);
    sresult.pathfound = lastnode;
    sresult.hppath = &hppath; //Here is a constant pointer
    sresult.lppath = &lppath;
    sresult.nodescreated = open_to_get.size() + close.size();
    auto TEND = std::chrono::system_clock::now();
    std::chrono::duration<double> time = (TEND - TBEGIN);
    sresult.time = time.count();
    makeSecondaryPath();
//    std::cout << sresult.pathfound << '\n';
//    std::cout << std::fixed << std::setprecision(8) << sresult.pathlength << '\n';
//    std::cout << sresult.numberofsteps << '\n';
//    std::cout << sresult.nodescreated << '\n';
    return sresult;
}

void Search::makePrimaryPath(const Node* curNode,  const Map &map) {
    while (curNode) {
        lppath.push_front(Node(*curNode));
        if (lppath.size() >= 2) {
            (++lppath.begin())->parent = &(*lppath.begin());
        }
        if (curNode->parent) {
            if (abs(curNode->i - curNode->parent->i) + abs(curNode->j - curNode->parent->j) == 1) {
                sresult.pathlength += 1;
            } else {
                sresult.pathlength += CN_SQRT_TWO;
            }
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
