#include "search.h"

Search::Search()
{
//set defaults here
}

Search::~Search() {
    for (auto& x : open_to_get) {
        delete x;
    }
    for (auto& x : close) {
        delete x.second;
    }
}

double Search::ComputeHeuristic(int i_cur, int j_cur, const EnvironmentOptions &options, const Map& map) {
    auto [i_finish, j_finish] = map.getFinish();
    int i_d = abs(i_cur - i_finish), j_d = abs(j_cur - j_finish);
    if (options.metrictype == 0) {
        return double(CN_SQRT_TWO) * std::min(i_d, j_d) + abs(i_d - j_d);
    } else if (options.metrictype == 1) {
        return i_d + j_d;
    } else if (options.metrictype == 2) {
        return sqrt(pow(i_d, 2) + pow(j_d, 2));
    } else if (options.metrictype == 3) {
        return std::max(i_d, j_d);
    }
}

SearchResult Search::startSearch(ILogger *Logger, const Map &map, const EnvironmentOptions &options)
{
//    std::cout << map.getStart().first << ' ' << map.getStart().second << '\n';
//    std::cout << map.getFinish().first << ' ' << map.getFinish().second << '\n';
    auto TBEGIN = std::chrono::system_clock::now();
    auto H = ComputeHeuristic(map.getStart().first,
                              map.getStart().second,
                              options, map);
//    std::cout << H << '\n';
    auto begin = new Node{map.getStart().first, map.getStart().second, H, 0, H, nullptr};
    open_to_get.insert(begin);
    open_to_find.insert({{begin->i, begin->j}, begin});
    sresult.nodescreated++;
    const Node* lastnode = nullptr;
    while (!open_to_get.empty()) {
        sresult.numberofsteps++;
        auto v = *open_to_get.begin();
        //std::cout << open_to_find.size() << ' ' << open_to_get.size() << '\n';

        open_to_get.erase(open_to_get.begin());
        open_to_find.erase(open_to_find.find({v->i, v->j}));
        //std::cout << open_to_find.size() << ' ' << open_to_get.size() << '\n';

        close.insert({{v->i, v->j}, v});
        if (v->i == map.getFinish().first && v->j == map.getFinish().second) {
            lastnode = v;
            break;
        }
        for (int i = -1; i <= 1; ++i) {
            for (int j = -1; j <= 1; ++j) {
                if (i == 0 && j == 0) { continue; }
                // Diagonal
                if (abs(i) == abs(j)) {
                    if (map.CellOnGrid(v->i + i, v->j + j) &&
                        map.CellIsTraversable(v->i + i, v->j + j)) {
                        if (!options.allowdiagonal) {
                            continue;
                        }
                        if (!options.allowdiagonal && map.CellIsObstacle(v->i + i, v->j) && map.CellIsObstacle(v->i, v->j + j)) {
                            continue;
                        }
                        if (!options.cutcorners && (map.CellIsObstacle(v->i + i, v->j) || map.CellIsObstacle(v->i, v->j + j))) {
                            continue;
                        }
                        std::pair<int, int> to_find = {v->i + i, v->j + j};
                        auto f1 = open_to_find.find(to_find);
                        auto f2 = close.find(to_find);
                        if (f1 == open_to_find.end() && f2 == close.end()) {
                            H = ComputeHeuristic(v->i + i, v->j + j,
                                                      options, map);
                            auto neigh = new Node{v->i + i, v->j + j, v->g + double(CN_SQRT_TWO) + H, v->g + double(CN_SQRT_TWO), H, v};
                            open_to_find[to_find] = neigh;
                            open_to_get.insert(neigh);
                        } else {
                            if (f2 != close.end()) {
                                continue;
                            }
                            if (f1 != open_to_find.end()) {
                                if (f1->second->g > v->g + CN_SQRT_TWO) {
                                    f1->second->g = v->g + CN_SQRT_TWO;
                                    f1->second->F = f1->second->g + f1->second->H;
                                    f1->second->parent = v;
                                }
                            }
                        }
                    }
                } else {
                    if (map.CellOnGrid(v->i + i, v->j + j) && map.CellIsTraversable(v->i + i, v->j + j)) {
//                        std::cout << "KEK" << '\n';
                        std::pair<int, int> to_find = {v->i + i, v->j + j};
                        auto f1 = open_to_find.find(to_find);
                        auto f2 = close.find(to_find);
                        if (f1 == open_to_find.end() && f2 == close.end()) {
                            H = ComputeHeuristic(v->i + i, v->j + j,
                                                      options, map);
                            auto neigh = new Node{v->i + i, v->j + j, v->g + 1 + H, v->g + 1, H, v};
                            //std::cout << v->i + i << ' ' << v->j + j << '\n';
                            //std::cout << open_to_find.size() << ' ' << open_to_get.size() << '\n';

                            open_to_find[to_find] = neigh;
                            open_to_get.insert(neigh);
                            //std::cout << open_to_find.size() << ' ' << open_to_get.size() << '\n';

                        } else {
                            if (f2 != close.end()) {
                                continue;
                            }
                            if (f1 != open_to_find.end()) {
                                if (f1->second->g > v->g + 1) {
                                    f1->second->g = v->g + 1;
                                    f1->second->F = f1->second->g + f1->second->H;
                                    f1->second->parent = v;
                                }
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
    sresult.pathlength = lastnode->g;
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
