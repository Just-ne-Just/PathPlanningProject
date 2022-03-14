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
        return (std::abs(abs(i_current - i_finish) - abs(j_current - j_finish)) +
        std::min(abs(i_current - i_finish), abs(j_current - j_finish)));
    }
    return 0;
}
SearchResult Search::startSearch(ILogger *Logger, const Map &map, const EnvironmentOptions &options)
{
//    std::cout << map.getStart().first << ' ' << map.getStart().second << '\n';
//    std::cout << map.getFinish().first << ' ' << map.getFinish().second << '\n';
    auto TBEGIN = std::chrono::system_clock::now();
    auto H = ComputeHeuristic(map.getStart().first,
                              map.getStart().second,
                              map.getFinish().first,
                              map.getFinish().second,
                              options);
//    std::cout << H << '\n';
    auto begin = new Node{map.getStart().first, map.getStart().second, H, 0, H, nullptr, nullptr};
    open_to_get.insert(begin);
    open_to_find.insert({{begin->i, begin->j}, begin});
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
                        if (!map.CellOnGrid(v->i, v->j + j) ||
                            !map.CellIsTraversable(v->i, v->j + j)) {
                            if (!options.cutcorners) {
                                continue;
                            }
                        }
                        if (!map.CellOnGrid(v->i + i, v->j) ||
                            !map.CellIsTraversable(v->i + i, v->j)) {
                            if (!options.cutcorners) {
                                continue;
                            }
                        }
                        if ((!map.CellOnGrid(v->i, v->j + j) ||
                             !map.CellIsTraversable(v->i, v->j + j)) &&
                            (!map.CellOnGrid(v->i + i, v->j) ||
                             !map.CellIsTraversable(v->i + i, v->j))) {
                            if (!options.allowsqueeze) {
                                continue;
                            }
                        }
                        std::pair<int, int> to_find = {v->i + i, v->j + j};
                        auto f1 = open_to_find.find(to_find);
                        auto f2 = close.find(to_find);
                        if (f1 == open_to_find.end() && f2 == close.end()) {
                            H = ComputeHeuristic(v->i + i, v->j + j,
                                                      map.getFinish().first,
                                                      map.getFinish().second,
                                                      options);
                            auto neigh = new Node{v->i + i, v->j + j, v->g + double(CN_SQRT_TWO) + H, v->g + double(CN_SQRT_TWO), H, v};
                            open_to_find[to_find] = neigh;
                            open_to_get.insert(neigh);
                        } else {
                            if (f2 != close.end()) {
                                continue;
                            }
                            if (f1 != open_to_find.end()) {
                                auto neigh = (*f1).second;
                                if (f1->second->g > v->g + CN_SQRT_TWO) {
                                    open_to_find.erase(f1);
                                    open_to_get.erase(open_to_get.find(neigh));
                                    neigh->g = v->g + CN_SQRT_TWO;
                                    neigh->F = neigh->g + neigh->H;
                                    neigh->parent = v;
                                    open_to_find.insert({{neigh->i, neigh->j}, neigh});
                                    open_to_get.insert(neigh);
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
                                                 map.getFinish().first,
                                                 map.getFinish().second,
                                                 options);
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
                                auto neigh = (*f1).second;
                                if (f1->second->g > v->g + 1) {
                                    open_to_find.erase(f1);
                                    open_to_get.erase(open_to_get.find(neigh));
                                    neigh->g = v->g + 1;
                                    neigh->F = neigh->g + neigh->H;
                                    neigh->parent = v;
                                    open_to_find.insert({{neigh->i, neigh->j}, neigh});
                                    open_to_get.insert(neigh);
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
//    PrintInFile(map);
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

#include "seqsearch.h"

SeqSearch::SeqSearch()
{
//set defaults here
}

SeqSearch::~SeqSearch() {
}

double SeqSearch::ComputeHeuristic(int i_current, int j_current,
                                   int i_finish, int j_finish,
                                   const EnvironmentOptions &options) {
    if (options.metrictype == 2) {
        return std::sqrt(std::pow(i_current - i_finish, 2) + std::pow(j_current - j_finish, 2));
    } else if (options.metrictype == 1) {
        return abs(i_current - i_finish) + abs(j_current - j_finish);
    } else if (options.metrictype == 3) {
        return std::max(abs(i_current - i_finish), abs(j_current - j_finish));
    } else if (options.metrictype == 0){
        return (std::abs(abs(i_current - i_finish) - abs(j_current - j_finish)) +
                std::min(abs(i_current - i_finish), abs(j_current - j_finish)));
    }
    return 0;
}
std::pair<Node*, Node*> SeqSearch::searchStep(ILogger *Logger, Map &map, const EnvironmentOptions &options,
                                              Node startNode)
{
    auto H = ComputeHeuristic(startNode.i,
                              startNode.j,
                              map.getFinish().first,
                              map.getFinish().second,
                              options);
    auto begin = new Node{startNode.i, startNode.j, H, 0, H, nullptr, nullptr};
    open_to_get.insert(begin);
    open_to_find.insert({{begin->i, begin->j}, begin});
    Node* lastnode = nullptr;
    while (!open_to_get.empty()) {
        sresult.numberofsteps++;
        auto v = *open_to_get.begin();

        open_to_get.erase(open_to_get.begin());
        open_to_find.erase(open_to_find.find({v->i, v->j}));
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
                        (map.CellIsTraversable(v->i + i, v->j + j) || !map.CellIsVisible(v->i + i, v->j + j))) {
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
                        std::pair<int, int> to_find = {v->i + i, v->j + j};
                        auto f1 = open_to_find.find(to_find);
                        auto f2 = close.find(to_find);
                        if (f1 == open_to_find.end() && f2 == close.end()) {
                            H = ComputeHeuristic(v->i + i, v->j + j,
                                                 map.getFinish().first,
                                                 map.getFinish().second,
                                                 options);
                            auto neigh = new Node{v->i + i, v->j + j, v->g + double(CN_SQRT_TWO) + H,
                                                  v->g + double(CN_SQRT_TWO), H, v, nullptr};
                            v->child = neigh;
                            open_to_find[to_find] = neigh;
                            open_to_get.insert(neigh);
                        } else {
                            if (f2 != close.end()) {
                                continue;
                            }
                            if (f1 != open_to_find.end()) {
                                auto neigh = (*f1).second;
                                if (f1->second->g > v->g + CN_SQRT_TWO) {
                                    open_to_find.erase(f1);
                                    open_to_get.erase(open_to_get.find(neigh));
                                    neigh->g = v->g + CN_SQRT_TWO;
                                    neigh->F = neigh->g + neigh->H;
                                    neigh->parent = v;
                                    v->child = neigh;
                                    open_to_find.insert({{neigh->i, neigh->j}, neigh});
                                    open_to_get.insert(neigh);
                                }
                            }
                        }
                    }
                } else {
                    if (map.CellOnGrid(v->i + i, v->j + j) && (map.CellIsTraversable(v->i + i, v->j + j) ||
                        !map.CellIsVisible(v->i + i, v->j + j))) {
                        std::pair<int, int> to_find = {v->i + i, v->j + j};
                        auto f1 = open_to_find.find(to_find);
                        auto f2 = close.find(to_find);
                        if (f1 == open_to_find.end() && f2 == close.end()) {
                            H = ComputeHeuristic(v->i + i, v->j + j,
                                                 map.getFinish().first,
                                                 map.getFinish().second,
                                                 options);
                            auto neigh = new Node{v->i + i, v->j + j, v->g + 1 + H, v->g + 1, H, v, nullptr};
                            v->child = neigh;
                            open_to_find[to_find] = neigh;
                            open_to_get.insert(neigh);
                        } else {
                            if (f2 != close.end()) {
                                continue;
                            }
                            if (f1 != open_to_find.end()) {
                                auto neigh = (*f1).second;
                                if (f1->second->g > v->g + 1) {
                                    open_to_find.erase(f1);
                                    open_to_get.erase(open_to_get.find(neigh));
                                    neigh->g = v->g + 1;
                                    neigh->F = neigh->g + neigh->H;
                                    neigh->parent = v;
                                    v->child = neigh;
                                    open_to_find.insert({{neigh->i, neigh->j}, neigh});
                                    open_to_get.insert(neigh);
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    sresult.pathfound = lastnode;
    sresult.nodescreated += open_to_get.size() + close.size();
    std::pair<Node*, Node*> to_return;
    to_return.first = begin;
    to_return.second = lastnode;
    return to_return;
}

void SeqSearch::makePrimaryPath(const Node* curNode, Map &map) {
    while (curNode) {
        lppath.push_front(Node(*curNode));
        if (lppath.size() >= 2) {
            (++lppath.begin())->parent = &(*lppath.begin());
        }
        curNode = curNode->parent;
    }
}

void SeqSearch::makeSecondaryPath() {
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

void SeqSearch::localClear() {
    for (auto& x : open_to_get) {
        delete x;
    }
    for (auto& x : close) {
        delete x.second;
    }
    open_to_get = std::set<Node*, Comparator>();
    open_to_find = std::unordered_map<std::pair<int, int>, Node*, MyHash>();
    close = std::unordered_map<std::pair<int, int>, Node*, MyHash>();
}

void SeqSearch::ExpandVisibility(Map &map) {
    for (int i = -map.getVisibility(); i <= map.getVisibility(); ++i) {
        for (int j = -map.getVisibility(); j <= map.getVisibility(); ++j) {
            if (map.CellOnGrid(lppath.back().i + i, lppath.back().j + j)) {
                map.makeVisible(lppath.back().i + i, lppath.back().j + j);
            }
        }
    }
}

void SeqSearch::PrintInFile(Map& map, std::pair<Node*, Node*> i_path) {
    std::vector<std::vector<int>> map_with_path;
    map_with_path.resize(map.getMapHeight());
    for (int i = 0; i < map.getMapHeight(); ++i) {
        map_with_path[i].resize(map.getMapWidth());
    }
    for (int i = 0; i < map.getMapHeight(); ++i) {
        for (int j = 0; j < map.getMapWidth(); ++j) {
            map_with_path[i][j] = map.getValue(i, j);
        }
    }
    Node* end = i_path.second;
    if (end) {
        while (end != nullptr) {
            std::cerr << end->i << ' ' << end->j << '\n';
            map_with_path[end->i][end->j] = 2;
            end = end->parent;
        }
    } else {
        for (auto it = lppath.begin(); it != lppath.end(); ++it) {
            map_with_path[(*it).i][(*it).j] = 2;
        }
    }
    std::ofstream out;          // поток для записи
    out.open("./out", std::ios_base::app); // окрываем файл для записи
    for (int i = 0; i < map.getMapHeight(); ++i) {
        for (int j = 0; j < map.getMapWidth(); ++j) {
            out << map_with_path[i][j] << ' ';
        }
        out << '\n';
    }
    out << '\n';

    out.close();
}

void SeqSearch::NormalizePath() {
    for (auto it1 = lppath.begin(); it1 != (--lppath.end()); ++it1) {
        for (auto _ = it1; _ != (--lppath.end()); ++_) {
            auto it2 = _;
            ++it2;
            if (it2->i == it1->i && it2->j == it1->j) {
                auto to_erase_l = it1;
                to_erase_l++;
                lppath.erase(to_erase_l, ++it2);
                break;
            }
        }
    }
}

SearchResult SeqSearch::startSeqSearch(ILogger *Logger, Map &map, const EnvironmentOptions &options)
{
    auto TBEGIN = std::chrono::system_clock::now();
    lppath.push_back(Node{map.getStart().first, map.getStart().second, 0, 0, 0, nullptr, nullptr});
    while (true) {
        auto begin_end = searchStep(Logger, map, options, lppath.back());
        if (!begin_end.second) {
            std::cerr << "Not found\n" << std::endl;
            localClear();
            return sresult;
        }
        Node *end = begin_end.second;
        Node *first = nullptr;
        Node *second = nullptr;
        while (end != nullptr) {
            second = first;
            first = end;
            end = end->parent;
        }
        if (second) {
            lppath.emplace_back(Node(*second));
        } else {
            localClear();
            break;
        }
        ExpandVisibility(map);
        localClear();
    }
    auto TEND = std::chrono::system_clock::now();
    NormalizePath();
    PrintInFile(map, { nullptr, nullptr });
    sresult.lppath = &lppath;
//    sresult.pathlength = lppath.size();
    std::chrono::duration<double> time = (TEND - TBEGIN);
    sresult.time = time.count();
    return sresult;
}