#include "search.h"

double DLiteSearch::ComputeHeuristic(int i_current, int j_current,
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

void DLiteSearch::NormalizePath() {
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

void DLiteSearch::makeSecondaryPath() {
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

DLiteSearch::DLiteSearch()
{
}

DLiteSearch::~DLiteSearch()
{
}

void PrintSet(const std::set<DLiteNode, DLiteComparator>& x) {
    for (const auto& y : x) {
        std::cout << "COORD " << y.i << ' ' << y.j << '\n';
        std::cout << "KEY " << y.key.first << ' ' << y.key.second << '\n';
    }
    std::cout << "========\n";
}

double CalculatePathLength(const std::list<DLiteNode>& path) {
    double answer = 0;

    for (auto it = path.begin(); it != --path.end(); ++it) {
        auto it1 = it;
        answer += ((abs(it->i - (++it1)->i) + abs(it->j - (++(--it1))->j)) == 2 ? CN_SQRT_TWO : 1);
    }
    return answer;
}

DLiteSearchResult DLiteSearch::StartDLiteSearch(Map &Map, const EnvironmentOptions &options) {
    auto TBEGIN = std::chrono::system_clock::now();
    s_start.i = Map.getStart().first;
    s_start.j = Map.getStart().second;
    s_goal.i = Map.getFinish().first;
    s_goal.j = Map.getFinish().second;
    NodesParam[s_goal] = NodeInfo(std::numeric_limits<double>::infinity(), 0);
    NodesParam[s_start] = NodeInfo(std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
    s_goal.key = CalculateKey(s_goal, Map, options);
    s_last = s_start;
    s_current.i = s_start.i;
    s_current.j = s_start.j;
    auto inserted = open_to_get.insert(s_goal);
    open_to_find[s_goal] = &(*inserted.first);
    MyHash hash;
    CreatedNodes[s_goal] = hash(s_goal);
    CreatedNodes[s_start] = hash(s_start);
    sresult.nodescreated += 2;
    auto to_answer = s_start;

    bool running = true;
    if (!ComputeShortestPath(Map, options)) {
        return DLiteSearchResult();
    }

    while (s_start != s_goal) {
//        PrintSet(open_to_get);
        auto min_val = GetMinNeigh(s_start, Map, options);
        lppath.push_back(s_start);
        if (min_val.first.i == -1) {
            return DLiteSearchResult();
        } else {
            s_start = min_val.first;
        }
        min_val = GetMinNeigh(s_start, Map, options);
        UpdateVertex(s_start, Map, options);
        auto to_update = ExpandVisibility(Map, s_start);
        if (!to_update.empty()) {
            k_m += ComputeHeuristic(s_last.i, s_last.j, s_start.i, s_start.j, options);
            s_last = s_start;
        }
        for (auto obstacle : to_update) {
            DLiteNode obstacle_node(obstacle.first, obstacle.second, {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()});
            if (CreatedNodes.find(obstacle_node) != CreatedNodes.end()) {
//                auto to_erase = MyFind(open_to_get, obstacle_node);
//                if (to_erase != open_to_get.end())
//                    open_to_get.erase(to_erase);
                if (open_to_find[obstacle_node]) {
                    open_to_get.erase(*open_to_find[obstacle_node]);
                    open_to_find[obstacle_node] = nullptr;
                }
                for (auto node : GetAllNotObst(obstacle_node, Map, options)) {
                    if (node != s_goal && (NodesParent[node] == obstacle_node || SomeCheck(node, obstacle_node))) {
                        auto min_val1 = GetMinNeigh(node, Map, options);
                        if (min_val1.first.i == -1) {
//                            to_erase = MyFind(open_to_get, node);
//                            if (to_erase != open_to_get.end()) {
//                                open_to_get.erase(to_erase);
//                            }
                            if (open_to_find[node]) {
                                open_to_get.erase(*open_to_find[node]);
                                open_to_find[node] = nullptr;
                            }
                            if  (node == s_start) {
                                return DLiteSearchResult();
                            }
                        } else {
                            NodesParam[node].rhs = min_val1.second;
                            NodesParent[node] = min_val1.first;
                            UpdateVertex(node, Map, options);
                        }
                    }
                }
            }
        }
        if (ComputeShortestPath(Map, options)) {
            continue;
        } else {
            if (open_to_get.begin()->key == std::pair<double, double>(std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity())) {
                return DLiteSearchResult();
            }
        }
    }
    lppath.push_back(s_goal);
    std::cout << "\n";
    NormalizePath();
    PrintInFile(Map, {nullptr, nullptr});
    sresult.pathfound = true;
    sresult.nodescreated = CreatedNodes.size();
    sresult.memory = sresult.nodescreated * sizeof(s_start);
    sresult.lppath = &lppath;
    makeSecondaryPath();
    sresult.hppath = &hppath;
    sresult.pathlength = CalculatePathLength(lppath);
    auto TEND = std::chrono::system_clock::now();
    std::chrono::duration<double> time = (TEND - TBEGIN);
    sresult.time = time.count();
    return sresult;
}

std::pair<double, double> DLiteSearch::CalculateKey(const DLiteNode& v, const Map& map, const EnvironmentOptions& options)
{
    return { std::min(NodesParam[v].g, NodesParam[v].rhs) +
             ComputeHeuristic(v.i, v.j, s_start.i, s_start.j, options) + k_m,
             std::min(NodesParam[v].g, NodesParam[v].rhs) };
}

void DLiteSearch::UpdateVertex(DLiteNode& v, const Map &map, const EnvironmentOptions &options)
{
//    auto TBEGIN = std::chrono::system_clock::now();
    if (abs(NodesParam[v].g - NodesParam[v].rhs) > EPS) {
        v.key = CalculateKey(v, map, options);
//        auto to_erase = MyFind(open_to_get, v);
//        if (to_erase != open_to_get.end()) {
//            open_to_get.erase(to_erase);
//        }
        if (open_to_find[v]) {
            open_to_get.erase(*open_to_find[v]);
            open_to_find[v] = nullptr;
        }
        auto inserted = open_to_get.insert(v);
        open_to_find[v] = &(*inserted.first);
    } else {
//        auto to_find = MyFind(open_to_get, v);
//        if  (to_find != open_to_get.end()) {
//            open_to_get.erase(to_find);
//        }
        if (open_to_find[v]) {
            open_to_get.erase(*open_to_find[v]);
            open_to_find[v] = nullptr;
        }
    }
//    auto TEND = std::chrono::system_clock::now();
//    std::cout << (TEND - TBEGIN).count() << '\n';
}

int DLiteSearch::ComputeShortestPath(const Map &map, const EnvironmentOptions &options) {
    std::list<DLiteNode> local_neigh;
//    if (open_to_get.empty()) {
//        return 1;
//    }
//    auto TBEGIN = std::chrono::system_clock::now();

    int c = 0;
    while ((!open_to_get.empty()) && ((open_to_get.begin()->key < CalculateKey(s_start, map, options)) || (NodesParam[s_start].rhs != NodesParam[s_start].g))) {
        sresult.numberofsteps++;
        DLiteNode v = *open_to_get.begin();
        auto old_k = v.key;
        auto new_k = CalculateKey(v, map, options);
        if (old_k < new_k) {
            v.key = new_k;
//            auto to_erase = MyFind(open_to_get, v);
//            if (to_erase != open_to_get.end()) {
//                open_to_get.erase(to_erase);
//            }
            if (open_to_find[v]) {
                open_to_get.erase(*open_to_find[v]);
                open_to_find[v] = nullptr;
            }
            auto inserted = open_to_get.insert(v);
            open_to_find[v] = &(*inserted.first);
        } else if (NodesParam[v].g > NodesParam[v].rhs) {
            NodesParam[v].g = NodesParam[v].rhs;
            open_to_find[*open_to_get.begin()] = nullptr;
            open_to_get.erase(open_to_get.begin());
            for (auto& node : GetNeigh(v, map, options)) {
                if (node != s_goal && NodesParam[node].rhs > NodesParam[v].g + ((abs(v.i - node.i) + abs(v.j - node.j)) == 2 ? CN_SQRT_TWO : 1)) {
                    NodesParent[node] = v;
                    NodesParam[node].rhs = NodesParam[v].g + ((abs(v.i - node.i) + abs(v.j - node.j)) == 2 ? CN_SQRT_TWO : 1);
                }

                UpdateVertex(node, map, options);
            }
        } else {
            NodesParam[v].g = std::numeric_limits<double>::infinity();
            local_neigh = GetNeigh(v, map, options);
            local_neigh.push_back(v);
            for (auto& node : local_neigh) {
                if (node != s_goal && NodesParent[node] == v) {
                    auto min_ans = GetMinNeigh(node, map, options);
                    NodesParam[node].rhs = min_ans.second;
                    if (min_ans.first.i != -1) {
                        NodesParent[node] = min_ans.first;
                    }
                }
                UpdateVertex(node, map, options);
            }
        }
    }
//    auto TEND = std::chrono::system_clock::now();
//    std::cout << (TEND - TBEGIN).count() << '\n';
    if (NodesParam[s_start].rhs != std::numeric_limits<double>::infinity()) {
        return true;
    } else {
        return false;
    }
    return false;
}

std::vector<std::pair<int, int>>
DLiteSearch::ExpandVisibility(Map &map, const DLiteNode& node) {
//    auto TBEGIN = std::chrono::system_clock::now();
    std::vector<std::pair<int, int>> to_return_obstacles;
    for (int i = -map.getVisibility(); i <= map.getVisibility(); ++i) {
        for (int j = -map.getVisibility(); j <= map.getVisibility(); ++j) {
            if (map.CellOnGrid(node.i + i, node.j + j) && Distance(node, DLiteNode(node.i + i, node.j + j, {-1, -1})) < map.getVisibility()) {
                if (!map.CellIsVisible(node.i + i, node.j + j) && !map.CellIsTraversable(node.i + i, node.j + j)) {
                    to_return_obstacles.push_back({node.i + i, node.j + j});
                }
                map.makeVisible(node.i + i, node.j + j);
            }
        }
    }
//    auto TEND = std::chrono::system_clock::now();
//    std::cout << (TEND - TBEGIN).count() << '\n';
    return to_return_obstacles;
}

void DLiteSearch::PrintInFile(Map& map, std::pair<Node*, Node*> i_path) {
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
    std::ofstream out;          // поток для записи
    out.open("../../Visual/out", std::ios_base::out); // окрываем файл для записи

    out << map.getVisibility() << '\n';

    for (int i = 0; i < map.getMapHeight(); ++i) {
        for (int j = 0; j < map.getMapWidth(); ++j) {
            out << map_with_path[i][j] << ' ';
        }
        out << '\n';
    }
    out << '\n';

    Node* end = i_path.second;
    if (end) {
        while (end != nullptr) {
            out << end->i << ' ' << end->j << '\n';
            end = end->parent;
        }
    } else {
        for (auto it = lppath.begin(); it != lppath.end(); ++it) {
            out << (*it).i << ' ' << (*it).j << '\n';
        }
    }

    out.close();
}

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

SearchResult Search::startSearch(const Map &map, const EnvironmentOptions &options)
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


SeqSearch::SeqSearch()
{
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

std::pair<Node*, Node*> SeqSearch::searchStep(Map &map, const EnvironmentOptions &options,
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
    open_to_find = std::unordered_map<std::pair<int, int>, Node*, MySimpleHash>();
    close = std::unordered_map<std::pair<int, int>, Node*, MySimpleHash>();
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
    std::ofstream out;          // поток для записи
    out.open("../../Visual/out", std::ios_base::out); // окрываем файл для записи

    out << map.getVisibility() << '\n';

    for (int i = 0; i < map.getMapHeight(); ++i) {
        for (int j = 0; j < map.getMapWidth(); ++j) {
            out << map_with_path[i][j] << ' ';
        }
        out << '\n';
    }
    out << '\n';

    Node* end = i_path.second;
    if (end) {
        while (end != nullptr) {
            out << end->i << ' ' << end->j << '\n';
            end = end->parent;
        }
    } else {
        for (auto it = lppath.begin(); it != lppath.end(); ++it) {
            out << (*it).i << ' ' << (*it).j << '\n';
        }
    }

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

SearchResult SeqSearch::startSeqSearch(Map &map, const EnvironmentOptions &options)
{
    auto TBEGIN = std::chrono::system_clock::now();
    lppath.push_back(Node{map.getStart().first, map.getStart().second, 0, 0, 0, nullptr, nullptr});
    while (true) {
        auto begin_end = searchStep(map, options, lppath.back());
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
    NormalizePath();
    PrintInFile(map, { nullptr, nullptr });
    auto TEND = std::chrono::system_clock::now();
    sresult.lppath = &lppath;
    makeSecondaryPath();
    sresult.hppath = &hppath;
    std::chrono::duration<double> time = (TEND - TBEGIN);
    sresult.time = time.count();
    sresult.pathlength = 10;
    return sresult;
}
