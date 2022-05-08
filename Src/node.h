#ifndef NODE_H
#define NODE_H

//That's the data structure for storing a single search node.
//You MUST store all the intermediate computations occuring during the search
//incapsulated to Nodes (so NO separate arrays of g-values etc.)

struct Node
{
    int     i, j; //grid cell coordinates
    double  F, g, H; //f-, g- and h-values of the search node
    Node    *parent; //backpointer to the predecessor node (e.g. the node which g-value was used to set the g-velue of the current node)
    Node    *child;
    double rhs;
    std::pair<double, double> key;
};

struct DLiteNode {
    int i = 0;
    int j = 0;
    std::pair<double, double> key = { std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity() };

    DLiteNode() = default;

    DLiteNode(int other_i, int other_j, std::pair<double, double> other_key) {
        i = other_i;
        j = other_j;
        key = other_key;
    }

    DLiteNode(const DLiteNode& other) {
        i = other.i;
        j = other.j;
        key = other.key;
    }

    bool operator== (const DLiteNode& other) const {
        return (i == other.i && j == other.j);
    }

    bool operator!= (const DLiteNode& other) const {
        return (i != other.i || j != other.j);
    }
};

#endif
