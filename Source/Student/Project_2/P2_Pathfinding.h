#pragma once
#include "Misc/PathfindingDetails.hpp"


struct Node
{
    Node() = default;
    Node(int row, int col)
    {
        m_parentNode = nullptr;
        m_finalCost = 0;
        m_givenCost = 0;
        m_gridPos.row = row;
        m_gridPos.col = col;
        m_open = false;
        m_close = false;
    }

    Node* m_parentNode;
    float m_finalCost;
    float m_givenCost;

    GridPos m_gridPos;

    bool m_open;
    bool m_close;
};

class AStarPather
{
public:
    /* 
        The class should be default constructible, so you may need to define a constructor.
        If needed, you can modify the framework where the class is constructed in the
        initialize functions of ProjectTwo and ProjectThree.
    */

    /* ************************************************** */
    // DO NOT MODIFY THESE SIGNATURES
    bool initialize();
    void shutdown();
    PathResult compute_path(PathRequest &request);
    /* ************************************************** */

    /*
        You should create whatever functions, variables, or classes you need.
        It doesn't all need to be in this header and cpp, structure it whatever way
        makes sense to you.
    */
   void checkneighbours(Node* neighbour, Node* parent, float length, PathRequest &request);
   void mapchange();
   float heuristic(PathRequest& request, const GridPos& a, const GridPos& b);

    std::vector<std::vector<Node*>> grid;
    std::vector<Node*> openList;
    std::vector<Node*> closeList;

    int mapwidth;
    int mapheight;
    float sq2;


};

