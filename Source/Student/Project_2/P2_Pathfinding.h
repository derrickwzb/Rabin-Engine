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
        dirty = false;
    }

     void precomputeneighbours(const int& mapwidth, const int& mapheight);

    Node* m_parentNode;
    float m_finalCost;
    float m_givenCost;

    GridPos m_gridPos;

    bool m_open;
    bool m_close;
    bool dirty;
    uint8_t m_validneighbours = 0;

};

class FastArray
{
public:

    Node* PopCheapest()
    {
        if (m_lastindex == -1)
        {
            return nullptr;
        }
        float min = m_data[0]->m_finalCost;
        int minindex = 0;
        for (int i = 0; i < m_lastindex; i++)
        {
            if (m_data[i]->m_finalCost < min)
            {
                min = m_data[i]->m_finalCost;
                minindex = i;
            }
        }
        std::swap(m_data[minindex], m_data[m_lastindex]);
        --m_lastindex;
        return m_data[m_lastindex + 1];
    }
    void Push(Node* node)
    {
        ++m_lastindex;
        m_data[m_lastindex] = node;
    }
    bool Empty()
    {
        return (m_lastindex == -1);
    }
    void ClearList()
    {
        m_lastindex = -1;
    }


private:
    short m_lastindex = -1;
    Node* m_data[1600];

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
   inline void checkneighbours(Node* neighbour, Node* parent, float length, PathRequest &request)
   {
       float heuristic1 = 0;
       GridPos goal = terrain->get_grid_position(request.goal);

       float ydiff = (float)std::abs(goal.row - neighbour->m_gridPos.row);
       float xdiff = (float)std::abs(goal.col - neighbour->m_gridPos.col);

       switch (request.settings.heuristic)
       {
       case Heuristic::MANHATTAN:
       {
           heuristic1 = (float)(xdiff + ydiff);
           break;
       }
       case Heuristic::CHEBYSHEV:
       {
           heuristic1 = (float)std::max(xdiff, ydiff);
           break;
       }
       case Heuristic::EUCLIDEAN:
       {
           heuristic1 = (float)std::sqrt(xdiff * xdiff + ydiff * ydiff);
           break;
       }
       case Heuristic::OCTILE:
       {
           heuristic1 = (float)(std::min(xdiff, ydiff) * sq2 + std::max(xdiff, ydiff) - std::min(xdiff, ydiff));
           break;
       }
       case Heuristic::INCONSISTENT:
       {
           if ((neighbour->m_gridPos.row + neighbour->m_gridPos.col) % 2 > 0)
           {
               heuristic1 = (float)std::sqrt(xdiff * xdiff + ydiff * ydiff);
           }
           else
           {

               heuristic1 = 0;
           }
           break;
       }

       }
       float temp = parent->m_givenCost + length + heuristic1;
       if (!neighbour->m_open && !neighbour->m_close)
       {
           openList.Push(neighbour);
           if (request.settings.debugColoring)
           {
               terrain->set_color(neighbour->m_gridPos, Colors::Blue);
           }
           neighbour->m_open = true;
           neighbour->m_close = false;
           neighbour->m_parentNode = parent;
           neighbour->m_givenCost = parent->m_givenCost + length;
           neighbour->m_finalCost = temp;
           neighbour->dirty = true;
       }
       else if (neighbour->m_open)
       {
           if (temp < neighbour->m_finalCost) {
               neighbour->m_parentNode = parent;
               neighbour->m_givenCost = parent->m_givenCost + length;
               neighbour->m_finalCost = temp;
           }
       }
       else if (neighbour->m_close)
       {
           if (temp < neighbour->m_finalCost)
           {
               neighbour->m_parentNode = parent;
               neighbour->m_givenCost = parent->m_givenCost + length;
               neighbour->m_finalCost = temp;
               if (request.settings.debugColoring)
               {
                   terrain->set_color(neighbour->m_gridPos, Colors::Blue);
               }
               openList.Push(neighbour);
               neighbour->m_open = true;
               neighbour->m_close = false;
               neighbour->dirty = true;
           }
       }
   }
   void mapchange();
   /*float heuristic(PathRequest& request, const GridPos& a, const GridPos& b);*/
   void rubberbanding(PathRequest& request);
   void smoothing(PathRequest& request);
   void rubberbandingsmoothing(PathRequest& request);
   

    std::vector<std::vector<Node*>> grid;
    FastArray openList;

    int mapwidth;
    int mapheight;
    float sq2;


};

