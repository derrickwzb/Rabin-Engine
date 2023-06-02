#include <pch.h>
#include "Projects/ProjectTwo.h"
#include "P2_Pathfinding.h"

#pragma region Extra Credit
bool ProjectTwo::implemented_floyd_warshall()
{
    return false;
}

bool ProjectTwo::implemented_goal_bounding()
{
    return false;
}

bool ProjectTwo::implemented_jps_plus()
{
    return false;
}
#pragma endregion

//Node::Node()
//{
//    parent = nullptr;
//    cost = 0;
//}

inline double heuristic(Node a, Node b) {
    return std::abs(a.m_gridPos.row - b.m_gridPos.row) + std::abs(a.m_gridPos.col - b.m_gridPos.col);
}

bool AStarPather::initialize()
{
    // handle any one-time setup requirements you have

    int mapsize = terrain->get_map_height() * terrain->get_map_width();
    for (int i = 0; i < terrain->get_map_width(); i++)
    {
        for (int j = 0; j < terrain->get_map_height(); j++)
        {
            grid[i].push_back(new Node());
        }
    }
    openList.reserve(mapsize);
    closeList.reserve(mapsize);
    /*
        If you want to do any map-preprocessing, you'll need to listen
        for the map change message.  It'll look something like this:

        Callback cb = std::bind(&AStarPather::your_function_name, this);
        Messenger::listen_for_message(Messages::MAP_CHANGE, cb);

        There are other alternatives to using std::bind, so feel free to mix it up.
        Callback is just a typedef for std::function<void(void)>, so any std::invoke'able
        object that std::function can wrap will suffice.
    */

    return true; // return false if any errors actually occur, to stop engine initialization
}

void AStarPather::shutdown()
{
    /*
        Free any dynamically allocated memory or any other general house-
        keeping you need to do during shutdown.
    */
}

PathResult AStarPather::compute_path(PathRequest &request)
{
    /*If(request.newRequest) {
        Initialize everything.Clear Open / Closed Lists.
            Push Start Node onto the Open List with cost of f(x) = g(x) + h(x).
    }
    While(Open List is not empty) {
        parentNode = Pop cheapest node off Open List.
            If parentNode is the Goal Node, then path found(return PathResult::COMPLETE).
            Place parentNode on the Closed List.
            For(all neighboring child nodes of parentNode) {
            Compute its cost, f(x) = g(x) + h(x)
                If child node isn’t on Open or Closed list, put it on Open List.
                Else if child node is on Open or Closed List, AND this new one is cheaper,
                then take the old expensive one off both listsand put this new
                cheaper one on the Open List.
        }
        If taken too much time this frame(or if request.settings.singleStep == true),
            abort search for nowand resume next frame(return PathResult::PROCESSING).
    }
    Open List empty, thus no path possible(return PathResult::IMPOSSIBLE).*/
    if (request.newRequest)
    {
        openList.clear();
        closeList.clear();
        GridPos start = terrain->get_grid_position(request.start);
        Node* startnode = grid[start.row][start.col];
        startnode->m_gridPos = start;
        startnode->m_givenCost = 0;
        startnode->m_finalCost = 0; // final cost
        startnode->m_parentNode = nullptr;
        openList.emplace_back(startnode);
    }
    while (!openList.empty())
    {
        Node* parentnode = openList.back();

        openList.pop_back();

        GridPos goal = terrain->get_grid_position(request.goal);
        Node* goalnode = grid[goal.row][goal.col];
        goalnode->m_gridPos = goal;

        if (parentnode == grid[goal.row][goal.col])
        {
            return PathResult::COMPLETE;
        }

        closeList.push_back(parentnode);
        if (Node* top = grid[parentnode->m_gridPos.row + 1][parentnode->m_gridPos.col])
        {
            top->m_finalCost = top->m_givenCost - heuristic(*top, *goalnode);
        }

        if (Node* topright = grid[parentnode->m_gridPos.row + 1][parentnode->m_gridPos.col + 1])
        {
            topright->m_finalCost = topright->m_givenCost - heuristic(*topright, *goalnode);
        }

        if (Node* topleft = grid[parentnode->m_gridPos.row + 1][parentnode->m_gridPos.col - 1])
        {
            topleft->m_finalCost = topleft->m_givenCost - heuristic(*topleft, *goalnode);
        }

        if (Node* bottom = grid[parentnode->m_gridPos.row - 1][parentnode->m_gridPos.col])
        {
            bottom->m_finalCost = bottom->m_givenCost - heuristic(*bottom, *goalnode);
        }

        if (Node* bottomright = grid[parentnode->m_gridPos.row - 1][parentnode->m_gridPos.col+1])
        {
            bottomright->m_finalCost = bottomright->m_givenCost - heuristic(*bottomright, *goalnode);
        }
        if (Node* bottomleft = grid[parentnode->m_gridPos.row - 1][parentnode->m_gridPos.col - 1])
        {
            bottomleft->m_finalCost = bottomleft->m_givenCost - heuristic(*bottomleft, *goalnode);
        }

        if (Node* left = grid[parentnode->m_gridPos.row ][parentnode->m_gridPos.col-1])
        {
            left->m_finalCost = left->m_givenCost - heuristic(*left, *goalnode);
        }

        if (Node* right = grid[parentnode->m_gridPos.row][parentnode->m_gridPos.col + 1])
        {
            right->m_finalCost = right->m_givenCost - heuristic(*right, *goalnode);
        }
        


    }
    /*
        This is where you handle pathing requests, each request has several fields:

        start/goal - start and goal world positions
        path - where you will build the path upon completion, path should be
            start to goal, not goal to start
        heuristic - which heuristic calculation to use
        weight - the heuristic weight to be applied
        newRequest - whether this is the first request for this path, should generally
            be true, unless single step is on

        smoothing - whether to apply smoothing to the path
        rubberBanding - whether to apply rubber banding
        singleStep - whether to perform only a single A* step
        debugColoring - whether to color the grid based on the A* state:
            closed list nodes - yellow
            open list nodes - blue

            use terrain->set_color(row, col, Colors::YourColor);
            also it can be helpful to temporarily use other colors for specific states
            when you are testing your algorithms

        method - which algorithm to use: A*, Floyd-Warshall, JPS+, or goal bounding,
            will be A* generally, unless you implement extra credit features

        The return values are:
            PROCESSING - a path hasn't been found yet, should only be returned in
                single step mode until a path is found
            COMPLETE - a path to the goal was found and has been built in request.path
            IMPOSSIBLE - a path from start to goal does not exist, do not add start position to path
    */

    // WRITE YOUR CODE HERE

    
    // Just sample code, safe to delete
    GridPos start = terrain->get_grid_position(request.start);
    GridPos goal = terrain->get_grid_position(request.goal);
    terrain->set_color(start, Colors::Orange);
    terrain->set_color(goal, Colors::Orange);
    request.path.push_back(request.start);
    request.path.push_back(request.goal);
    return PathResult::COMPLETE;
}
