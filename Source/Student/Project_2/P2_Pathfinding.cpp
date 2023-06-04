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
float AStarPather::heuristic(PathRequest& request,const GridPos& a,const GridPos& b) {
    float ydiff = (float)std::abs(b.row - a.row);
    float xdiff = (float)std::abs(b.col - a.col);
    switch (request.settings.heuristic)
    {
        case Heuristic::MANHATTAN:
        {
            return (float)(xdiff + ydiff);
            break;
        }
        case Heuristic::CHEBYSHEV:
        {
            return (float)std::max(xdiff, ydiff);
            break;
        }
        case Heuristic::EUCLIDEAN:
        {
            return (float)std::sqrt(xdiff*xdiff + ydiff*ydiff);
            break;
        }
        case Heuristic::OCTILE:
        {
            return (float)(std::min(xdiff, ydiff) * std::sqrt(2) + std::max(xdiff, ydiff) - std::min(xdiff, ydiff));
            break;
        }

    }
            
    return 0;
}

void AStarPather::mapchange()
{

    mapwidth = terrain->get_map_width();
    mapheight = terrain->get_map_height();
}

void AStarPather::checkneighbours(Node* neighbour , Node* parent, float length , PathRequest &request)
{
    if (terrain->is_wall(neighbour->m_gridPos))
    {
        return;
    }
    GridPos goal = terrain->get_grid_position(request.goal);
    float temp = parent->m_givenCost +length + heuristic(request,neighbour->m_gridPos, goal);
    if (!neighbour->m_open && !neighbour->m_close)
    {
        openList.push_back(neighbour);
        terrain->set_color(neighbour->m_gridPos, Colors::Blue);
        neighbour->m_open = true;
        neighbour->m_close = false;
        neighbour->m_parentNode = parent;
        neighbour->m_givenCost = parent->m_givenCost + length;
        neighbour->m_finalCost = temp;
    }
    else if (neighbour->m_open)
    {
        for (int i = 0 ; i< openList.size(); i++)
        {
            if (openList[i]->m_gridPos == neighbour->m_gridPos)
            {
                if (temp < openList[i]->m_finalCost)
                {
                    /*neighbour->m_parentNode = parent;
                    openList[i] = neighbour;*/
                    openList[i]->m_parentNode = parent;
                    openList[i]->m_givenCost = parent->m_givenCost + length;
                    openList[i]->m_finalCost = temp;
                    
                    
                    break;

                }
            }
        }
    }
    else if (neighbour->m_close)
    {
        int counter = 0;
        for (int i = 0; i < closeList.size(); i++)
        {
            if (closeList[i]->m_gridPos == neighbour->m_gridPos)
            {
                if (temp < closeList[i]->m_finalCost)
                {
                    counter = i;
                    break;
                }
            }
        }
        closeList.erase(closeList.begin() + counter);
        neighbour->m_parentNode = parent;
        neighbour->m_givenCost = parent->m_givenCost + length;
        neighbour->m_finalCost = temp;
        terrain->set_color(neighbour->m_gridPos, Colors::Blue);
        openList.push_back(neighbour);
        neighbour->m_open = true;
        neighbour->m_close = false;
    
    }
}


bool AStarPather::initialize()
{
    // handle any one-time setup requirements you have
    mapheight = 40;
    mapwidth = 40;
    int mapsize = mapheight * mapwidth;
    grid.reserve(mapsize);
    for (int i = 0; i < mapheight; i++)
    {
        std::vector<Node*> row;
        for (int j = 0; j < mapwidth; j++)
        {
            row.push_back(new Node(i,j)); // initialize gridpos
        }
        grid.push_back(row);
    }
    openList.reserve(mapsize);
    closeList.reserve(mapsize);

    sq2 = (float)std::sqrt(2);
    /*
        If you want to do any map-preprocessing, you'll need to listen
        for the map change message.  It'll look something like this:

        Callback cb = std::bind(&AStarPather::your_function_name, this);
        Messenger::listen_for_message(Messages::MAP_CHANGE, cb);

        There are other alternatives to using std::bind, so feel free to mix it up.
        Callback is just a typedef for std::function<void(void)>, so any std::invoke'able
        object that std::function can wrap will suffice.
    */
    Callback cb = std::bind(&AStarPather::mapchange, this);
    Messenger::listen_for_message(Messages::MAP_CHANGE, cb);

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
    GridPos start = terrain->get_grid_position(request.start);
    Node* startnode = grid[start.row][start.col];
    GridPos goal = terrain->get_grid_position(request.goal);
    Node* goalnode = grid[goal.row][goal.col];
    if (request.newRequest)
    {
        for (int i = 0; i < grid.size(); i++)
        {
            for (int j = 0; j < grid[i].size(); j++)
            {
                grid[i][j]->m_parentNode = nullptr; 
                grid[i][j]->m_givenCost = 0;
                grid[i][j]->m_finalCost = 0;
                grid[i][j]->m_open = false;
                grid[i][j]->m_close = false;

            }
        }
        openList.clear();
        closeList.clear();
        startnode->m_gridPos = start;
        startnode->m_givenCost = 0;
        startnode->m_finalCost = startnode->m_givenCost + heuristic(request, start, goal); // final cost
        openList.emplace_back(startnode);
        terrain->set_color(startnode->m_gridPos, Colors::Blue);
    }
    while (!openList.empty())
    {
        float min = openList[0]->m_finalCost;
        int minindex = 0;
        for (int i = 0; i < openList.size(); i++)
        {
            if (openList[i]->m_finalCost < min)
            {
                minindex = i;
                min = openList[i]->m_finalCost;
            }
        }
        Node* parentnode = openList[minindex];

        openList.erase(openList.begin() +minindex);

        /*
        goalnode->m_gridPos = goal;*/

        if (parentnode == grid[goal.row][goal.col])
        {
            Node* temp = parentnode;
            while (true)
            {

                request.path.emplace_front(terrain->get_world_position(temp->m_gridPos));
                if (temp == startnode)
                {
                
                    break;
                }
                temp = temp->m_parentNode;
            }
            return PathResult::COMPLETE;

        }

        closeList.push_back(parentnode);
        parentnode->m_close = true;
        
        terrain->set_color(parentnode->m_gridPos, Colors::Yellow);
        //check out of bounds
        bool walltop = false;
        bool wallbot = false;
        bool wallright = false;
        bool wallleft = false;
        if (parentnode->m_gridPos.row + 1 < mapheight)
        {
            if (Node* top = grid[parentnode->m_gridPos.row + 1][parentnode->m_gridPos.col])
            {
               /* top->m_givenCost = parentnode->m_givenCost + 1;
                top->m_finalCost = top->m_givenCost - heuristic(*top, *goalnode);*/
                checkneighbours(top, parentnode,1 , request);
                if (terrain->is_wall(top->m_gridPos))
                {
                    walltop = true;
                }

            }
        }
        
        if (parentnode->m_gridPos.row - 1 >= 0)
        {
            if (Node* bottom = grid[parentnode->m_gridPos.row - 1][parentnode->m_gridPos.col])
            {
                /*bottom->m_givenCost = parentnode->m_givenCost + 1;
                bottom->m_finalCost = bottom->m_givenCost - heuristic(*bottom, *goalnode);*/
                checkneighbours(bottom, parentnode ,  1, request);
                if (terrain->is_wall(bottom->m_gridPos))
                {
                    wallbot = true;
                }
            }
        }
        
        if (parentnode->m_gridPos.col - 1 >= 0)
        {
            if (Node* left = grid[parentnode->m_gridPos.row][parentnode->m_gridPos.col - 1])
            {
                /*left->m_givenCost = parentnode->m_givenCost + 1;
                left->m_finalCost = left->m_givenCost - heuristic(*left, *goalnode);*/
                checkneighbours(left, parentnode, 1 ,request);
                if (terrain->is_wall(left->m_gridPos))
                {
                    wallleft = true;
                }
            }
        }
        if (parentnode->m_gridPos.col + 1 < mapwidth)
        {
            if (Node* right = grid[parentnode->m_gridPos.row][parentnode->m_gridPos.col + 1])
            {
                /*right->m_givenCost = parentnode->m_givenCost + 1;
                right->m_finalCost = right->m_givenCost - heuristic(*right, *goalnode);*/
                checkneighbours(right , parentnode,1,request);
                if (terrain->is_wall(right->m_gridPos))
                {
                    wallright = true;
                }
            }
        }

        if (!wallbot && !wallright)
        {
            if (parentnode->m_gridPos.row - 1 >= 0 && parentnode->m_gridPos.col + 1 < mapwidth)
            {
                if (Node* bottomright = grid[parentnode->m_gridPos.row - 1][parentnode->m_gridPos.col + 1])
                {
                    //bottomright->m_givenCost = parentnode->m_givenCost + sq2; //sqrt 2
                    //bottomright->m_finalCost = bottomright->m_givenCost - heuristic(*bottomright, *goalnode);
                    checkneighbours(bottomright, parentnode, sq2, request);

                }
            }
        }
        if(!wallbot && !wallleft)
        {
            if (parentnode->m_gridPos.row - 1 >= 0 && parentnode->m_gridPos.col - 1 >= 0)
            {
                if (Node* bottomleft = grid[parentnode->m_gridPos.row - 1][parentnode->m_gridPos.col - 1])
                {
                    //bottomleft->m_givenCost = parentnode->m_givenCost + sq2; //sqrt 2
                    //bottomleft->m_finalCost = bottomleft->m_givenCost - heuristic(*bottomleft, *goalnode);
                    checkneighbours(bottomleft, parentnode, sq2, request);
                }
            }
        }
        if (!walltop && !wallright)
        {
            if (parentnode->m_gridPos.row + 1 < mapheight && parentnode->m_gridPos.col + 1 < mapwidth)
            {
                if (Node* topright = grid[parentnode->m_gridPos.row + 1][parentnode->m_gridPos.col + 1])
                {
                    //topright->m_givenCost = parentnode->m_givenCost + sq2; //sqrt 2
                    //topright->m_finalCost = topright->m_givenCost - heuristic(*topright, *goalnode);
                    checkneighbours(topright, parentnode, sq2, request);
                }
            }
        }
        if(!walltop && !wallleft)
        {
            if (parentnode->m_gridPos.row + 1 < mapheight && parentnode->m_gridPos.col - 1 >= 0)
            {
                if (Node* topleft = grid[parentnode->m_gridPos.row + 1][parentnode->m_gridPos.col - 1])
                {
                    //topleft->m_givenCost = parentnode->m_givenCost + sq2; //sqrt 2
                    //topleft->m_finalCost = topleft->m_givenCost - heuristic(*topleft, *goalnode);
                    checkneighbours(topleft, parentnode, sq2, request);
                }
            }
        }
        

        if (request.settings.singleStep == true)
        {
            return PathResult::PROCESSING;
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
    /*GridPos start = terrain->get_grid_position(request.start);
    GridPos goal = terrain->get_grid_position(request.goal);
    terrain->set_color(start, Colors::Orange);
    terrain->set_color(goal, Colors::Orange);
    request.path.push_back(request.start);
    request.path.push_back(request.goal);
    return PathResult::COMPLETE;*/
    return PathResult::IMPOSSIBLE;
}
