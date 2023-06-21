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

void Node::precomputeneighbours(const int& mapwidth, const int& mapheight)
{
    m_validneighbours = 0;
    bool top = false;
    bool right = false;
    bool left = false;
    bool bottom = false;
    //top
    if (m_gridPos.row + 1 < mapheight)
    {
        if (!terrain->is_wall({ m_gridPos.row + 1, m_gridPos.col }))
        {
            m_validneighbours |= (1 << 7);
        }
        else
        {
            top = true;
        }
    }

    if (m_gridPos.row - 1 >= 0)
    {
        if (!terrain->is_wall({ m_gridPos.row - 1, m_gridPos.col }))
        {
            m_validneighbours |= (1 << 6);
        }
        else
        {
            bottom = true;
        }
    }
    if (m_gridPos.col - 1 >= 0)
    {
        if (!terrain->is_wall({ m_gridPos.row,m_gridPos.col - 1 }))
        {
            m_validneighbours |= (1 << 5);
        }
        else
        {
            left = true;
        } 
    }
    if (m_gridPos.col + 1 < mapwidth)
    {
        if (!terrain->is_wall({ m_gridPos.row,m_gridPos.col + 1 }))
        {
            m_validneighbours |= (1 << 4);
        }
        else
        {
            right = true;
        }
    }

    //topright
    if (!top && !right)
    {
        if (m_gridPos.row + 1 < mapheight && m_gridPos.col + 1 < mapwidth)
        {
            if (!terrain->is_wall({ m_gridPos.row + 1, m_gridPos.col + 1 }))
            {
                m_validneighbours |= (1 << 3);
            }
        }
    }
    //topleft
    if (!top && !left)
    {
        if (m_gridPos.row + 1 < mapheight && m_gridPos.col - 1 >= 0)
        {
            if (!terrain->is_wall({ m_gridPos.row + 1, m_gridPos.col - 1 }))
            {
                m_validneighbours |= (1 << 2);
            }
        }
    }
    //bottomright
    if (!bottom && !right)
    {
        if (m_gridPos.row - 1 >= 0 && m_gridPos.col + 1 < mapwidth)
        {
            if (!terrain->is_wall({ m_gridPos.row - 1,m_gridPos.col + 1 }))
            {
                m_validneighbours |= (1 << 1);
            }
        }
    }
    if (!bottom && !left)
    {
        if (m_gridPos.row - 1 >= 0 && m_gridPos.col - 1 >= 0)
        {
            if (!terrain->is_wall({ m_gridPos.row - 1 , m_gridPos.col - 1 }))
            {
                m_validneighbours |= (1 << 0);
            }
        }
    }


}

void AStarPather::mapchange()
{

    mapwidth = terrain->get_map_width();
    mapheight = terrain->get_map_height();
    for (int i = 0; i < mapwidth; i++)
    {
        for (int j = 0; j < mapwidth; j++)
        {
            grid[i][j]->precomputeneighbours(mapwidth, mapheight);
        }
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
                if (grid[i][j]->dirty)
                {
                    grid[i][j]->m_parentNode = nullptr; 
                    grid[i][j]->m_givenCost = 0;
                    grid[i][j]->m_finalCost = 0;
                    grid[i][j]->m_open = false;
                    grid[i][j]->m_close = false;
                    grid[i][j]->dirty = false;

                }

            }
        }
        openList.ClearList();
        startnode->m_gridPos = start;
        startnode->m_givenCost = 0;
        float heuristic2 = 0;
        float ydiff = (float)std::abs(goal.row - startnode->m_gridPos.row);
        float xdiff = (float)std::abs(goal.col - startnode->m_gridPos.col);
        switch (request.settings.heuristic)
        {
        case Heuristic::MANHATTAN:
        {
            heuristic2 = (float)(xdiff + ydiff);
            break;
        }
        case Heuristic::CHEBYSHEV:
        {
            heuristic2 = (float)std::max(xdiff, ydiff);
            break;
        }
        case Heuristic::EUCLIDEAN:
        {
            heuristic2 = (float)std::sqrt(xdiff * xdiff + ydiff * ydiff);
            break;
        }
        case Heuristic::OCTILE:
        {
            heuristic2 = (float)(std::min(xdiff, ydiff) * sq2 + std::max(xdiff, ydiff) - std::min(xdiff, ydiff));
            break;
        }
        case Heuristic::INCONSISTENT:
        {
            if ((startnode->m_gridPos.row + startnode->m_gridPos.col) % 2 > 0)
            {
                heuristic2 = (float)std::sqrt(xdiff * xdiff + ydiff * ydiff);
            }
            else
            {

                heuristic2 = 0;
            }
            break;
        }
        }
        startnode->m_finalCost = startnode->m_givenCost + heuristic2; 
        startnode->dirty = true;
        startnode->m_open = true;
        openList.Push(startnode);
        if (request.settings.debugColoring)
        {
            terrain->set_color(startnode->m_gridPos, Colors::Blue);
        }
    }
    while (!openList.Empty())
    {
        Node* parentnode = openList.PopCheapest();
        parentnode->m_open = false;

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
            if (request.settings.rubberBanding && request.settings.smoothing)
            {
                rubberbandingsmoothing(request);
            }
            else
            {

                if (request.settings.rubberBanding)
                {
                    rubberbanding(request);
                }
                else if (request.settings.smoothing)
                {
                    smoothing(request);
                }
            }
            

            return PathResult::COMPLETE;

        }

        parentnode->m_close = true;
        
        if (request.settings.debugColoring)
        {
            terrain->set_color(parentnode->m_gridPos, Colors::Yellow);
        }        

        if (parentnode->m_validneighbours & (1 << 7))
        {
            Node* top = grid[parentnode->m_gridPos.row + 1][parentnode->m_gridPos.col];
            if (parentnode->m_parentNode != top)
            {
                checkneighbours(top, parentnode, 1, request);
            }      
        }
        if (parentnode->m_validneighbours & (1 << 6))
        {
            Node* bottom = grid[parentnode->m_gridPos.row - 1][parentnode->m_gridPos.col];
            if (parentnode->m_parentNode != bottom)
            {

                checkneighbours(bottom, parentnode, 1, request);
            }
        }
        if (parentnode->m_validneighbours & (1 << 5))
        {
            Node* left = grid[parentnode->m_gridPos.row][parentnode->m_gridPos.col - 1];
            if (parentnode->m_parentNode != left)
            {
                checkneighbours(left, parentnode, 1, request);
            }
        }
        if (parentnode->m_validneighbours & (1 << 4))
        {
            Node* right = grid[parentnode->m_gridPos.row][parentnode->m_gridPos.col + 1];
            if (parentnode->m_parentNode != right)
            {
                checkneighbours(right, parentnode, 1, request);
            }
        }
        if (parentnode->m_validneighbours & (1 << 3))
        {
            Node* topright = grid[parentnode->m_gridPos.row + 1][parentnode->m_gridPos.col + 1];
            if (parentnode->m_parentNode != topright)
            {
                checkneighbours(topright, parentnode, sq2, request);
            }
        }
        if (parentnode->m_validneighbours & (1 << 2))
        {
            Node* topleft = grid[parentnode->m_gridPos.row + 1][parentnode->m_gridPos.col - 1];
            if (parentnode->m_parentNode != topleft)
            {
                checkneighbours(topleft, parentnode, sq2, request);
            }
        }
        if (parentnode->m_validneighbours & (1 << 1))
        {
            Node* bottomright = grid[parentnode->m_gridPos.row - 1][parentnode->m_gridPos.col + 1];
            if (parentnode->m_parentNode != bottomright)
            {
            checkneighbours(bottomright, parentnode, sq2, request);

            }
        }
        if (parentnode->m_validneighbours & (1 << 0))
        {
            Node* bottomleft = grid[parentnode->m_gridPos.row - 1][parentnode->m_gridPos.col - 1];
            if (parentnode->m_parentNode != bottomleft)
            {

            checkneighbours(bottomleft, parentnode, sq2, request);
            }
        }

        

        if (request.settings.singleStep == true)
        {
            return PathResult::PROCESSING;
        }
    }
   
    return PathResult::IMPOSSIBLE;
}

void AStarPather::rubberbanding(PathRequest& request)
{
    WaypointList& paths = request.path;
    WaypointList::iterator it;
    for (it = paths.begin(); it != paths.end(); ) {
        GridPos first, second, third;
        if (it == paths.end() || std::next(it, 1) == paths.end() || std::next(it, 2) == paths.end())
        {
            break;
        }
        first = terrain->get_grid_position(*it) ;
        second = terrain->get_grid_position(*(std::next(it, 1)));
        third = terrain->get_grid_position(*(std::next(it, 2)));
        
        GridPos max, min; 
        max.row = std::max(first.row, third.row);
        max.col = std::max(first.col, third.col);
        min.row = std::min(first.row, third.row);
        min.col = std::min(first.col, third.col);
        bool tobreak = false;
        for (int i = min.row; i <= max.row; i++)
        {
            for (int j = min.col; j <= max.col; j++)
            {
                if (terrain->is_wall(grid[i][j]->m_gridPos))
                {
                    tobreak = true;
                    break;
                }
                
            }
            if (tobreak)
            {
                break;
            }
        }
        if (tobreak)
        {
            ++it;
            continue;
        }
        paths.erase((std::next(it, 1)));
    }
}

void AStarPather::smoothing(PathRequest& request)
{
    if (request.path.size() < 2)
    {
        return;
    }
    WaypointList& paths = request.path;
    WaypointList::iterator it = paths.begin();
    float t1 = 0.25f;
    float t2 = 0.5f;
    float t3 = 0.75f;
    WaypointList::iterator p1, p2, p3, p4;
    WaypointList::iterator pathend = std::prev(paths.end());
    if (paths.size() == 2)
    {
            p1 = it;
            p2 = it;
            p3 = (std::next(it, 1));
            p4 = (std::next(it, 1));
    }
    else
    {
            p1 = it;
            p2 = it;
            p3 = (std::next(it, 1));
            p4 = (std::next(it, 2));

    }
    
    for (it = paths.begin(); it != paths.end(); ) {
        paths.insert(p3,Vec3::CatmullRom(*p1,*p2,*p3,*p4,t1));
        paths.insert(p3, Vec3::CatmullRom(*p1, *p2, *p3, *p4, t2));
        paths.insert(p3, Vec3::CatmullRom(*p1, *p2, *p3, *p4, t3));

        if (p3 == pathend)
        {
            break;
        }
       
        // if at the start
        if (p2 == p1)
        {
            p2 = p3;
            p3 = p4;
            if (p4 != pathend)
            {
                ++p4;
            }
        }
        //if not at start/ towards end
        else
        {
           
            p1 = p2;
            p2 = p3;
            p3 = p4;
            if (p4 != pathend)
            {
                ++p4;
            }

        }
        
    }

}

void AStarPather::rubberbandingsmoothing(PathRequest& request)
{
    WaypointList& path = request.path;
    if (path.size() <= 2)
    {
        return;
    }

    Vec3 diffsize = terrain->get_world_position(GridPos{ 0,0 }) - terrain->get_world_position(GridPos{ 0,1 });
    float griddistance = 1.5f * diffsize.Length();

    rubberbanding(request);

    WaypointList::iterator it, it2;
    it = path.begin();
    it2 = std::next(it, 1);

    Vec3 distance, midpoint;

    while (it2 != path.end())
    {
        distance = *it - *it2;

        
        if (distance.Length() > griddistance)
        {
            midpoint = ( * it + *it2) * 0.5f;

            path.insert(it2, midpoint);
            --it2;

            if (it == it2)
            {
                ++it;
                ++it2;
            }
        }
        else
        {
            ++it;
            ++it2;
        }
    }

    smoothing(request);
}
