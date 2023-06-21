#include <pch.h>
#include "Terrain/TerrainAnalysis.h"
#include "Terrain/MapMath.h"
#include "Agent/AStarAgent.h"
#include "Terrain/MapLayer.h"
#include "Projects/ProjectThree.h"

#include <iostream>

bool ProjectThree::implemented_fog_of_war() const // extra credit
{
    return false;
}

float distance_to_closest_wall(int row, int col)
{
    /*
        Check the euclidean distance from the given cell to every other wall cell,
        with cells outside the map bounds treated as walls, and return the smallest
        distance.  Make use of the is_valid_grid_position and is_wall member
        functions in the global terrain to determine if a cell is within map bounds
        and a wall, respectively.
    */

    // WRITE YOUR CODE HERE
    float smallestdistance = FLT_MAX;
    
    for (int i = -1; i <= terrain->get_map_width(); i++)
    {
        for (int j = -1; j <= terrain->get_map_height(); j++)
        {
            if (!terrain->is_valid_grid_position(j, i) )
            {
                float ydiff = (float)std::abs(row - j);
                float xdiff = (float)std::abs(col -i);
                smallestdistance = std::min((float)std::sqrt(xdiff * xdiff + ydiff * ydiff),smallestdistance);
            }
            else if (terrain->is_wall(j,i))
            {
                float ydiff = (float)std::abs(row - j);
                float xdiff = (float)std::abs(col - i);
                smallestdistance = std::min((float)std::sqrt(xdiff * xdiff + ydiff * ydiff), smallestdistance);
            }
        }
    }
    return smallestdistance; // REPLACE THIS
}

bool is_clear_path(int row0, int col0, int row1, int col1)
{
    /*
        Two cells (row0, col0) and (row1, col1) are visible to each other if a line
        between their centerpoints doesn't intersect the four boundary lines of every
        wall cell.  You should puff out the four boundary lines by a very tiny amount
        so that a diagonal line passing by the corner will intersect it.  Make use of the
        line_intersect helper function for the intersection test and the is_wall member
        function in the global terrain to determine if a cell is a wall or not.
    */

    // WRITE YOUR CODE HERE
    int max_x, min_x, max_y, min_y;
    max_x = std::max(col0, col1);
    max_y = std::max(row0, row1);
    min_x = std::min(col0, col1);
    min_y = std::min(row0, row1);

    Vec2 start = { (float)row0,(float)col0 };
    Vec2 goal = { (float)row1,(float)col1 };
    
    float offset = 0.1f;

    for (int i = min_y; i <= max_y; i++)
    {
        for (int j = min_x; j <= max_x; j++)
        {
            if (terrain->is_wall(i,j))
            {
                Vec2 topedge0 = { i + 0.5f + offset,j + 0.5f + offset };
                Vec2 topedge1 = { i + 0.5f + offset,j - 0.5f + offset };
                if (line_intersect(start, goal, topedge0, topedge1))
                {
                    return false;
                }
                Vec2 bottomedge0 = { i - 0.5f + offset,j + 0.5f + offset };
                Vec2 bottomedge1 = { i - 0.5f + offset,j - 0.5f + offset };
                if (line_intersect(start, goal, bottomedge0, bottomedge1))
                {
                    return false;
                }
                Vec2 rightedge0 = { i + 0.5f + offset,j + 0.5f + offset };
                Vec2 rightedge1 = { i - 0.5f + offset,j + 0.5f  + offset };
                if (line_intersect(start, goal, rightedge0, rightedge1))
                {
                    return false;
                }
                Vec2 leftedge0 = { i + 0.5f + offset,j - 0.5f + offset };
                Vec2 leftedge1 = { i - 0.5f + offset,j - 0.5f + offset };
                if (line_intersect(start, goal, leftedge0, leftedge1))
                {
                    return false;
                }
            }

        }
    }

    return true; // REPLACE THIS
}

void analyze_openness(MapLayer<float> &layer)
{
    /*
        Mark every cell in the given layer with the value 1 / (d * d),
        where d is the distance to the closest wall or edge.  Make use of the
        distance_to_closest_wall helper function.  Walls should not be marked.
    */

    // WRITE YOUR CODE HERE
    for (int i = 0; i < terrain->get_map_width(); i++)
    {
        for (int j = 0; j < terrain->get_map_height(); j++)
        {
            layer.set_value({ j,i }, 1 / (distance_to_closest_wall(j, i) * distance_to_closest_wall(j, i)));
        }
    }
}

void analyze_visibility(MapLayer<float> &layer)
{
    /*
        Mark every cell in the given layer with the number of cells that
        are visible to it, divided by 160 (a magic number that looks good).  Make sure
        to cap the value at 1.0 as well.

        Two cells are visible to each other if a line between their centerpoints doesn't
        intersect the four boundary lines of every wall cell.  Make use of the is_clear_path
        helper function.
    */

    // WRITE YOUR CODE HERE
    
    for (int i = 0; i < terrain->get_map_width(); i++)
    {
        for (int j = 0; j < terrain->get_map_height(); j++)
        {
            int count = 0;
            float magicnumber = 0.f;
            for (int k = 0; k < terrain->get_map_width(); k++)
            {
                for (int l = 0; l < terrain->get_map_height(); l++)
                {
                    if (is_clear_path(j, i, l, k) && !terrain->is_wall(l,k))
                    {
                        count++;
                    }
                    else if (j == l && i == k)
                    {
                        count++;
                    }
                }
            }
            magicnumber = count / 160.0f;
            if (magicnumber > 1.f)
            {
                magicnumber = 1.f;
            }
            layer.set_value({ j,i }, magicnumber);
        }
    }

}

void analyze_visible_to_cell(MapLayer<float> &layer, int row, int col)
{
    /*
        For every cell in the given layer mark it with 1.0
        if it is visible to the given cell, 0.5 if it isn't visible but is next to a visible cell,
        or 0.0 otherwise.

        Two cells are visible to each other if a line between their centerpoints doesn't
        intersect the four boundary lines of every wall cell.  Make use of the is_clear_path
        helper function.
    */

    // WRITE YOUR CODE HERE

    for (int i = 0; i < terrain->get_map_width(); i++)
    {
        for (int j = 0; j < terrain->get_map_height(); j++)
        {
            
            if (!is_clear_path(row, col, j, i))
            {
                
                layer.set_value({ j,i }, 0.0f);

                
                bool top, bottom, left, right;
                top = false;
                bottom = false;
                left = false;
                right = false;

                //top
                if (terrain->is_valid_grid_position(j + 1, i) && !terrain->is_wall(j+1,i))
                {
                    if (is_clear_path(row, col, j + 1, i))
                    {
                        if (!terrain->is_wall(j, i))
                        {
                            layer.set_value({ j,i }, 0.5f);
                            top = true;
                        }
                    }
                }

                //bottom
                if (terrain->is_valid_grid_position(j - 1, i) && !terrain->is_wall(j - 1, i))
                {
                    if (is_clear_path(row, col, j - 1, i))
                    {
                        if (!terrain->is_wall(j, i))
                        {
                            layer.set_value({ j,i }, 0.5f);
                            bottom = true;
                        }
                    }
                }

                //right
                if (terrain->is_valid_grid_position(j, i + 1) && !terrain->is_wall(j, i +1))
                {
                    if (is_clear_path(row, col, j, i + 1))
                    {
                        if (!terrain->is_wall(j, i))
                        {
                            layer.set_value({ j,i }, 0.5f);
                            right = true;
                        }
                    }
                }

                //left
                if (terrain->is_valid_grid_position(j, i - 1) && !terrain->is_wall(j , i -1))
                {
                    if (is_clear_path(row, col, j, i - 1))
                    {
                        if (!terrain->is_wall(j, i))
                        {
                            layer.set_value({ j,i }, 0.5f);
                            left = true;
                        }
                    }
                }

                //topright
                if (top && right)
                {
                    if (terrain->is_valid_grid_position(j + 1, i + 1) && !terrain->is_wall(j + 1, i + 1))
                    {
                        if (is_clear_path(row, col, j + 1, i + 1))
                        {
                            if (!terrain->is_wall(j, i))
                            {
                                layer.set_value({ j,i }, 0.5f);
                            }

                        }
                    }
                }

                //topleft
                if (top && left)
                {
                    if (terrain->is_valid_grid_position(j + 1, i - 1) && !terrain->is_wall(j + 1, i - 1))
                    {
                        if (is_clear_path(row, col, j + 1, i - 1))
                        {
                            if (!terrain->is_wall(j, i))
                            {
                                layer.set_value({ j,i }, 0.5f);
                            }
                        }
                    }
                }

                //bottomright
                if (bottom && right)
                {
                    if (terrain->is_valid_grid_position(j - 1, i + 1) && !terrain->is_wall(j - 1, i + 1))
                    {
                        if (is_clear_path(row, col, j - 1, i + 1))
                        {
                            if (!terrain->is_wall(j, i))
                            {
                                layer.set_value({ j,i }, 0.5f);
                            }
                        }
                    }
                }

                //bottomleft
                if (bottom && left)
                {
                    if (terrain->is_valid_grid_position(j - 1, i - 1) && !terrain->is_wall(j - 1, i - 1))
                    {
                        if (is_clear_path(row, col, j - 1, i - 1))
                        {
                            if (!terrain->is_wall(j, i))
                            {
                                layer.set_value({ j,i }, 0.5f);
                            }
                        }
                    }
                }
            }
            else
            {
                if (!terrain->is_wall(j, i))
                {
                layer.set_value({ j,i }, 1.f);

                }
            }
            
        }
    }

}

void analyze_agent_vision(MapLayer<float> &layer, const Agent *agent)
{
    /*
        For every cell in the given layer that is visible to the given agent,
        mark it as 1.0, otherwise don't change the cell's current value.

        You must consider the direction the agent is facing.  All of the agent data is
        in three dimensions, but to simplify you should operate in two dimensions, the XZ plane.


        Take the dot product between the view vector and the vector from the agent to the cell,
        both normalized, and compare the cosines directly instead of taking the arccosine to
        avoid introducing floating-point inaccuracy (larger cosine means smaller angle).

        Give the agent a field of view slighter larger than 180 degrees.

        Two cells are visible to each other if a line between their centerpoints doesn't
        intersect the four boundary lines of every wall cell.  Make use of the is_clear_path
        helper function.
    */
    Vec3 vv = agent->get_forward_vector();
    Vec2 view = { vv.z,vv.x };
    view.Normalize();
    //Vec3 agentwrldpos = terrain->get_world_position(agent->get_position().)

    // WRITE YOUR CODE HERE
    for (int i = 0; i < terrain->get_map_width(); i++)
    {
        for (int j = 0; j < terrain->get_map_height(); j++)
        {
            
            Vec3 wrld = terrain->get_world_position( j,i );

            Vec2 dvec = { wrld.z - agent->get_position().z , wrld.x - agent->get_position().x};
            dvec.Normalize();
            float dprod =  dvec.Dot(view);

            //float cosangle = dprod / (view.Length() * dvec.Length());
            if (dprod > 0)
            {
                if (is_clear_path(j, i, terrain->get_grid_position(agent->get_position()).row, terrain->get_grid_position(agent->get_position()).col))
                {
                    layer.set_value(j, i, 1.f);
                }
            }     
        }
    }

}

void propagate_solo_occupancy(MapLayer<float> &layer, float decay, float growth)
{
    /*
        For every cell in the given layer:

            1) Get the value of each neighbor and apply decay factor
            2) Keep the highest value from step 1
            3) Linearly interpolate from the cell's current value to the value from step 2
               with the growing factor as a coefficient.  Make use of the lerp helper function.
            4) Store the value from step 3 in a temporary layer.
               A float[40][40] will suffice, no need to dynamically allocate or make a new MapLayer.

        After every cell has been processed into the temporary layer, write the temporary layer into
        the given layer;
    */
    float temp[40][40];
    for (int a = 0; a < 40 ;a++)
    {
        for (int b = 0; b < 40; b++)
        {
            temp[a][b] = 0;
        }
    }
    
    // WRITE YOUR CODE HERE
    for (int i = 0; i < terrain->get_map_width(); i++)
    {
        for (int j = 0; j < terrain->get_map_height(); j++)
        {
            float highest = 0.f;
            bool top, bottom, left, right;
            top = false;
            bottom = false;
            left = false;
            right = false;
            //top
            if (!terrain->is_wall(j, i) && terrain->is_valid_grid_position(j, i))
            {
                if (terrain->is_valid_grid_position(j + 1, i) && !terrain->is_wall(j + 1, i))
                {
                    float val = layer.get_value(j+1, i);
                    //val *= decay;
                    if (val > highest)
                    {
                        highest = val;
                    }
                    top = true;
                }

                //bottom
                if (terrain->is_valid_grid_position(j - 1, i) && !terrain->is_wall(j - 1, i))
                {
                    float val = layer.get_value(j-1, i);
                    if (val > highest)
                    {
                        highest = val;
                    }
                    bottom = true;
                }

                //right
                if (terrain->is_valid_grid_position(j, i + 1) && !terrain->is_wall(j, i + 1))
                {
                    float val = layer.get_value(j, i+1);
                    if (val > highest)
                    {
                        highest = val;
                    }
                    right = true;
                }

                //left
                if (terrain->is_valid_grid_position(j, i - 1) && !terrain->is_wall(j, i - 1))
                {
                    float val = layer.get_value(j, i-1);
                    if (val > highest)
                    {
                        highest = val;
                    }
                    left = true;
                }

                //topright
                if (top && right)
                {
                    if (terrain->is_valid_grid_position(j + 1, i + 1) && !terrain->is_wall(j + 1, i + 1))
                    {
                        float val = layer.get_value(j + 1, i +1);
                        if (val > highest)
                        {
                            highest = val;
                        }
                    }
                }

                //topleft
                if (top && left)
                {
                    if (terrain->is_valid_grid_position(j + 1, i - 1) && !terrain->is_wall(j + 1, i - 1))
                    {
                        float val = layer.get_value(j + 1, i - 1);
                        if (val > highest)
                        {
                            highest = val;
                        }
                    }
                }

                //bottomright
                if (bottom && right)
                {
                    if (terrain->is_valid_grid_position(j - 1, i + 1) && !terrain->is_wall(j - 1, i + 1))
                    {
                        float val = layer.get_value(j - 1, i + 1);
                        if (val > highest)
                        {
                            highest = val;
                        }
                    }
                }

                //bottomleft
                if (bottom && left)
                {
                    if (terrain->is_valid_grid_position(j - 1, i - 1) && !terrain->is_wall(j - 1, i - 1))
                    {  
                        float val = layer.get_value(j - 1, i - 1);
                        if (val > highest)
                        {
                            highest = val;
                        }
                    }
                }
            }
            temp[j][i] = highest;
        }
    }
    for (int i = 0; i < terrain->get_map_width(); i++)
    {
        for (int j = 0; j < terrain->get_map_height(); j++)
        {
            float lp = lerp(layer.get_value(j, i), temp[j][i], growth);
            layer.set_value({ j,i }, lp);
        }
    }
    
}

void propagate_dual_occupancy(MapLayer<float> &layer, float decay, float growth)
{
    /*
        Similar to the solo version, but the values range from -1.0 to 1.0, instead of 0.0 to 1.0

        For every cell in the given layer:

        1) Get the value of each neighbor and apply decay factor
        2) Keep the highest ABSOLUTE value from step 1
        3) Linearly interpolate from the cell's current value to the value from step 2
           with the growing factor as a coefficient.  Make use of the lerp helper function.
        4) Store the value from step 3 in a temporary layer.
           A float[40][40] will suffice, no need to dynamically allocate or make a new MapLayer.

        After every cell has been processed into the temporary layer, write the temporary layer into
        the given layer;
    */

    // WRITE YOUR CODE HERE
    
}

void normalize_solo_occupancy(MapLayer<float> &layer)
{
    /*
        Determine the maximum value in the given layer, and then divide the value
        for every cell in the layer by that amount.  This will keep the values in the
        range of [0, 1].  Negative values should be left unmodified.
    */


    float max = 0.f;
    // WRITE YOUR CODE HERE
    for (int i = 0; i < terrain->get_map_width(); i++)
    {
        for (int j = 0; j < terrain->get_map_height(); j++)
        {
            if (layer.get_value(j, i) > max)
            {
                max = layer.get_value(j,i);
            }
        }
    }
    for (int i = 0; i < terrain->get_map_width(); i++)
    {
        for (int j = 0; j < terrain->get_map_height(); j++)
        {
            float val = layer.get_value(j, i);
            if (val > 0.f)
            {
            val /= max;
            layer.set_value(j, i, val);
            }
        }
    }

}

void normalize_dual_occupancy(MapLayer<float> &layer)
{
    /*
        Similar to the solo version, but you need to track greatest positive value AND 
        the least (furthest from 0) negative value.

        For every cell in the given layer, if the value is currently positive divide it by the
        greatest positive value, or if the value is negative divide it by -1.0 * the least negative value
        (so that it remains a negative number).  This will keep the values in the range of [-1, 1].
    */

    // WRITE YOUR CODE HERE
}

void enemy_field_of_view(MapLayer<float> &layer, float fovAngle, float closeDistance, float occupancyValue, AStarAgent *enemy)
{
    /*
        First, clear out the old values in the map layer by setting any negative value to 0.
        Then, for every cell in the layer that is within the field of view cone, from the
        enemy agent, mark it with the occupancy value.  Take the dot product between the view
        vector and the vector from the agent to the cell, both normalized, and compare the
        cosines directly instead of taking the arccosine to avoid introducing floating-point
        inaccuracy (larger cosine means smaller angle).

        If the tile is close enough to the enemy (less than closeDistance),
        you only check if it's visible to enemy.  Make use of the is_clear_path
        helper function.  Otherwise, you must consider the direction the enemy is facing too.
        This creates a radius around the enemy that the player can be detected within, as well
        as a fov cone.
    */
    Vec3 enemypos = enemy->get_position();
    Vec2 enem = { enemypos.z,enemypos.x };
    Vec3 enemview = enemy->get_forward_vector();
    Vec2 view = { enemview.z, enemview.x };
    view.Normalize();

    // WRITE YOUR CODE HERE
    for (int i = 0; i < terrain->get_map_width(); i++)
    {
        for (int j = 0; j < terrain->get_map_height(); j++)
        {
            if (!terrain->is_wall(j, i))
            {
                if (layer.get_value(j, i) < 0.f)
                {
                    layer.set_value(j, i, 0.f);
                }

                Vec3 wrld = terrain->get_world_position(j, i);
                Vec2 dist = { wrld.z - enemypos.z , wrld.x - enemypos.x };
                float xdiff = std::abs(dist.x);
                float ydiff = std::abs(dist.y);

                float distance = (float)sqrt((xdiff * xdiff) + (ydiff * ydiff));
                dist.Normalize();

                float angle = view.Dot(dist) / (view.Length() * dist.Length());
                float fov = (float)(fovAngle / 2 * (3.142 / 180));

                float closedist = closeDistance * terrain->mapSizeInWorld / terrain->get_map_width();

                if ((angle > cos(fov) || distance < closedist) && is_clear_path(j, i, terrain->get_grid_position(enemy->get_position()).row, terrain->get_grid_position(enemy->get_position()).col))
                {
                    layer.set_value(j, i, occupancyValue);
                }
            }


            
        }
    }
    

}

bool enemy_find_player(MapLayer<float> &layer, AStarAgent *enemy, Agent *player)
{
    /*
        Check if the player's current tile has a negative value, ie in the fov cone
        or within a detection radius.
    */

    const auto &playerWorldPos = player->get_position();

    const auto playerGridPos = terrain->get_grid_position(playerWorldPos);

    // verify a valid position was returned
    if (terrain->is_valid_grid_position(playerGridPos) == true)
    {
        if (layer.get_value(playerGridPos) < 0.0f)
        {
            return true;
        }
    }

    // player isn't in the detection radius or fov cone, OR somehow off the map
    return false;
}

bool enemy_seek_player(MapLayer<float> &layer, AStarAgent *enemy)
{
    /*
        Attempt to find a cell with the highest nonzero value (normalization may
        not produce exactly 1.0 due to floating point error), and then set it as
        the new target, using enemy->path_to.

        If there are multiple cells with the same highest value, then pick the
        cell closest to the enemy.

        Return whether a target cell was found.
    */

    // WRITE YOUR CODE HERE
    float max = 0.f;
    // WRITE YOUR CODE HERE
    for (int i = 0; i < terrain->get_map_width(); i++)
    {
        for (int j = 0; j < terrain->get_map_height(); j++)
        {
            if (layer.get_value(j, i) > max)
            {
                max = layer.get_value(j, i);
            }
        }
    }
    if(max == 0)
    return false; // REPLACE THIS

    float closestdist = FLT_MAX;
    int indexj = -1;
    int indexi = -1;

    for (int i = 0; i < terrain->get_map_width(); i++)
    {
        for (int j = 0; j < terrain->get_map_height(); j++)
        {
            float influence = layer.get_value(j, i);

            if (influence == max)
            {
                Vec3 wrld = terrain->get_world_position(j, i);
                Vec3 enem = enemy->get_position();

                float dist = (enem - wrld).Length();
                if (dist < closestdist)
                {
                    closestdist = dist;
                    indexj = j;
                    indexi = i;
                }
            }
        }
    }
    
    if (indexj == -1 || indexi == -1)
    {
        return false;
    }
    enemy->path_to(terrain->get_world_position(indexj, indexi));
    return true;
}
