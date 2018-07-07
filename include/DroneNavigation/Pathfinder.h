#ifndef PATHFINDER_H
#define PATHFINDER_H

// System Includes
#include <vector>
#include <memory>
#include <cstdint>
#include <functional>
#include <ctime>
#include <cassert>
#include <cstring>
#include <algorithm>
#include <boost/foreach.hpp>

// ROS Includes
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include "BlockAllocator.h"
#include "Vec3Int.h"
#include "Costmap.h"

class BlockAllocator;

using namespace std;

class Pathfinder
{
private:
    enum NodeState
    {
        NOTEXIST,
        IN_OPENLIST,
        IN_CLOSEDLIST
    };

    struct Node
    {
        uint16_t    g;
        uint16_t    h;
        Vec3Int        pos;
        NodeState   state;
        Node*       parent;

        int f() const
        {
            return g + h;
        }

        inline Node(const Vec3Int &pos)
            : g(0), h(0), pos(pos), parent(nullptr), state(NOTEXIST)
        {
        }
    };

public:
    Pathfinder(Costmap* costmap);

    ~Pathfinder()
    {
        clear();
    }

public:
    int get_step_value() const;

    int get_oblique_value() const;

    void set_step_value(int value);

    void set_oblique_value(int value);

    std::vector<Vec3Int> Find(Vec3Int start, Vec3Int end);

private:
    void clear();

private:
    void percolate_up(size_t hole);

    bool get_node_index(Node *node, size_t *index);

    int get_mapping_index(const Vec3Int *pos);

    uint16_t calculate_g_value(Node *parent, const Vec3Int &current);

    uint16_t calculate_h_value(const Vec3Int &current, const Vec3Int &end);

    bool in_open_list(const Vec3Int &pos, Node *&out_node);

    bool in_closed_list(const Vec3Int &pos);

    bool can_pass(const Vec3Int &current, const Vec3Int &destination);

    void find_can_pass_nodes(const Vec3Int &current, std::vector<Vec3Int> *out_lists);

    void handle_found_node(Node *current, Node *destination);

    void handle_not_found_node(Node *current, Node *destination, const Vec3Int &end);


private:
    std::vector<Vec3Int>    directions;
    std::vector<Node*>      mapping;
    std::vector<Node*>      open_list;
    BlockAllocator*         allocator;
    Vec3Int                 last_direction;
    ros::Time               execution_time;

    Costmap*                costmap;
    const int               StepValue = 10;
    const int               ObliqueValue = 14;
    const int               ChangedDirectionValue = 30;
    const double            TimeOut = 5.0;

    int penalty_matrix[3][3][3] = {
        {
            {20, 18, 20},
            {18, 15, 18},
            {20, 18, 20}
        },
        {
            {14, 10, 14},
            {10, 0, 10},
            {14, 10, 14}
        },
        {
            {20, 18, 20},
            {18, 15, 18},
            {20, 18, 20}
        }
    };

};

#endif // PATHFINDER_H
