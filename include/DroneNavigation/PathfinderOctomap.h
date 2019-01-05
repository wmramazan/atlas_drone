#ifndef PATHFINDER_OCTOMAP_H
#define PATHFINDER_OCTOMAP_H

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

#include <octomap/OcTree.h>
#include "BlockAllocator.h"
#include "Vec3Int.h"

class BlockAllocator;

using namespace std;

class PathfinderOctomap
{
public:
    enum NodeState
    {
        NOTEXIST,
        IN_OPENLIST,
        IN_CLOSEDLIST
    };

    struct Node
    {
        bool occupancy;
        uint16_t    g;
        uint16_t    h;
        Vec3Int     pos;
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
    PathfinderOctomap(octomap::OcTree* octree, double resolution, int size);

    ~PathfinderOctomap()
    {
        clear();
    }

public:
    //
    int get_step_value() const;

    //
    int get_oblique_value() const;

    //
    void set_step_value(int value);

    //
    void set_oblique_value(int value);

    //
    std::vector<Vec3Int> Find(Vec3Int start, Vec3Int end);
    int to_map_index(const Vec3Int position);

private:
    void clear();

private:
    //
    void percolate_up(size_t hole);

    //
    bool get_node_index(Node *node, size_t *index);

    //
    uint16_t calculate_g_value(Node *parent, const Vec3Int &current);

    //
    uint16_t calculate_h_value(const Vec3Int &current, const Vec3Int &end);

    //
    bool in_open_list(const Vec3Int &pos, Node *&out_node);

    //
    bool in_closed_list(const Vec3Int &pos);

    //
    bool can_pass(const Vec3Int &current, const Vec3Int &destination);

    //
    void find_can_pass_nodes(const Vec3Int &current, std::vector<Vec3Int> *out_lists);

    //
    void handle_found_node(Node *current, Node *destination);

    //
    void handle_not_found_node(Node *current, Node *destination, const Vec3Int &end);

    int ToIndex(double value);
    double ToPosition(int value);


private:
    std::vector<Vec3Int>    directions;
    std::vector<Node*>      mapping;
    std::vector<Node*>      open_list;
    BlockAllocator*         allocator;
    Vec3Int                 last_direction;
    ros::Time               execution_time;

    octomap::OcTree*        octree;
    octomap::OcTreeNode*    octreeNode;
    const int               StepValue = 10;
    const int               ChangedDirectionValue = 30;
    const double            TimeOut = 5.0;
    const int               heightPenalty = 1;

    double offset;
    double resolution;
    double occupancy_threshold;
    int size;
    int size_square;
    int size_cube;
    int origin;

    int penalty_matrix[3][3][3] = {
        {
            {20 * heightPenalty, 18 * heightPenalty, 20 * heightPenalty},
            {18 * heightPenalty, 15, 18 * heightPenalty},
            {20 * heightPenalty, 18 * heightPenalty, 20 * heightPenalty}
        },
        {
            {14, 10, 14},
            {10, 0, 10},
            {14, 10, 14}
        },
        {
            {20 * heightPenalty, 18 * heightPenalty, 20 * heightPenalty},
            {18 * heightPenalty, 15, 18 * heightPenalty},
            {20 * heightPenalty, 18 * heightPenalty, 20 * heightPenalty}
        }
    };

};

#endif // PATHFINDER_OCTOMAP_H
