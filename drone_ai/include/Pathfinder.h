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

// Octomap Includes
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/GetOctomap.h>

// PointCloud Includes
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include "BlockAllocator.h"
#include "Vec3.h"
#include "Costmap.h"

class BlockAllocator;

using namespace std;
using namespace octomap_msgs;
using namespace octomap;

class Pathfinder
{
public:
    typedef function<bool(const Vec3&)> Callback;

    struct Params
    {
        uint16_t    height;
        uint16_t    width;
        uint16_t    depth;
        Vec3        start;
        Vec3        end;
        Callback    can_pass;

        Params() : height(0), width(0), depth(0)
        {
        }
    };

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
        Vec3        pos;
        NodeState   state;
        Node*       parent;

        int f() const
        {
            return g + h;
        }

        inline Node(const Vec3 &pos)
            : g(0), h(0), pos(pos), parent(nullptr), state(NOTEXIST)
        {
        }
    };

public:
    Pathfinder(Costmap &costmap, const Params &param);

    ~Pathfinder()
    {
        clear();
    }

public:
    int get_step_value() const;

    int get_oblique_value() const;

    void set_step_value(int value);

    void set_oblique_value(int value);

    std::vector<Vec3> Find(Vec3& start, Vec3& end);

private:
    void clear();

    void init(const Params &param);

    bool is_valid_params(const Params &param);

private:
    void percolate_up(size_t hole);

    bool get_node_index(Node *node, size_t *index);

    int get_mapping_index(const Vec3 *pos);

    uint16_t calculate_g_value(Node *parent, const Vec3 &current);

    uint16_t calculate_h_value(const Vec3 &current, const Vec3 &end);

    bool in_open_list(const Vec3 &pos, Node *&out_node);

    bool in_closed_list(const Vec3 &pos);

    bool can_pass(const Vec3 &current, const Vec3 &destination);

    void find_can_pass_nodes(const Vec3 &current, std::vector<Vec3> *out_lists);

    void handle_found_node(Node *current, Node *destination);

    void handle_not_found_node(Node *current, Node *destination, const Vec3 &end);


private:
    int                     step_val_;
    int                     oblique_val_;
    std::vector<Vec3>       directions;
    std::vector<Node*>      mapping_;
    uint16_t                height_;
    uint16_t                width_;
    uint16_t                depth_;
    uint16_t                wxh_;
    Callback                can_pass_;
    std::vector<Node*>      open_list_;
    BlockAllocator*         allocator;
    Vec3                    last_direction;
    std::clock_t            execution_time;

    Costmap*                costmap;
};

#endif // PATHFINDER_H
