#include "DroneNavigation/Pathfinder.h"

Pathfinder::Pathfinder(GlobalPlanner* global_planner, LocalPlanner* local_planner)
{
    allocator = new BlockAllocator();

    this->global_planner = global_planner;
    this->local_planner = local_planner;

    assert(global_planner->GetMapSize() == local_planner->GetMapSize());

    size = global_planner->GetMapSize();
    size_squared = size * size;
    size_cubed = size_squared * size;

    mapping.resize(size_cubed, nullptr);

    directions.reserve(27);
    std::vector<int> vec = {-1, 0, 1};
    int i, j, k;
    for (i = 0; i < 3; i++)
      for (j = 0; j < 3; j++)
        for (k = 0; k < 3; k++)
          directions.push_back(Vec3Int(vec[i], vec[j], vec[k]));

    last_direction = {0, 0, 0};

    assert(allocator != nullptr);
}

void Pathfinder::clear()
{
    size_t index = 0;

    while (index < size_cubed)
    {
        allocator->Free(mapping[index++], sizeof(Node));
    }
    open_list.clear();

    if (!mapping.empty())
    {
        memset(&mapping[0], 0, sizeof(Node*) * mapping.size());
    }
}

bool Pathfinder::get_node_index(Node *node, size_t *index)
{
    *index = 0;
    const size_t size = open_list.size();
    while (*index < size)
    {
        if (open_list[*index]->pos == node->pos)
        {
            return true;
        }
        ++(*index);
    }
    return false;
}

uint Pathfinder::to_map_index(Vec3Int position)
{
    return position.z * size_squared + position.y * size + position.x;
}

void Pathfinder::percolate_up(size_t hole)
{
    size_t parent = 0;
    while (hole > 0)
    {
        parent = (hole - 1) / 2;
        if (open_list[hole]->f() < open_list[parent]->f())
        {
            std::swap(open_list[hole], open_list[parent]);
            hole = parent;
        }
        else
        {
            return;
        }
    }
}

inline uint16_t Pathfinder::calculate_g_value(Node *parent, const Vec3Int &current)
{
    Vec3Int direction = parent->pos - current;
    uint16_t g_value = penalty_matrix[direction.z + 1][direction.x + 1][direction.y + 1];
    //uint16_t g_value = current.Distance(parent->pos) == 1 ? StepValue : ObliqueValue;
    if (parent->parent)
      last_direction = parent->pos - parent->parent->pos;

    if (last_direction != current - parent->pos)
      g_value += ChangedDirectionValue;

    return g_value += parent->g;
}

inline uint16_t Pathfinder::calculate_h_value(const Vec3Int &current, const Vec3Int &end)
{
    return end.Distance(current) * StepValue;
}

inline bool Pathfinder::in_open_list(const Vec3Int &pos, Node *&out_node)
{
    out_node = mapping[to_map_index(pos)];
    return out_node ? out_node->state == IN_OPENLIST : false;
}

inline bool Pathfinder::in_closed_list(const Vec3Int &pos)
{
    Node *node_ptr = mapping[to_map_index(pos)];
    return node_ptr ? node_ptr->state == IN_CLOSEDLIST : false;
}

bool Pathfinder::can_pass(const Vec3Int &current, const Vec3Int &destination)
{
    if (destination.x >= 0 && destination.x < size
        && destination.y >= 0 && destination.y < size
        && destination.z >= 0 && destination.z < size)
    {
        if (in_closed_list(destination))
        {
            return false;
        }

        return !global_planner->IsOccupied(destination) && !local_planner->IsOccupied(destination);
    }
    return false;
}

void Pathfinder::find_can_pass_nodes(const Vec3Int &current, std::vector<Vec3Int> *out_lists)
{
    Vec3Int destination;
    int index = 0;

    while (index < directions.size())
    {
      destination = current + directions[index];
      if (can_pass(current, destination))
      {
        out_lists->push_back(destination);
      }
      ++index;
    }
}

void Pathfinder::handle_found_node(Node *current, Node *destination)
{
    unsigned int g_value = calculate_g_value(current, destination->pos);
    if (g_value < destination->g)
    {
        destination->g = g_value;
        destination->parent = current;

        size_t index = 0;
        if (get_node_index(destination, &index))
        {
            percolate_up(index);
        }
        else
        {
            assert(false);
        }
    }
}

void Pathfinder::handle_not_found_node(Node *current, Node *destination, const Vec3Int &end)
{
    destination->parent = current;
    destination->h = calculate_h_value(destination->pos, end);
    destination->g = calculate_g_value(current, destination->pos);

    Node *&reference_node = mapping[to_map_index(destination->pos)];
    reference_node = destination;
    reference_node->state = IN_OPENLIST;

    open_list.push_back(destination);
    std::push_heap(open_list.begin(), open_list.end(), [](const Node *a, const Node *b)->bool
    {
        return a->f() > b->f();
    });
}

vector<Vec3Int> Pathfinder::Find(Vec3Int start, Vec3Int end)
{
    vector<Vec3Int> path;

    vector<Vec3Int> nearby_nodes;
    nearby_nodes.reserve(directions.size());

    Node *start_node = new(allocator->Allocate(sizeof(Node))) Node(start);
    open_list.push_back(start_node);

    Node *&reference_node = mapping[to_map_index(start_node->pos)];
    reference_node = start_node;
    reference_node->state = IN_OPENLIST;

    last_direction = {0, 0, 0};

    execution_time = Time::now();
    while (!open_list.empty() && Time::now() - execution_time < Duration(TimeOut))
    {
        Node *current = open_list.front();
        pop_heap(open_list.begin(), open_list.end(), [](const Node *a, const Node *b)->bool
        {
            return a->f() > b->f();
        });
        open_list.pop_back();
        mapping[to_map_index(current->pos)]->state = IN_CLOSEDLIST;

        if (current->pos == end)
        {
            while (current->parent && Time::now() - execution_time < Duration(TimeOut))
            {
                path.push_back(current->pos);
                current = current->parent;
            }

            path.push_back(current->pos);
            reverse(path.begin(), path.end());
            break;
        }

        nearby_nodes.clear();
        find_can_pass_nodes(current->pos, &nearby_nodes);

        size_t index = 0;
        const size_t size = nearby_nodes.size();
        while (index < size && Time::now() - execution_time < Duration(TimeOut))
        {
            Node *next_node = nullptr;
            if (in_open_list(nearby_nodes[index], next_node))
            {
                handle_found_node(current, next_node);
            }
            else
            {
                next_node = new(allocator->Allocate(sizeof(Node))) Node(nearby_nodes[index]);
                handle_not_found_node(current, next_node, end);
            }
            ++index;
        }
    }
    clear();
    return path;
}
