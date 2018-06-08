#include "Pathfinder.h"

static const int StepValue = 10;
static const int ObliqueValue = 14;
static const int ChangedDirectionValue = 30;
static const double TimeOut = 100000.0;

Pathfinder::Pathfinder(Costmap &costmap, const Params &param)
    : width_(param.width)
    , height_(param.height)
    , depth_(param.depth)
    , step_val_(StepValue)
    , oblique_val_(ObliqueValue)
{
    allocator = new BlockAllocator();

    wxh_ = width_ * height_;
    if (!mapping_.empty())
    {
        memset(&mapping_[0], 0, sizeof(Node*) * mapping_.size());
    }
    mapping_.resize(wxh_ * depth_, nullptr);

    /*directions.reserve(27);
    std::vector<int> vec = {-1, 0, 1};
    int i, j, k;
    for (i = 0; i < 3; i++)
      for (j = 0; j < 3; j++)
        for (k = 0; k < 3; k++)
          directions.push_back(Vec3(vec[i], vec[j], vec[k]));*/

    directions =
    {
        {-1, 0, 0}, {1, 0, 0},
        {0, -1, 0}, {0, 1, 0},
        {0, 0, -1}, {0, 0, 1}
    };

    last_direction = {0, 0, 0};

    assert(allocator != nullptr);
}

int Pathfinder::get_step_value() const
{
    return step_val_;
}

int Pathfinder::get_oblique_value() const
{
    return oblique_val_;
}

void Pathfinder::set_step_value(int value)
{
    step_val_ = value;
}

void Pathfinder::set_oblique_value(int value)
{
    oblique_val_ = value;
}

void Pathfinder::clear()
{
    size_t index = 0;
    const size_t max_size = width_ * height_ * depth_;
    while (index < max_size)
    {
        allocator->Free(mapping_[index++], sizeof(Node));
    }
    open_list_.clear();
    can_pass_ = nullptr;
    width_ = height_ = depth_ = 0;
}

bool Pathfinder::is_valid_params(const Pathfinder::Params &param)
{
    return (param.can_pass != nullptr
            && (param.width > 0 && param.height > 0 && param.depth > 0)
            //&& (param.end.x >= 0 && param.end.x < param.width)
            //&& (param.end.y >= 0 && param.end.y < param.height)
            //&& (param.start.x >= 0 && param.start.x < param.width)
            //&& (param.start.y >= 0 && param.start.y < param.height)
            );
}

bool Pathfinder::get_node_index(Node *node, size_t *index)
{
    *index = 0;
    const size_t size = open_list_.size();
    while (*index < size)
    {
        if (open_list_[*index]->pos == node->pos)
        {
            return true;
        }
        ++(*index);
    }
    return false;
}

int Pathfinder::get_mapping_index(const Vec3 *pos)
{
  return pos->z * wxh_ + pos->y * width_ + pos->x;
}

void Pathfinder::percolate_up(size_t hole)
{
    size_t parent = 0;
    while (hole > 0)
    {
        parent = (hole - 1) / 2;
        if (open_list_[hole]->f() < open_list_[parent]->f())
        {
            std::swap(open_list_[hole], open_list_[parent]);
            hole = parent;
        }
        else
        {
            return;
        }
    }
}

inline uint16_t Pathfinder::calculate_g_value(Node *parent, const Vec3 &current)
{
    uint16_t g_value = current.Distance(parent->pos) == 1 ? step_val_ : oblique_val_;
    if (parent->parent)
      last_direction = parent->pos - parent->parent->pos;

    if (last_direction != current - parent->pos)
      g_value += ChangedDirectionValue;

    return g_value += parent->g;
}

inline uint16_t Pathfinder::calculate_h_value(const Vec3 &current, const Vec3 &end)
{
    unsigned int h_value = end.Distance(current);
    return h_value * step_val_;
}

inline bool Pathfinder::in_open_list(const Vec3 &pos, Node *&out_node)
{
    out_node = mapping_[get_mapping_index(&pos)];
    return out_node ? out_node->state == IN_OPENLIST : false;
}

inline bool Pathfinder::in_closed_list(const Vec3 &pos)
{
    Node *node_ptr = mapping_[get_mapping_index(&pos)];
    return node_ptr ? node_ptr->state == IN_CLOSEDLIST : false;
}

bool Pathfinder::can_pass(const Vec3 &current, const Vec3 &destination)
{
    if (destination.x >= 0 && destination.x < width_
        && destination.y >= 0 && destination.y < height_
        && destination.z >= 0 && destination.z < depth_)
    {
        if (in_closed_list(destination))
        {
            return false;
        }

        return can_pass_(destination);
    }
    return false;
}

void Pathfinder::find_can_pass_nodes(const Vec3 &current, std::vector<Vec3> *out_lists)
{
    Vec3 destination;
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

void Pathfinder::handle_not_found_node(Node *current, Node *destination, const Vec3 &end)
{
    destination->parent = current;
    destination->h = calculate_h_value(destination->pos, end);
    destination->g = calculate_g_value(current, destination->pos);

    Node *&reference_node = mapping_[get_mapping_index(&destination->pos)];
    reference_node = destination;
    reference_node->state = IN_OPENLIST;

    open_list_.push_back(destination);
    std::push_heap(open_list_.begin(), open_list_.end(), [](const Node *a, const Node *b)->bool
    {
        return a->f() > b->f();
    });
}

vector<Vec3> Pathfinder::Find(Vec3& start, Vec3& end)
{
    vector<Vec3> paths;

    vector<Vec3> nearby_nodes;
    nearby_nodes.reserve(directions.size());

    Node *start_node = new(allocator->Allocate(sizeof(Node))) Node(start);
    open_list_.push_back(start_node);

    Node *&reference_node = mapping_[get_mapping_index(&start_node->pos)];
    reference_node = start_node;
    reference_node->state = IN_OPENLIST;

    execution_time = clock();

    while (!open_list_.empty() && clock() - execution_time < TimeOut)
    {
        Node *current = open_list_.front();
        pop_heap(open_list_.begin(), open_list_.end(), [](const Node *a, const Node *b)->bool
        {
            return a->f() > b->f();
        });
        open_list_.pop_back();
        mapping_[get_mapping_index(&current->pos)]->state = IN_CLOSEDLIST;

        if (current->pos == end)
        {
            while (current->parent)
            {
                paths.push_back(current->pos);
                current = current->parent;
            }
            reverse(paths.begin(), paths.end());
            goto __end__;
        }

        nearby_nodes.clear();
        find_can_pass_nodes(current->pos, &nearby_nodes);

        size_t index = 0;
        const size_t size = nearby_nodes.size();
        while (index < size)
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

__end__:
    clear();
    return paths;
}
