#include "graph.hpp"
#include "timings.hpp"

#include <algorithm>
#include <iostream>
#include <list>
#include <map>
#include <set>
#include <stack>
#include <utility>
#include <vector>

using namespace std;

class Solution
{
public:
    Solution(std::vector<int> buffer_pos, double capacitance, double rat)
        : capacitance{capacitance}
        , rat{rat}
        , prev_buffer_pos{buffer_pos}
    {
        std::cout << "started: " << __PRETTY_FUNCTION__ << std::endl;
    }

    bool is_inferior(Solution const& other_solution) const
    {
        std::cout << "started: " << __PRETTY_FUNCTION__ << std::endl;
        return other_solution.capacitance <= capacitance && other_solution.rat >= rat;
    }

    bool is_better(Solution const& other_solution) const { std::cout << "started: " << __PRETTY_FUNCTION__ << std::endl; return rat > other_solution.rat; }

    const std::vector<int>& get_prev_pos() const
    {
        std::cout << "started: " << __PRETTY_FUNCTION__ << std::endl;
        if(!prev_buffer_pos.empty())
        {
            return prev_buffer_pos;
        }
        else
        {
            throw std::invalid_argument("There is no previous buffers");
        }
    }

    double get_rat() const
    {
        std::cout << "started: " << __PRETTY_FUNCTION__ << std::endl;
        return rat;
    }

    double get_capacitance() const
    {
        std::cout << "started: " << __PRETTY_FUNCTION__ << std::endl;
        return capacitance;
    }

    void update(const std::vector<int>& buffer_pos, double capacitance, double rat)
    {
        std::cout << "started: " << __PRETTY_FUNCTION__ << std::endl;
        move_prev_pos(buffer_pos, rat);
        capacitance = capacitance;
        buffers.push_back(buffer_pos);
    }

    void move_prev_pos(const std::vector<int>& new_pos, double rat)
    {
        std::cout << "started: " << __PRETTY_FUNCTION__ << std::endl;
        if (new_pos.size() != 2)
        {
            throw std::invalid_argument("Invalid buffers' position");
        }

        rat = rat;
        prev_buffer_pos = new_pos;
    }

// private:
    double capacitance;
    double rat;
    std::vector<std::vector<int>> buffers{};
    std::vector<int> prev_buffer_pos;
};

std::vector<int> _get_nodes_order(gr::Graph const& graph)
{
    std::cout << "started: " << __PRETTY_FUNCTION__ << std::endl;
    int start_id = graph.get_start_node_id();
    std::vector<int> order;
    std::map<int, bool> visited;

    std::stack<int> stack;
    stack.push(graph.get_node(start_id).get_id());

    while (!stack.empty())
    {
        int node_id = stack.top();
        stack.pop();
        if (visited[node_id]) { continue; }

        order.insert(order.begin(), node_id);
        std::vector<int> const& neighbors = graph.get_node_neighbors(node_id);
        for (auto const& neighb_id : neighbors)
        {
            stack.push(graph.nodes.at(neighb_id).get_id());
        }
        visited[node_id] = true;
    }

    return order;
}

Solution _try_insert_buffer(Solution& prev_solution, std::vector<int> const& cur_pos, gr::Edge const& wire, tmg::Model const& timings)
{
    std::cout << "started: " << __PRETTY_FUNCTION__ << std::endl;
    std::vector<int> prev_pos = prev_solution.get_prev_pos();
    int offset_from_prev_point = wire.get_distance(prev_pos, cur_pos);
    int whole_len = wire.get_distance(prev_pos, wire.get_start_pos());
    double C_load = prev_solution.get_capacitance();

    double delay_before_pos = timings.wire_delay(offset_from_prev_point, C_load);
    double buf_delay = timings.buffer_delay(C_load);
    double delay_after_pos = timings.wire_delay(whole_len - offset_from_prev_point, C_load);
    double delay_without_buffer = timings.wire_delay(whole_len, C_load);
    double delay_with_buffer = delay_before_pos + buf_delay + delay_after_pos;

    if (delay_with_buffer < delay_without_buffer)
    {
        double new_rat = prev_solution.get_rat() - (delay_before_pos + buf_delay);
        prev_solution.update(cur_pos, timings.get_buffer_capacitance(), new_rat);
    }

    return prev_solution;
}

void change_cur_pos(std::vector<Solution>& cur_solutions, gr::Edge const& wire, tmg::Model const& timings)
{
    std::cout << "started: " << __PRETTY_FUNCTION__ << std::endl;
    std::vector<int> final_pos = wire.get_start_pos();

    for (auto& solution : cur_solutions)
    {
        int L = wire.get_distance(final_pos, solution.get_prev_pos());
        double wire_delay = timings.wire_delay(L, solution.get_capacitance());
        solution.move_prev_pos(final_pos, solution.get_rat() - wire_delay);
    }
}

std::vector<Solution> _get_solutions_after_segment(gr::Edge const& wire, std::vector<Solution> const& solutions_on_end, tmg::Model const& timings)
{
    std::cout << "started: " << __PRETTY_FUNCTION__ << std::endl;
    int wire_len = wire.get_len();
    std::vector<Solution> cur_solutions = solutions_on_end;

    if (wire_len)
    {
        for (int offset_from_end = 0; offset_from_end < wire_len - 1; ++offset_from_end)
        {
            std::vector<Solution> new_solutions;
            for (Solution& solution : cur_solutions)
            {
                std::vector<int> pos = wire.get_position_by_offset(offset_from_end);
                new_solutions.push_back(_try_insert_buffer(solution, pos, wire, timings));
            }
            cur_solutions = new_solutions;
        }

        change_cur_pos(cur_solutions, wire, timings);
    }

    return cur_solutions;
}

std::vector<Solution> _select_best_solutions(std::vector<Solution> const& all_solutions)
{
    std::cout << "started: " << __PRETTY_FUNCTION__ << std::endl;
    std::set<int> solutions_id_to_remove;
    for (size_t lhs_id = 0; lhs_id < all_solutions.size(); ++lhs_id)
    {
        for (size_t rhs_id = lhs_id; rhs_id < all_solutions.size(); ++rhs_id)
        {
            Solution const& lhs = all_solutions[lhs_id];
            Solution const& rhs = all_solutions[rhs_id];
            if (lhs.is_inferior(rhs)) { solutions_id_to_remove.insert(lhs_id); }
        }
    }
    std::vector<Solution> best_solutions;
    for (size_t id = 0; id < all_solutions.size(); ++id)
    {
        if (solutions_id_to_remove.find(id) == solutions_id_to_remove.end()) { best_solutions.push_back(all_solutions[id]); }
    }
    return best_solutions;
}

std::vector<Solution> _get_solutions(gr::Graph const& graph, int node_id, std::unordered_map<int, std::vector<Solution>>& visited_nodes, tmg::Model const& timings)
{
    std::cout << "started: " << __PRETTY_FUNCTION__ << std::endl;
    if (visited_nodes.contains(node_id)) { return visited_nodes.at(node_id); }

    std::vector<int> neighbors = graph.get_node_neighbors(node_id);
    std::vector<Solution> all_solutions;
    for (auto const& neighb_id : neighbors)
    {
        if (visited_nodes.contains(neighb_id))
        {
            gr::Edge const* edge = graph.get_edge_between_nodes(node_id, neighb_id);
            if (edge)
            {
                std::vector<Solution> solutions_for_wire = _get_solutions_after_segment(*edge, visited_nodes.at(neighb_id), timings);
                for (auto elem: solutions_for_wire) { all_solutions.emplace_back(elem); }
            }
        }
    }
    return _select_best_solutions(all_solutions);
}

Solution _get_best_solution(std::vector<Solution> const& solutions)
{
    std::cout << "started: " << __PRETTY_FUNCTION__ << std::endl;
    if (solutions.empty()) { throw std::invalid_argument("Empty solutions list"); }

    Solution best = solutions[0];

    for (Solution const& solution : solutions)
    {
        if (solution.is_better(best))
        {
            best = solution;
        }
    }
    return best;
}

gr::Graph _create_graph_with_buffers(gr::Graph const& graph, Solution const& root_solution)
{
    std::cout << "started: " << __PRETTY_FUNCTION__ << std::endl;
    gr::Graph new_graph = graph;
    for (auto& buf_pos : root_solution.buffers) { new_graph.add_buffer(buf_pos); }
    return new_graph;
}

std::unordered_map<int, std::vector<Solution>> get_init_solutions(gr::Graph const& graph, std::vector<int> const& order)
{
    std::cout << "started: " << __PRETTY_FUNCTION__ << std::endl;
    std::unordered_map<int, std::vector<Solution>> init_solutions;

    for (auto const& node_id : order)
    {
        auto node = graph.get_node(node_id);
        auto params = node.get_param();
        if (params)
        {
            init_solutions.emplace(node_id, std::vector<Solution>{Solution{node.get_position(), params->capacitance, params->rat}});
        }
    }

    return init_solutions;
}

gr::Graph place_buffers(gr::Graph const& graph, tmg::Model const& timings)
{
    std::cout << "started: " << __PRETTY_FUNCTION__ << std::endl;
    auto traverse_order = _get_nodes_order(graph);
    auto visited_nodes = get_init_solutions(graph, traverse_order);

    for (int node_id : traverse_order)
    {
        std::vector<Solution> solutions_for_node = _get_solutions(graph, node_id, visited_nodes, timings);
        visited_nodes.emplace(node_id, solutions_for_node);
    }

    std::vector<Solution> solutions_for_root = visited_nodes.at(graph.get_start_node_id());
    return _create_graph_with_buffers(graph, _get_best_solution(solutions_for_root));
}
