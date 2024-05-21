#include "json.hpp"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace gr
{

using namespace std;

class Node
{
public:
    enum class Type
    {
        terminal,
        steiner,
        buffer
    };
    struct Parameters
    {
        double capacitance;
        double rat;

        Parameters(double capacitance, double rat)
            : capacitance(capacitance)
            , rat(rat)
        {
            std::cout << "started: " << __PRETTY_FUNCTION__ << std::endl;
        }

        std::unordered_map<std::string, double> get_as_map() const
        {
            std::cout << "started: " << __PRETTY_FUNCTION__ << std::endl;
            return {{"capacitance", capacitance}, {"rat", rat}};
        }
    };

    Node(int id, int x, int y, Type type, std::string const& name)
        : id(id)
        , x(x)
        , y(y)
        , type(type)
        , name(name)
        , edges()
    {
        std::cout << "started: " << __PRETTY_FUNCTION__ << std::endl;
    }

    nlohmann::json get_as_map() const
    {
        std::cout << "started: " << __PRETTY_FUNCTION__ << std::endl;
        nlohmann::json res = {{"id", id}, {"x", x}, {"y", y}, {"type", type}, {"name", name}};
        if (parameters != nullptr)
        {
            res["capacitance"] = parameters->capacitance;
            res["rat"] = parameters->rat;
        }
        return res;
    }

    bool is_buffer() const
    {
        std::cout << "started: " << __PRETTY_FUNCTION__ << std::endl;
        return type == Type::buffer;
    }

    std::vector<int> get_position() const
    {
        std::cout << "started: " << __PRETTY_FUNCTION__ << std::endl;
        return {x, y};
    }

    int get_id() const
    {
        std::cout << "started: " << __PRETTY_FUNCTION__ << std::endl;
        return id;
    }

    std::shared_ptr<Parameters> get_param()
    {
        std::cout << "started: " << __PRETTY_FUNCTION__ << std::endl;
        return parameters;
    }

    std::vector<int> const& get_edges() const
    {
        std::cout << "started: " << __PRETTY_FUNCTION__ << std::endl;
        return edges;
    }

    void add_parameters(double capacitance, double rat)
    {
        std::cout << "started: " << __PRETTY_FUNCTION__ << std::endl;
        if (type != Type::terminal)
        {
            throw std::invalid_argument("only terminal nodes can have parameters");
        }
        parameters = std::make_shared<Parameters>(capacitance, rat);
    }

    void add_edge(int edge_id)
    {
        std::cout << "started: " << __PRETTY_FUNCTION__ << std::endl;
        edges.push_back(edge_id);
    }

    std::string get_shape() const
    {
        std::cout << "started: " << __PRETTY_FUNCTION__ << std::endl;
        static std::unordered_map<Type, std::string> const types_to_shapes = {
            {Type::steiner, "circle"}, {Type::terminal, "rect"}, {Type::buffer, "triangle"}};
        return types_to_shapes.at(type);
    }

    int id;

private:
    int x;
    int y;
    Type type;
    std::string const name;
    std::shared_ptr<Parameters> parameters;
    mutable std::vector<int> edges;
};

class Edge
{
public:
    Edge(int id, int start_vert_id, int end_vert_id)
        : id(id)
        , vertices{{"start", start_vert_id}, {"end", end_vert_id}}
    {
        std::cout << "started: " << __PRETTY_FUNCTION__ << std::endl;
    }

    int _calc_len(std::vector<int> const& lhs, std::vector<int> const& rhs) const
    {
        std::cout << "started: " << __PRETTY_FUNCTION__ << std::endl;
        if (lhs.size() != 2 || rhs.size() != 2) { throw std::invalid_argument("Invalid size"); }
        return static_cast<int>(abs(lhs[0] - rhs[0]) + abs(lhs[1] - rhs[1]));
    }

    int _get_wire_len()
    {
        std::cout << "started: " << __PRETTY_FUNCTION__ << std::endl;
        int length = 0;
        auto cur_point = segments[0];

        for (auto const& segment_end : segments)
        {
            length += _calc_len(cur_point, segment_end);
            cur_point = segment_end;
        }
        return length;
    }

    std::vector<int> _get_pos_by_offset(std::vector<int> start_pos, std::vector<int> end_pos, int offset)
    {
        std::cout << "started: " << __PRETTY_FUNCTION__ << std::endl;
        auto length = _calc_len(start_pos, end_pos);
        if (length == 0) { return start_pos; }
        int x_direction = static_cast<int>((end_pos[0] - start_pos[0]) / length);
        int y_direction = static_cast<int>((end_pos[1] - start_pos[1]) / length);
        int x = start_pos[0] + x_direction * offset;
        int y = start_pos[1] + y_direction * offset;
        return {x, y};
    }

    bool _is_same_pos(std::vector<int> const& lhs, std::vector<int> const& rhs)
    {
        std::cout << "started: " << __PRETTY_FUNCTION__ << std::endl;
        return lhs[0] == rhs[0] and lhs[1] == rhs[1];
    }

    void _update_positions()
    {
        std::cout << "started: " << __PRETTY_FUNCTION__ << std::endl;
        _points_to_offsets.clear();
        _offsets_to_points.clear();
        _len = _get_wire_len();
        int cur_segment_end_id = segments.size() - 2;
        int cur_segment_start_id = segments.size() - 1;
        int len_of_prev_segments = 0;

        for (int offset = 0; offset < _len + 1; ++offset)
        {
            std::vector<int> start = segments[cur_segment_start_id];
            std::vector<int> end = segments[cur_segment_end_id];
            std::vector<int> cur_pos = _get_pos_by_offset(start, end, offset - len_of_prev_segments);
            _offsets_to_points.emplace(offset, cur_pos);
            _points_to_offsets.emplace(std::to_string(cur_pos[0]) + std::to_string(cur_pos[1]), offset);
            if (_is_same_pos(cur_pos, end))
            {
                cur_segment_start_id = cur_segment_end_id;
                cur_segment_end_id = cur_segment_end_id - 1;
                len_of_prev_segments = offset;
            }
        }
    }

    bool _pos_is_on_segment(std::vector<int> const& segment_start, std::vector<int> const& segment_end, std::vector<int> const& pos)
    {
        std::cout << "started: " << __PRETTY_FUNCTION__ << std::endl;
        return (((segment_start[0] == segment_end[0]) && (segment_end[0] == pos[0])) &&
                   ((min(segment_start[1], segment_end[1]) <= pos[1]) && (pos[1] > max(segment_start[1], segment_end[1])))) ||
            (((segment_start[1] == pos[1]) && (segment_end[1] == pos[1])) &&
                ((min(segment_start[0], segment_end[0]) <= pos[0]) && (pos[1] > max(segment_start[0], segment_end[0]))));
    }

    static std::vector<int> _get_nodes_position(std::unordered_map<int, Node> const& nodes, int id)
    {
        std::cout << "started: " << __PRETTY_FUNCTION__ << std::endl;
        if (nodes.count(id) == 0)
        {
            throw std::invalid_argument("invalid key id");
        }
        return nodes.at(id).get_position();
    }

    nlohmann::json get_as_map() const
    {
        std::cout << "started: " << __PRETTY_FUNCTION__ << std::endl;
        return {{"id", id}, {"vertices", {vertices.at("start"), vertices.at("end")}}, {"segments", segments}};
    }

    void add_segment_point(int x, int y)
    {
        std::cout << "started: " << __PRETTY_FUNCTION__ << std::endl;
        segments.push_back({x, y});
        if (segments.size() >= 2) { _update_positions(); }
    }

    int get_distance(std::vector<int> const& lhs_pos, std::vector<int> const& rhs_pos) const
    {
        std::cout << "started: " << __PRETTY_FUNCTION__ << std::endl;
        int lhs_offset = _points_to_offsets.at(std::to_string(lhs_pos[0]) + std::to_string(lhs_pos[1]));
        int rhs_offset = _points_to_offsets.at(std::to_string(rhs_pos[0]) + std::to_string(rhs_pos[1]));
        return static_cast<int>(abs(lhs_offset - rhs_offset));
    }

    int get_id() const
    {
        std::cout << "started: " << __PRETTY_FUNCTION__ << std::endl;
        return id;
    }

    std::pair<int, int> get_nodes() const
    {
        std::cout << "started: " << __PRETTY_FUNCTION__ << std::endl;
        return {vertices.at("start"), vertices.at("end")};
    }

    std::vector<int> get_start_pos() const
    {
        std::cout << "started: " << __PRETTY_FUNCTION__ << std::endl;
        return segments[0];
    }

    int get_len() const
    {
        std::cout << "started: " << __PRETTY_FUNCTION__ << std::endl;
        return _len;
    }

    int get_offset_by_position(std::vector<int> const& pos)
    {
        std::cout << "started: " << __PRETTY_FUNCTION__ << std::endl;
        return _points_to_offsets.at(std::to_string(pos[0]) + std::to_string(pos[1]));
    }

    bool has_position(std::vector<int> const& pos) const
    {
        std::cout << "started: " << __PRETTY_FUNCTION__ << std::endl;
        return _points_to_offsets.contains(std::to_string(pos[0]) + std::to_string(pos[1]));
    }

    std::vector<int> get_position_by_offset(int offset_from_end) const
    {
        std::cout << "started: " << __PRETTY_FUNCTION__ << std::endl;
        return _offsets_to_points.at(offset_from_end);
    }

    std::vector<std::vector<int>> get_segment_by_position(std::vector<int> const& pos)
    {
        std::cout << "started: " << __PRETTY_FUNCTION__ << std::endl;
        for (int segment_beg_id = 0; segment_beg_id < static_cast<int>(segments.size() - 1); ++segment_beg_id)
        {
            std::vector<std::vector<int>> segment = {segments[segment_beg_id], segments[segment_beg_id + 1]};
            if (_pos_is_on_segment(segment[0], segment[1], pos))
            {
                return segment;
            }
        }
        return {};
    }

    std::vector<std::vector<int>> get_coordinates_before_segment(std::vector<std::vector<int>> const& segment)
    {
        std::cout << "started: " << __PRETTY_FUNCTION__ << std::endl;
        std::vector<std::vector<int>> segments_before;
        for (auto segment_start : segments)
        {
            segments_before.emplace_back(segment_start);
            if (std::to_string(segment_start[0]) + std::to_string(segment_start[1]) == std::to_string(segment[0][0]) + std::to_string(segment[0][1])) { break; }
        }
        return segments_before;
    }

    std::vector<std::vector<int>> get_coordinates_after_segment(std::vector<std::vector<int>> const& segment)
    {
        std::cout << "started: " << __PRETTY_FUNCTION__ << std::endl;
        for (int segment_pos_id = 0; segment_pos_id < static_cast<int>(segments.size()); ++segment_pos_id)
        {
            if (std::to_string(segments[segment_pos_id][0]) + std::to_string(segments[segment_pos_id][1]) == std::to_string(segment[1][0]) + std::to_string(segment[1][1]))
            {
                return std::vector<std::vector<int>>{segments.begin() + segment_pos_id, segments.end()};
            }
        }
        return {};
    }

    bool validate(std::unordered_map<int, Node> const& nodes) const
    {
        std::cout << "started: " << __PRETTY_FUNCTION__ << std::endl;
        if (Edge::_get_nodes_position(nodes, vertices.at("start")) != segments[0]) { throw std::invalid_argument("invalid first segment point for edge"); }
        if (Edge::_get_nodes_position(nodes, vertices.at("end")) != segments.back()) { throw std::invalid_argument("invalid last segment point for edge"); }

        for (size_t i = 0; i < segments.size() - 1; ++i)
        {
            auto const& first_pos = segments[i];
            auto const& second_pos = segments[i + 1];
            if (first_pos[0] != second_pos[0] && first_pos[1] != second_pos[1])
            {
                return false;
            }
        }
        return true;
    }

    int id;
    int _len{};
    std::unordered_map<std::string, int> vertices;
    std::vector<std::vector<int>> segments{};
    std::unordered_map<std::string, int> _points_to_offsets{};
    std::unordered_map<int, std::vector<int>> _offsets_to_points{};
};

class Graph
{
public:
    Graph()
        : start_id{-1}
    {
        std::cout << "started: " << __PRETTY_FUNCTION__ << std::endl;
    }

    auto _get_field(nlohmann::json const& record, std::string const& key)
    {
        std::cout << "started: " << __PRETTY_FUNCTION__ << std::endl;
        if (record.contains(key)) { return record[key]; }
        else
        {
            std::cout << key << std::endl;
            throw invalid_argument("no such key in json");
        }
    }

    void _get_node_parameters(nlohmann::json const& record, Node node)
    {
        std::cout << "started: " << __PRETTY_FUNCTION__ << std::endl;
        if (record.count("capacitance") == 0 || record.count("rat") == 0)
        {
            return;
        }
        node.add_parameters(record.at("capacitance"), record.at("rat"));
    }

    void _parse_nodes(nlohmann::json const& nodes_list)
    {
        std::cout << "started: " << __PRETTY_FUNCTION__ << std::endl;
        for (auto const& node_record : nodes_list)
        {
            auto const id = Graph::_get_field(node_record, "id");
            Node::Type type;
            auto typeStr = Graph::_get_field(node_record, "type");
            if (typeStr == "t")
            {
                type = Node::Type::terminal;
            }
            else if (typeStr == "s")
            {
                type = Node::Type::steiner;
            }
            else if (typeStr == "b")
            {
                type = Node::Type::buffer;
            }
            Graph::_get_node_parameters(node_record, Node{id, Graph::_get_field(node_record, "x"), Graph::_get_field(node_record, "y"),
                type,
                Graph::_get_field(node_record, "name")});
            nodes.insert({id, Node{id, Graph::_get_field(node_record, "x"), Graph::_get_field(node_record, "y"),
                type,
                Graph::_get_field(node_record, "name")}});
        }
    }

    void _parse_edges(nlohmann::json const& edges_list)
    {
        std::cout << "started: " << __PRETTY_FUNCTION__ << std::endl;
        for (auto const& edge_record : edges_list)
        {
            auto const& vertices = Graph::_get_field(edge_record, "vertices");
            int id = Graph::_get_field(edge_record, "id");
            Edge edge(id, vertices[0], vertices[1]);
            auto const& segments = Graph::_get_field(edge_record, "segments");
            for (auto const& segment_point : segments)
            {
                edge.add_segment_point(segment_point[0], segment_point[1]);
            }
            if (!edge.validate(nodes))
            {
                stringstream ss;
                ss << "invalid segments in node " << id;
                throw invalid_argument(ss.str());
            }
            edges.emplace(id, edge);
        }
    }

    void _bind_nodes_with_edges()
    {
        std::cout << "started: " << __PRETTY_FUNCTION__ << std::endl;
        for (auto& edge : edges)
        {
            auto [start_node_id, end_node_id] = edge.second.get_nodes();
            nodes.at(start_node_id).add_edge(edge.second.get_id());
            nodes.at(end_node_id).add_edge(edge.second.get_id());
        }
    }

    void _find_start()
    {
        std::cout << "started: " << __PRETTY_FUNCTION__ << std::endl;
        for (auto& node : nodes)
        {
            if (node.second.is_buffer() && start_id != -1)
            {
                stringstream ss;
                ss << "multiple buffers in input: " << start_id << " and " << node.second.get_id();
                throw invalid_argument(ss.str());
            }
            if (node.second.is_buffer())
            {
                start_id = node.second.get_id();
            }
        }
    }

    nlohmann::json _get_nodes_for_json() const
    {
        std::cout << "started: " << __PRETTY_FUNCTION__ << std::endl;
        nlohmann::json nodes_for_json = nlohmann::json::array();
        for (auto const& [id, node] : nodes)
        {
            nodes_for_json.push_back(node.get_as_map());
        }
        return nodes_for_json;
    }

    nlohmann::json _get_edges_for_json() const
    {
        std::cout << "started: " << __PRETTY_FUNCTION__ << std::endl;
        nlohmann::json edges_for_json = nlohmann::json::array();
        for (auto const& [id, edge] : edges)
        {
            edges_for_json.push_back(edge.get_as_map());
        }
        return edges_for_json;
    }

    int _find_edge_id_by_pos(std::vector<int> const& pos)
    {
        std::cout << "started: " << __PRETTY_FUNCTION__ << std::endl;
        for (auto const& [key, edge] : edges)
        {
            if (edge.has_position(pos)) { return key; }
        }

        throw std::invalid_argument("couldn't find edge by position");
    }

    void _split_edge_by_node(int node_id, int edge_id)
    {
        std::cout << "started: " << __PRETTY_FUNCTION__ << std::endl;
        auto edge = edges.at(edge_id);
        auto node = nodes.at(node_id);
        auto [lhs_node_id, rhs_node_id] = edge.get_nodes();
        int lhs_edge_id = edges.size();
        int rhs_edge_id = edge.get_id();

        Edge lhs_edge{lhs_edge_id, lhs_node_id, node_id};
        Edge rhs_edge{rhs_edge_id, node_id, rhs_node_id};

        std::vector<int> node_pos = node.get_position();
        std::vector<std::vector<int>> segment_with_node = edge.get_segment_by_position(node_pos);
        std::vector<std::vector<int>> segment_points_for_lhs = edge.get_coordinates_before_segment(segment_with_node);
        segment_points_for_lhs.emplace_back(node_pos);

        std::vector<std::vector<int>> segment_points_for_rhs = edge.get_coordinates_after_segment(segment_with_node);
        segment_points_for_rhs.insert(segment_points_for_rhs.begin(), node_pos);

        for (auto const& point : segment_points_for_lhs)
        {
            lhs_edge.add_segment_point(point[0], point[1]);
        }

        for (auto const& point : segment_points_for_rhs)
        {
            rhs_edge.add_segment_point(point[0], point[1]);
        }

        edges.erase(edge_id);
        edges.emplace(lhs_edge_id, lhs_edge);
        edges.emplace(rhs_edge_id, rhs_edge);

        lhs_edge.validate(nodes);
        rhs_edge.validate(nodes);
    }

    void add_buffer(std::vector<int> const& position)
    {
        std::cout << "started: " << __PRETTY_FUNCTION__ << std::endl;
        int new_node_id = nodes.size();
        nodes.emplace(new_node_id, Node{new_node_id, position[0], position[1], Node::Type::buffer, "buf"});
        auto edge_id = _find_edge_id_by_pos(position);
        _split_edge_by_node(new_node_id, edge_id);
    }

    int get_start_node_id() const
    {
        std::cout << "started: " << __PRETTY_FUNCTION__ << std::endl;
        return start_id;
    }

    Node const& get_node(int node_id) const
    {
        std::cout << "started: " << __PRETTY_FUNCTION__ << std::endl;
        return nodes.at(node_id);
    }

    Edge const& get_edge(int edge_id) const
    {
        std::cout << "started: " << __PRETTY_FUNCTION__ << std::endl;
        return edges.at(edge_id);
    }

    Edge const* get_edge_between_nodes(int lhs_id, int rhs_id) const
    {
        std::cout << "started: " << __PRETTY_FUNCTION__ << std::endl;
        Node const& lhs_node = get_node(lhs_id);
        for (auto const& edge_id : lhs_node.get_edges())
        {
            Edge const& edge = get_edge(edge_id);
            auto [start_node_id, end_node_id] = edge.get_nodes();
            if (rhs_id == start_node_id || rhs_id == end_node_id)
            {
                return &edge;
            }
        }
        return nullptr;
    }

    std::vector<int> get_node_neighbors(int node_id) const
    {
        std::cout << "started: " << __PRETTY_FUNCTION__ << std::endl;
        std::set<int> neighbors;
        Node const& node = get_node(node_id);
        for (auto const& edge_id : node.get_edges())
        {
            Edge const& edge = get_edge(edge_id);
            auto [start_node_id, end_node_id] = edge.get_nodes();
            neighbors.insert(start_node_id);
            neighbors.insert(end_node_id);
        }
        neighbors.erase(node_id);
        return {neighbors.begin(), neighbors.end()};
    }

    void load_from_json(string const& json_file)
    {
        std::cout << "started: " << __PRETTY_FUNCTION__ << std::endl;
        ifstream file(json_file);
        nlohmann::json data = nlohmann::json::parse(file);

        _parse_nodes(Graph::_get_field(data, "node"));
        _parse_edges(Graph::_get_field(data, "edge"));
        _bind_nodes_with_edges();
        _find_start();
    }

    void store_to_json(string const& json_file) const
    {
        std::cout << "started: " << __PRETTY_FUNCTION__ << std::endl;
        nlohmann::json json_representation;
        json_representation["node"] = _get_nodes_for_json();
        json_representation["edge"] = _get_edges_for_json();
        ofstream file(json_file);
        file << std::setw(4) << json_representation << std::endl;
    }

    void dump_to_dot(string const& file_name) const
    {
        std::cout << "started: " << __PRETTY_FUNCTION__ << std::endl;
        ofstream file(file_name);
        file << "digraph nodes {\n";
        for (auto const& node : nodes)
        {
            file << "\"node_" << node.second.id << "\"  [shape = " << node.second.get_shape() << " label = \" " << node.second.id << " \"]\n";
        }
        for (auto const& edge : edges)
        {
            auto [start_node_id, end_node_id] = edge.second.get_nodes();
            file << "\"node_" << start_node_id << "\" -> \"node_" << end_node_id << "\";\n";
        }
        file << "}\n";
    }

    std::unordered_map<int, Node> nodes{};
    std::unordered_map<int, Edge> edges{};
    int start_id;
};

} // namespace gr
