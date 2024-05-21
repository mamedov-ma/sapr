#include "../include/algo.hpp"

#include <iostream>

int main(int argc, char* argv[])
{
    std::string input_file, tech_file, file_to_dump;

    if (argc > 1)
    {
        input_file = argv[1];
        std::cout << input_file << '\n';
    }

    if (argc > 2)
    {
        tech_file = argv[2];
        std::cout << tech_file << '\n';
    }

    if (argc > 3)
    {
        file_to_dump = argv[3];
        std::cout << file_to_dump << '\n';
    }

    gr::Graph graph;
    graph.load_from_json(input_file);

    tmg::Model timings{};
    timings.load_from_json(tech_file);

    auto final_graph = place_buffers(graph, timings);
    final_graph.store_to_json(input_file + "_out" + ".json");

    if (!file_to_dump.empty())
    {
        graph.dump_to_dot(file_to_dump);
        final_graph.dump_to_dot(file_to_dump);
    }

    return 0;
}
