
#include "json.hpp"

#include <fstream>
#include <iostream>

namespace tmg
{

using json = nlohmann::json;

class Model
{
public:
    Model()
        : D_int(0.0)
        , R_buf(0.0)
        , C_buf(0.0)
        , unit_R(0.0)
        , unit_C(0.0)
    {}

    double get_buffer_capacitance() const { return C_buf; }

    double buffer_delay(double C) const { return D_int + R_buf * C; }

    double wire_delay(double L, double C) const
    {
        return (unit_R * unit_C * L * L) / 2 + unit_R * L * C;
    }

    void load_from_json(std::string const& json_file)
    {
        std::ifstream file(json_file);
        json data = json::parse(file);

        auto const model = data["module"];
        auto const input = model[0]["input"];
        D_int = input[0]["intrinsic_delay"];
        R_buf = input[0]["R"];
        R_buf = input[0]["C"];

        auto const technology = data["technology"];
        unit_R = technology["unit_wire_resistance"];
        unit_C = technology["unit_wire_capacitance"];
    }

private:
    double D_int;
    double R_buf;
    double C_buf;
    double unit_R;
    double unit_C;
};

} // namespace tmg
