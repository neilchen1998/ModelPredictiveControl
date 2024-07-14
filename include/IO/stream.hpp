#ifndef IO_STREAM_H_
#define IO_STREAM_H_

#include <fstream>
#include <fmt/os.h>
#include <filesystem>
#include <string>
#include <Eigen/Dense>

#include "models/mass_spring_damper.hpp"

namespace IO
{
    void SaveData(const models::MassSpringDamper& model, const std::string& filename)
    {
        auto out = fmt::output_file(filename);
        out.print("hiiiiiiiiiiiiiii {}", "Panic!!!!!!!!!!!!!!!");
    }

    void LoadData(const std::string& filename);
}   // IO

#endif  // IO_STREAM_H_