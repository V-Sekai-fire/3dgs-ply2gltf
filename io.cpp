#include "io.h"

#include <fstream>

std::string loadFile(const std::string& filename)
{
    std::ifstream file(filename, std::ios::ate | std::ios::binary);
    if (!file.is_open())
    {
        return {};
    }

    std::size_t fileSize = static_cast<std::size_t>(file.tellg());
    file.seekg(0);

    std::string input(fileSize, 0);

    file.read(input.data(), fileSize);
    file.close();

    return input;
}

bool saveFile(const std::string& output, const std::string& filename)
{
    std::ofstream file(filename, std::ios::binary);
    if (!file.is_open())
    {
        return false;
    }

    file.write(output.data(), output.size());
    file.close();

    return true;
}
