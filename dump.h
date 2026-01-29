#ifndef GLTF_DUMP_H
#define GLTF_DUMP_H

#include <cstdint>
#include <string>

std::string dumpPly(const std::string& binary, std::uint32_t count, std::uint32_t byteStride, std::uint32_t degree);

#endif /*GLTF_DUMP_H*/
