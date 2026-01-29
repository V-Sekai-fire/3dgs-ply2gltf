#ifndef GLTF_IO_H
#define GLTF_IO_H

#include <string>

std::string loadFile(const std::string& filename);

bool saveFile(const std::string& output, const std::string& filename);

#endif /*GLTF_IO_H*/
