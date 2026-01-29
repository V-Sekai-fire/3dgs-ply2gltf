#include "dump.h"

#include <cmath>
#include <vector>

void appendLittleEndian(std::string& dump, float value)
{
    const std::uint8_t* data = reinterpret_cast<const std::uint8_t*>(&value);
    dump += data[0u];
    dump += data[1u];
    dump += data[2u];
    dump += data[3u];
}

std::string dumpPly(const std::string& binary, std::uint32_t count, std::uint32_t byteStride, std::uint32_t degree)
{
    std::string dump{};

    dump += "ply\n";
    dump += "format binary_little_endian 1.0\n";
    dump += "element vertex " + std::to_string(count) + "\n";
    dump += "property float x\n";
    dump += "property float y\n";
    dump += "property float z\n";
    dump += "property float rot_0\n";
    dump += "property float rot_1\n";
    dump += "property float rot_2\n";
    dump += "property float rot_3\n";
    dump += "property float scale_0\n";
    dump += "property float scale_1\n";
    dump += "property float scale_2\n";
    dump += "property float opacity\n";

    dump += "property float f_dc_0\n";
    dump += "property float f_dc_1\n";
    dump += "property float f_dc_2\n";
    if (degree > 0u)
    {
        dump += "property float f_rest_0\n";
        dump += "property float f_rest_1\n";
        dump += "property float f_rest_2\n";

        dump += "property float f_rest_3\n";
        dump += "property float f_rest_4\n";
        dump += "property float f_rest_5\n";

        dump += "property float f_rest_6\n";
        dump += "property float f_rest_7\n";
        dump += "property float f_rest_8\n";

        if (degree > 1u)
        {
            dump += "property float f_rest_9\n";
            dump += "property float f_rest_10\n";
            dump += "property float f_rest_11\n";

            dump += "property float f_rest_12\n";
            dump += "property float f_rest_13\n";
            dump += "property float f_rest_14\n";

            dump += "property float f_rest_15\n";
            dump += "property float f_rest_16\n";
            dump += "property float f_rest_17\n";

            dump += "property float f_rest_18\n";
            dump += "property float f_rest_19\n";
            dump += "property float f_rest_20\n";

            dump += "property float f_rest_21\n";
            dump += "property float f_rest_22\n";
            dump += "property float f_rest_23\n";

            if (degree > 2u)
            {

                dump += "property float f_rest_24\n";
                dump += "property float f_rest_25\n";
                dump += "property float f_rest_26\n";

                dump += "property float f_rest_27\n";
                dump += "property float f_rest_28\n";
                dump += "property float f_rest_29\n";

                dump += "property float f_rest_30\n";
                dump += "property float f_rest_31\n";
                dump += "property float f_rest_32\n";

                dump += "property float f_rest_33\n";
                dump += "property float f_rest_34\n";
                dump += "property float f_rest_35\n";

                dump += "property float f_rest_36\n";
                dump += "property float f_rest_37\n";
                dump += "property float f_rest_38\n";

                dump += "property float f_rest_39\n";
                dump += "property float f_rest_40\n";
                dump += "property float f_rest_41\n";

                dump += "property float f_rest_42\n";
                dump += "property float f_rest_43\n";
                dump += "property float f_rest_44\n";
            }
        }
    }
    dump += "end_header\n";

    // Write binary
    for (std::uint32_t vertex = 0u; vertex < count; vertex++)
    {
        std::uint32_t byteOffset{0u};
        const float* data{nullptr};

        {
            // POSITION
            data = reinterpret_cast<const float*>(binary.data() + byteStride * vertex + byteOffset);

            // x
            appendLittleEndian(dump, data[0u]);

            // y
            appendLittleEndian(dump, data[1u]);

            // z
            appendLittleEndian(dump, data[2u]);

            byteOffset += 3u * sizeof(float);
        }

        {
            // ROTATION
            data = reinterpret_cast<const float*>(binary.data() + byteStride * vertex + byteOffset);

            // Swizzle back

            // w
            appendLittleEndian(dump, data[3u]);

            // x
            appendLittleEndian(dump, data[0u]);

            // y
            appendLittleEndian(dump, data[1u]);

            // z
            appendLittleEndian(dump, data[2u]);

            byteOffset += 4u * sizeof(float);
        }

        {
            // SCALE
            const float* data = reinterpret_cast<const float*>(binary.data() + byteStride * vertex + byteOffset);

            // x
            appendLittleEndian(dump, data[0u]);

            // y
            appendLittleEndian(dump, data[1u]);

            // z
            appendLittleEndian(dump, data[2u]);

            byteOffset += 3u * sizeof(float);
        }

        {
            // OPACITY
            const float* data = reinterpret_cast<const float*>(binary.data() + byteStride * vertex + byteOffset);

            // Inverse sigmoid.
            float opacity = std::log(data[0u] / (1.0f - data[0u]));

            appendLittleEndian(dump, opacity);

            byteOffset += 1u * sizeof(float);
        }

        {
            // SH
            const float* data = reinterpret_cast<const float*>(binary.data() + byteStride * vertex + byteOffset);

            // r
            appendLittleEndian(dump, data[0u]);

            // g
            appendLittleEndian(dump, data[1u]);

            // b
            appendLittleEndian(dump, data[2u]);

            byteOffset += 3u * sizeof(float);
        }

        {
            // Rest

            std::vector<float> r{};
            std::vector<float> g{};
            std::vector<float> b{};

            for (std::uint32_t current_degree = 1u; current_degree <= degree; current_degree++)
            {
                if (current_degree == 1u)
                {
                    for (std::uint32_t i = 0u; i < 3u; i++)
                    {
                        const float* data = reinterpret_cast<const float*>(binary.data() + byteStride * vertex + byteOffset);

                        // r
                        r.push_back(data[0u]);

                        // g
                        g.push_back(data[1u]);

                        // b
                        b.push_back(data[2u]);

                        byteOffset += 3u * sizeof(float);
                    }
                }
                if (current_degree == 2u)
                {
                    for (std::uint32_t i = 0u; i < 5u; i++)
                    {
                        const float* data = reinterpret_cast<const float*>(binary.data() + byteStride * vertex + byteOffset);

                        // r
                        r.push_back(data[0u]);

                        // g
                        g.push_back(data[1u]);

                        // b
                        b.push_back(data[2u]);

                        byteOffset += 3u * sizeof(float);
                    }
                }
                if (current_degree == 3u)
                {
                    for (std::uint32_t i = 0u; i < 7u; i ++)
                    {
                        const float* data = reinterpret_cast<const float*>(binary.data() + byteStride * vertex + byteOffset);

                        // r
                        r.push_back(data[0u]);

                        // g
                        g.push_back(data[1u]);

                        // b
                        b.push_back(data[2u]);

                        byteOffset += 3u * sizeof(float);
                    }
                }
            }

            for (auto v : r)
            {
                appendLittleEndian(dump, v);
            }
            for (auto v : g)
            {
                appendLittleEndian(dump, v);
            }
            for (auto v : b)
            {
                appendLittleEndian(dump, v);
            }
        }        
    }

    return dump;
}
