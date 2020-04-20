#ifndef OSM2PGSQL_WKB_HPP
#define OSM2PGSQL_WKB_HPP

#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>

// #include <osmium/geom/coordinates.hpp>
// #include <osmium/geom/factory.hpp>

namespace ewkb {

enum geometry_type : uint32_t
{
    wkb_point = 1,
    wkb_line = 2,
    wkb_polygon = 3,
    wkb_multi_point = 4,
    wkb_multi_line = 5,
    wkb_multi_polygon = 6,
    wkb_collection = 7,

    wkb_srid = 0x20000000 // SRID-presence flag (EWKB)
};

enum wkb_byte_order_type_t : uint8_t
{
#if __BYTE_ORDER == __LITTLE_ENDIAN
    Endian = 1 // Little Endian
#else
    Endian = 0, // Big Endian
#endif
};

/**
 * Class that allows to iterate over the elements of a ewkb geometry.
 *
 * Note: this class assumes that the wkb was created by ewkb::writer_t.
 *       It implements the exact opposite decoding.
 */
class parser_t
{
public:
    inline static std::string wkb_from_hex(std::string const &wkb)
    {
        std::string out;

        bool front = true;
        char outc;
        for (char c : wkb) {
            c -= 48;
            if (c > 9) {
                c -= 7;
            }
            if (front) {
                outc = char(c << 4);
                front = false;
            } else {
                out += outc | c;
                front = true;
            }
        }

		/*
        if (out[0] != Endian)
            throw std::runtime_error(
#if __BYTE_ORDER == __LITTLE_ENDIAN
                "Geometries in the database are returned in big-endian byte order. "
#else
                "Geometries in the database are returned in little-endian byte order. "
#endif
                "osm2pgsql can only process geometries in native byte order."
                );

		*/
        return out;
    }

    explicit parser_t(char const *wkb) : m_wkb(wkb), m_pos(0) {}
    explicit parser_t(std::string const &wkb) : m_wkb(wkb.c_str()), m_pos(0) {}

    size_t save_pos() const { return m_pos; }
    void rewind(size_t pos) { m_pos = pos; }

    int read_header()
    {
        m_pos += sizeof(uint8_t); // skip endianess

        auto type = read_data<uint32_t>();

        if (type & wkb_srid) {
            m_pos += sizeof(int); // skip srid
        }

        return type & 0xff;
    }

    uint32_t read_length() { return read_data<uint32_t>(); }

	struct Coordinates {
            double x;
            double y;
	};

    Coordinates read_point()
    {
        auto x = read_data<double>();
        auto y = read_data<double>();

		Coordinates c = {x, y};

        return c;
    }

    void skip_points(size_t num) { m_pos += sizeof(double) * 2 * num; }

private:
    template <typename T>
    T read_data()
    {
        T data;
        memcpy(&data, m_wkb + m_pos, sizeof(T));
        m_pos += sizeof(T);

        return data;
    }

    char const *m_wkb;
    size_t m_pos;
};

} // namespace

#endif // OSM2PGSQL_WKB_HPP
