#pragma once

#include <array>

namespace collision_avoidance::coordinate
{

struct GeodeticReference
{
    double latitude_deg{0.0};
    double longitude_deg{0.0};
    double altitude_m{0.0};
};

class CommonNedTransform
{
public:
    explicit CommonNedTransform(const GeodeticReference & common_reference);

    bool valid() const;
    bool translationFrom(
        const GeodeticReference & local_reference,
        std::array<double, 3> & translation_ned) const;

private:
    GeodeticReference m_common_reference;
    double m_meridional_radius_m{0.0};
    double m_transverse_radius_m{0.0};
    double m_cos_latitude{0.0};
    bool m_valid{false};
};

}  // namespace collision_avoidance::coordinate
