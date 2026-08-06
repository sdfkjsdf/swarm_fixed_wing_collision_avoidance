#include <collision_avoidance/coordinate/CommonNedTransform.hpp>

#include <cmath>

namespace collision_avoidance::coordinate
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegreesToRadians = kPi / 180.0;
constexpr double kWgs84SemiMajorAxisM = 6378137.0;
constexpr double kWgs84EccentricitySquared = 6.69437999014e-3;

bool validReference(const GeodeticReference & reference)
{
    return std::isfinite(reference.latitude_deg)
        && std::isfinite(reference.longitude_deg)
        && std::isfinite(reference.altitude_m)
        && std::abs(reference.latitude_deg) <= 90.0
        && std::abs(reference.longitude_deg) <= 180.0;
}

}  // namespace

CommonNedTransform::CommonNedTransform(const GeodeticReference & common_reference)
: m_common_reference(common_reference)
{
    if (!validReference(common_reference)) {
        return;
    }

    const double latitude_rad = common_reference.latitude_deg * kDegreesToRadians;
    const double sin_latitude = std::sin(latitude_rad);
    const double denominator = std::sqrt(
        1.0 - kWgs84EccentricitySquared * sin_latitude * sin_latitude);
    m_transverse_radius_m = kWgs84SemiMajorAxisM / denominator;
    m_meridional_radius_m = kWgs84SemiMajorAxisM
        * (1.0 - kWgs84EccentricitySquared)
        / (denominator * denominator * denominator);
    m_cos_latitude = std::cos(latitude_rad);
    m_valid = std::isfinite(m_meridional_radius_m)
        && std::isfinite(m_transverse_radius_m)
        && std::isfinite(m_cos_latitude);
}

bool CommonNedTransform::valid() const
{
    return m_valid;
}

bool CommonNedTransform::translationFrom(
    const GeodeticReference & local_reference,
    std::array<double, 3> & translation_ned) const
{
    if (!m_valid || !validReference(local_reference)) {
        return false;
    }

    const double latitude_delta_rad =
        (local_reference.latitude_deg - m_common_reference.latitude_deg)
        * kDegreesToRadians;
    const double longitude_delta_rad =
        (local_reference.longitude_deg - m_common_reference.longitude_deg)
        * kDegreesToRadians;
    translation_ned = {
        latitude_delta_rad * m_meridional_radius_m,
        longitude_delta_rad * m_transverse_radius_m * m_cos_latitude,
        m_common_reference.altitude_m - local_reference.altitude_m,
    };
    return std::isfinite(translation_ned[0])
        && std::isfinite(translation_ned[1])
        && std::isfinite(translation_ned[2]);
}

}  // namespace collision_avoidance::coordinate
