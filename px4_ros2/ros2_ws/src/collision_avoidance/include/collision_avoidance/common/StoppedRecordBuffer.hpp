#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace collision_avoidance::common {
// One writer while running; read only after that writer stops. Allocate once
// on the heap before start. No dynamic record contents, waiting or overwrite.
template<class T, std::size_t Capacity>
struct StoppedRecordBuffer {
    static_assert(std::is_trivially_copyable_v<T>, "Records must not own dynamic data");
    std::array<T, Capacity> records{};
    std::size_t size{0};
    std::uint64_t dropped{0};
    void append(const T & record) noexcept {
        if (size == Capacity) { ++dropped; return; }
        records[size++] = record;
    }
};
}
