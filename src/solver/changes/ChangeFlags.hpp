#ifndef SOLVER_CHANGEFLAGS_HPP_
#define SOLVER_CHANGEFLAGS_HPP_

#include <initializer_list>

namespace solver {

enum class ChangeFlags : int {
    UNCHANGED = 0x00,
    DELETED = 0x01,
    REWARD = 0x02,
    TRANSITION = 0x04,
    OBSERVATION = 0x08,
    OBSERVATION_BEFORE = 0x10,
    HEURISTIC = 0x20
};

inline ChangeFlags &operator|=(ChangeFlags &lhs, const ChangeFlags &rhs) {
    lhs = static_cast<ChangeFlags>(static_cast<int>(lhs) | static_cast<int>(rhs));
    return lhs;
}

inline ChangeFlags operator|(ChangeFlags lhs, const ChangeFlags &rhs) {
    lhs |= rhs;
    return lhs;
}

inline ChangeFlags &operator&=(ChangeFlags &lhs, const ChangeFlags &rhs) {
    lhs = static_cast<ChangeFlags>(static_cast<int>(lhs) & static_cast<int>(rhs));
    return lhs;
}

inline ChangeFlags operator&(ChangeFlags lhs, const ChangeFlags &rhs) {
    lhs &= rhs;
    return lhs;
}

inline ChangeFlags operator~(ChangeFlags const &cf) {
    return static_cast<ChangeFlags>(~static_cast<int>(cf));
}

namespace changes {
inline ChangeFlags combine_flags(std::initializer_list<ChangeFlags> flags) {
    ChangeFlags combined = ChangeFlags::UNCHANGED;
    for (ChangeFlags flag : flags) {
        combined |= flag;
    }
    return combined;
}

inline bool has_flag(ChangeFlags value, ChangeFlags flag) {
    return (value & flag) == flag;
}
} /* namespace changes */

} /* namespace solver */

#endif /* SOLVER_CHANGEFLAGS_HPP_ */
