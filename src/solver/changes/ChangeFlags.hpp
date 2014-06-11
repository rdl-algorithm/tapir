#ifndef SOLVER_CHANGEFLAGS_HPP_
#define SOLVER_CHANGEFLAGS_HPP_

#include <initializer_list>

namespace solver {

enum class ChangeFlags : int {
    UNCHANGED = 0x00,
    REWARD = 0x01,
    TRANSITION = 0x02,
    TRANSITION_BEFORE = 0x04,
    DELETED = 0x08, // NOTE: DELETED necessarily implies TRANSITION_BEFORE
    OBSERVATION = 0x10,
    OBSERVATION_BEFORE = 0x20,
    HEURISTIC = 0x40
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
