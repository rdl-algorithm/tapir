#ifndef SOLVER_CHANGEFLAGS_HPP_
#define SOLVER_CHANGEFLAGS_HPP_

#include <cstdint>

#include <initializer_list>

namespace solver {
typedef uint8_t ChangeFlagsIntType;

enum class ChangeFlags : ChangeFlagsIntType {
    // No changes
    UNCHANGED = 0x000,

    /* --------------- Flags to apply within the state pool. --------------- */
    // Change in the reward value.
    REWARD = 0x001,

    // Different transition.
    TRANSITION = 0x002,

    // Different transition from the previous state;
    // TRANSITION_BEFORE(e) implies TRANSITION(prev(e))
    TRANSITION_BEFORE = 0x004,

    // This state is deleted; this also means the prior transition must change.
    // DELETED(e) implies TRANSITION_BEFORE(e)
    DELETED = 0x008,

    // Recalculate observation for this current time step.
    OBSERVATION = 0x010,

    // Recalculate observation for the previous time step (o depends on s').
    // OBSERVATION_BEFORE(e) implies OBSERVATION(prev(e))
    OBSERVATION_BEFORE = 0x020,

    // Recalculate the heuristic value iff this state is at the end of a sequence.
    HEURISTIC = 0x040,
};

inline ChangeFlags &operator|=(ChangeFlags &lhs, const ChangeFlags &rhs) {
    lhs = static_cast<ChangeFlags>(static_cast<ChangeFlagsIntType>(lhs) | static_cast<ChangeFlagsIntType>(rhs));
    return lhs;
}

inline ChangeFlags operator|(ChangeFlags lhs, const ChangeFlags &rhs) {
    lhs |= rhs;
    return lhs;
}

inline ChangeFlags &operator&=(ChangeFlags &lhs, const ChangeFlags &rhs) {
    lhs = static_cast<ChangeFlags>(static_cast<ChangeFlagsIntType>(lhs) & static_cast<ChangeFlagsIntType>(rhs));
    return lhs;
}

inline ChangeFlags operator&(ChangeFlags lhs, const ChangeFlags &rhs) {
    lhs &= rhs;
    return lhs;
}

inline ChangeFlags operator~(ChangeFlags const &cf) {
    return static_cast<ChangeFlags>(~static_cast<ChangeFlagsIntType>(cf));
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
