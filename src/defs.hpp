#ifndef DEFS_HPP_
#define DEFS_HPP_

#include <memory>                       // for unique_ptr
#include <random>                       // for default_random_engine
#include <utility>                      // for forward

typedef std::default_random_engine RandomGenerator;

namespace std {
template<typename T, typename ... Args>
std::unique_ptr<T> make_unique(Args && ... args) {
    return std::unique_ptr<T>(new T(std::forward<Args>(args) ...));
}
}

#endif /* DEFS_HPP_ */
