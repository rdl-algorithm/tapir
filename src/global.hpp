/** file: global.hpp
 *
 * Contains some key definitions used throughout the ABT code.
 */
#ifndef GLOBAL_HPP_
#define GLOBAL_HPP_

#include <cctype>
#include <ctime>

#include <algorithm>
#include <functional>
#include <locale>
#include <memory>                       // for unique_ptr
#include <random>                       // for default_random_engine
#include <sstream>
#include <utility>                      // for forward

#include "solver/abstract-problem/Point.hpp"

// A macro used to delete all copy- and move- -constructors and -assignment operators
#define _NO_COPY_OR_MOVE(ClassName) \
    ClassName(ClassName const &) = delete; \
    ClassName(ClassName &&) = delete; \
    ClassName &operator=(ClassName const &) = delete; \
    ClassName &operator=(ClassName &&) = delete

// The RNG engine to use.
typedef std::default_random_engine RandomGenerator;

// If we're not using C++1y / C++14, we need to define our own std::make_unique
#if __cplusplus <= 201103L
namespace std {
    template<class T> struct _Unique_if {
        typedef unique_ptr<T> _Single_object;
    };

    template<class T> struct _Unique_if<T[]> {
        typedef unique_ptr<T[]> _Unknown_bound;
    };

    template<class T, size_t N> struct _Unique_if<T[N]> {
        typedef void _Known_bound;
    };

    template<class T, class... Args>
        typename _Unique_if<T>::_Single_object
        make_unique(Args&&... args) {
            return unique_ptr<T>(new T(std::forward<Args>(args)...));
        }

    template<class T>
        typename _Unique_if<T>::_Unknown_bound
        make_unique(size_t n) {
            typedef typename remove_extent<T>::type U;
            return unique_ptr<T>(new U[n]());
        }

    template<class T, class... Args>
        typename _Unique_if<T>::_Known_bound
        make_unique(Args&&...) = delete;
}
#endif

// Returns the time (in ms) since the program started running.
namespace abt {
inline double clock_ms() {
    return std::clock() * 1000.0 / CLOCKS_PER_SEC;
}

// A template method to combine hash values - from boost::hash_combine
template<class T>
inline void hash_combine(std::size_t &seed, T const &v) {
    std::hash<T> hasher;
    seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

// trim from start
static inline std::string &ltrim(std::string &s) {
        s.erase(s.begin(), std::find_if(s.begin(), s.end(), std::not1(std::ptr_fun<int, int>(std::isspace))));
        return s;
}

// trim from end
static inline std::string &rtrim(std::string &s) {
        s.erase(std::find_if(s.rbegin(), s.rend(), std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
        return s;
}

// trim from both ends
static inline std::string &trim(std::string &s) {
        return ltrim(rtrim(s));
}

/** A simple method to print a value with a fixed width. */
template <typename T>
void print_with_width(T value , std::ostream &os, int width,
        std::ios_base::fmtflags flags = std::ios_base::fixed) {
    std::streamsize oldWidth = os.width(width);
    std::ios_base::fmtflags oldFlags = os.flags(flags);
    os << value;
    os.flags(oldFlags);
    os.width(oldWidth);
}

/** A method to print a floating point value, with various options. */
inline void print_double(double value, std::ostream &os, int width = 10,
        int precision = 7,
        std::ios_base::fmtflags flags = std::ios_base::fixed,
        std::ostream::char_type fillCh = ' ') {
    std::streamsize oldPrecision = os.precision(precision);
    std::streamsize oldWidth = os.width(width);
    std::ostream::char_type oldFillCh = os.fill(fillCh);
    std::ios_base::fmtflags oldflags = os.flags(flags);
    os << value;
    os.flags(oldflags);
    os.fill(oldFillCh);
    os.width(oldWidth);
    os.precision(oldPrecision);
}
} /* namespace abt */

namespace debug {
    /** Global debug method to display an error / warning message.
     * This method is a great place for setting breakpoints!
     */
    void show_message(std::string message, bool print = true,
            bool bp_branch = true);

    /** A generic method for converting values to strings. This is useful in GDB, which
     * doesn't deal very nicely with using streams directly.
     */
    template <typename T>
    std::string to_string(T t) {
        std::ostringstream sstr;
        sstr << t;
        return sstr.str();
    }
} /* namespace debug */
#endif /* GLOBAL_HPP_ */
