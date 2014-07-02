/** @file defs.hpp
 *
 * Some key definitions and macros for C++ programs.
 */
#ifndef DEFS_HPP_
#define DEFS_HPP_

/** A macro used to delete all copy- and move-constructors and -assignment operators */
#define _NO_COPY_OR_MOVE(ClassName) \
    ClassName(ClassName const &) = delete; \
    ClassName(ClassName &&) = delete; \
    ClassName &operator=(ClassName const &) = delete; \
    ClassName &operator=(ClassName &&) = delete

// If we're not using C++1y / C++14, we need to define our own std::make_unique
#if __cplusplus <= 201103L
#ifndef __STD__MAKE__UNIQUE___
#define __STD__MAKE__UNIQUE___
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
#endif

#endif /* DEFS_HPP_ */
