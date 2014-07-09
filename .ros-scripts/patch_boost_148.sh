#!/bin/bash
echo "Patching Boost Header..."
perl -pi -e 's/defined\(_GLIBCXX__PTHREADS\)$/defined(_GLIBCXX__PTHREADS) \\\n        || defined(_GLIBCXX_HAS_GTHREADS)/' tapir_boost_install/boost_1_48_0/boost/config/stdlib/libstdcpp3.hpp
