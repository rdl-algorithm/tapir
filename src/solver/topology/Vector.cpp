#include "Vector.hpp"

#include <cmath>
#include <cstddef>

#include <vector>

#include "global.hpp"

namespace solver {
bool Vector::equals(Point const &otherPoint) const {
    std::vector<double> v1 = this->asVector();
    std::vector<double> v2 = static_cast<Vector const *>(
            &otherPoint)->asVector();
    std::vector<double>::iterator i1 = v1.begin();
    std::vector<double>::iterator i2 = v2.begin();

    for (; i1 < v1.end(); i1++, i2++) {
        if (*i1 != *i2) {
            return false;
        }
    }
    return true;
}

std::size_t Vector::hash() const {
    std::vector<double> v1 = this->asVector();

    std::size_t hashValue = 0;
    for (std::vector<double>::iterator i1 = v1.begin(); i1 < v1.end(); i1++) {
        abt::hash_combine(hashValue, *i1);
    }
    return hashValue;

}

void Vector::print(std::ostream &os) const {
    std::vector<double> values = asVector();
    os << "(";
    for (std::vector<double>::const_iterator it = values.begin();
            it < values.end(); it++) {
        os << *it;
        if (it + 1 != values.end()) {
            os << ",";
        }
    }
    os << ")";
}

} /* namespace solver */
