#ifndef SOLVER_RANDOMACCESSSET_HPP_
#define SOLVER_RANDOMACCESSSET_HPP_

#include <iostream>
#include <initializer_list>
#include <unordered_map>
#include <vector>

namespace abt {
template <typename Element>
class RandomAccessSet {
  public:
    RandomAccessSet() :
        map_(),
        elementList_() {
    }

    RandomAccessSet(std::initializer_list<Element> elements) :
        RandomAccessSet() {
        for (Element e : elements) {
            add(e);
        }
    }

    ~RandomAccessSet() = default;

    typename std::vector<Element>::const_iterator begin() const {
        return elementList_.cbegin();
    }

    typename std::vector<Element>::const_iterator end() const {
        return elementList_.cend();
    }

    void clear() {
        map_.clear();
        elementList_.clear();
    }

    long size() const {
        return elementList_.size();
    }

    void add(Element entry) {
        if (contains(entry)) {
            return;
        }
        elementList_.push_back(entry);
        map_.emplace(entry, elementList_.size() - 1);
    }

    void remove(Element entry) {
        if (!contains(entry)) {
            return;
        }
        long index = map_[entry];
        long lastIndex = elementList_.size() - 1;

        if (index != lastIndex) {
            Element lastEntry = elementList_[lastIndex];
            elementList_[index] = lastEntry;
            map_[lastEntry] = index;
        }

        // Remove extraneous elements.
        elementList_.pop_back();
        map_.erase(entry);
    }
    Element get(long index) const {
        return elementList_[index];
    }

    int indexOf(Element entry) const {
        if (!contains(entry)) {
            return -1;
        }
        return map_[entry];
    }

    bool contains(Element entry) const {
        return map_.count(entry) > 0;
    }

  private:
    std::unordered_map<Element, long> map_;
    std::vector<Element> elementList_;
};
} /* namespace abt */

#endif /* SOLVER_RANDOMACCESSSET_HPP_ */
