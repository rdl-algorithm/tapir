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
        elements_() {
    }

    RandomAccessSet(std::initializer_list<Element> elements) :
        RandomAccessSet() {
        for (Element e : elements) {
            add(e);
        }
    }

    ~RandomAccessSet() = default;

    typename std::vector<Element>::const_iterator begin() const {
        return elements_.cbegin();
    }

    typename std::vector<Element>::const_iterator end() const {
        return elements_.cend();
    }

    long size() const {
        return elements_.size();
    }

    void add(Element entry) {
        if (contains(entry)) {
            debug::show_message("ERROR: should not be adding duplicate entry!");
            return;
        }
        elements_.push_back(entry);
        map_.emplace(entry, elements_.size() - 1);
    }

    void remove(Element entry) {
        long index = map_[entry];
        long lastIndex = elements_.size() - 1;

        if (index != lastIndex) {
            Element lastEntry = elements_[lastIndex];
            elements_[index] = lastEntry;
            map_[lastEntry] = index;
        }

        // Remove extraneous elements.
        elements_.pop_back();
        map_.erase(entry);
    }
    Element get(long index) const {
        return elements_[index];
    }

    bool contains(Element entry) const {
        return map_.count(entry) > 0;
    }

  private:
    std::unordered_map<Element, long> map_;
    std::vector<Element> elements_;
};
} /* namespace abt */

#endif /* SOLVER_RANDOMACCESSSET_HPP_ */
