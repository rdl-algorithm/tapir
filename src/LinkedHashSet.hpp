#ifndef SOLVER_LINKEDHASHSET_HPP_
#define SOLVER_LINKEDHASHSET_HPP_

#include <iostream>
#include <initializer_list>
#include <list>
#include <unordered_map>

namespace abt {
template <typename Element>
class LinkedHashSet {
  public:
    LinkedHashSet() :
        map_(),
        elementList_() {
    }

    LinkedHashSet(std::initializer_list<Element> elements) :
        LinkedHashSet() {
        for (Element e : elements) {
            add(e);
        }
    }

    template <typename InputIterator>
    LinkedHashSet(InputIterator first, InputIterator last) :
        LinkedHashSet() {
        for (auto it = first; it != last; it++) {
            add(*it);
        }
    }

    ~LinkedHashSet() = default;

    typename std::list<Element>::const_iterator begin() const {
        return elementList_.cbegin();
    }

    typename std::list<Element>::const_iterator end() const {
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
        map_.emplace(entry, std::prev(elementList_.end()));
    }

    void remove(Element entry) {
        if (!contains(entry)) {
            return;
        }
        elementList_.erase(map_[entry]);
        map_.erase(entry);
    }

    Element getFirst() const {
        return elementList_.front();
    }

    bool contains(Element entry) const {
        return map_.count(entry) > 0;
    }

  private:
    std::unordered_map<Element, typename std::list<Element>::iterator> map_;
    std::list<Element> elementList_;
};
} /* namespace abt */

#endif /* SOLVER_LINKEDHASHSET_HPP_ */
