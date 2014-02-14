#ifndef SOLVER_PARTICLESET_HPP_
#define SOLVER_PARTICLESET_HPP_

#include <unordered_map>
#include <vector>

#include "global.hpp"

namespace solver {
class HistoryEntry;

class ParticleSet {
  public:
    ParticleSet();
    ~ParticleSet() = default;

    std::vector<HistoryEntry *>::const_iterator begin() const;
    std::vector<HistoryEntry *>::const_iterator end() const;
    long size() const;

    void add(HistoryEntry *entry);
    void remove(HistoryEntry *entry);
    HistoryEntry *get(long index) const;
    bool contains(HistoryEntry *entry) const;

  private:
    std::unordered_map<HistoryEntry *, long> particleMap_;
    std::vector<HistoryEntry *> particles_;
};
} /* namespace solver */

#endif /* SOLVER_PARTICLESET_HPP_ */
