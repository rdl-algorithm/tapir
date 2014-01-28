#include "ParticleSet.hpp"

#include <iterator>
#include <unordered_map>
#include <vector>

namespace solver {

ParticleSet::ParticleSet() :
    particleMap_(),
    particles_() {
}

std::vector<HistoryEntry *>::const_iterator ParticleSet::begin() const {
    return particles_.begin();
}

std::vector<HistoryEntry *>::const_iterator ParticleSet::end() const {
    return particles_.end();
}

unsigned long ParticleSet::size() {
    return particles_.size();
}

void ParticleSet::add(HistoryEntry* entry) {
    particles_.push_back(entry);
    particleMap_.emplace(entry, particles_.size() - 1);
}

void ParticleSet::remove(HistoryEntry* entry) {
    unsigned long index = particleMap_.at(entry);
    particles_[index] = *particles_.rbegin();
    particles_.pop_back();
    particleMap_.erase(entry);
}

HistoryEntry* ParticleSet::get(unsigned long index) {
    return particles_[index];
}

bool ParticleSet::contains(HistoryEntry* entry) {
    return particleMap_.count(entry) > 0;
}

HistoryEntry* ParticleSet::getRandom(RandomGenerator* randGen) {
    return particles_[std::uniform_int_distribution<unsigned long>(
                             0, particles_.size() - 1)(*randGen)];
}
} /* namespace solver */
