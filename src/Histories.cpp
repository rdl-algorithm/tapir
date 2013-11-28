#include "Histories.hpp"

#include <set>
using std::set;
#include <utility>
using std::pair;
#include <vector>
using std::vector;

Histories::Histories() :
            allHistSeq() {
}

Histories::~Histories() {
    reset();
}

void Histories::reset() {
    for (HistorySequence *seq : allHistSeq) {
        delete seq;
    }
    allHistSeq.clear();
}
