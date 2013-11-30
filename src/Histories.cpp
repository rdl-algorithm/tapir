#include "Histories.hpp"

#include "HistorySequence.hpp"          // for HistorySequence
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
