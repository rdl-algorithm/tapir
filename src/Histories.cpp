#include "Histories.hpp"

#include <fstream>
using std::ifstream;
#include <ostream>
using std::ostream;
#include <set>
using std::set;
#include <string>
using std::getline;
using std::string;
#include <sstream>
using std::stringstream;
#include <utility>
using std::pair;
#include <vector>
using std::vector;

#include "HistoryEntry.hpp"
#include "StatePool.hpp"
#include "StateWrapper.hpp"
#include "TextSerializer.hpp"

Histories::Histories() :
            allHistSeq() {
}

Histories::~Histories() {
    clear();
}

void Histories::clear() {
    for (HistorySequence *seq : allHistSeq) {
        delete seq;
    }
    allHistSeq.clear();
}
