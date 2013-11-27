#include "TextSerializer.hpp"

#include <ostream>
#include <vector>

#include "State.hpp"
#include "StateWrapper.hpp"

void TextSerializer::save(StateWrapper &wrapper, std::ostream &os) {
    os << "s " << wrapper.id << " : ";
    std::vector<double>::iterator it;
    for (it = wrapper.s.begin(); it != wrapper.s.end(); it++) {
        os << *it << " ";
    }
}

void TextSerializer::load(StateWrapper *&wrapper, std::istream &is) {
}

void TextSerializer::save(StatePool &pool, std::ostream &os) {
}

void TextSerializer::load(StatePool *&pool, std::istream &is) {
}

void TextSerializer::save(HistoryEntry &entry, std::ostream &os) {
}

void TextSerializer::load(HistoryEntry *&entry, std::istream &is) {
}

void TextSerializer::save(HistorySequence &seq, std::ostream &os) {
}

void TextSerializer::load(HistorySequence *&seq, std::istream &is) {
}

void TextSerializer::save(Histories &histories, std::ostream &os) {
}

void TextSerializer::load(Histories *&histories, std::istream &is) {
}

void TextSerializer::save(ObservationEdge &edge, std::ostream &os) {
}

void TextSerializer::load(ObservationEdge *&edge, std::istream &is) {
}

void TextSerializer::save(ActionNode &node, std::ostream &os) {
}

void TextSerializer::load(ActionNode *&node, std::istream &is) {
}

void TextSerializer::save(BeliefNode &node, std::ostream &os) {
}

void TextSerializer::load(BeliefNode *&node, std::istream &is) {
}

void TextSerializer::save(BeliefTree &tree, std::ostream &os) {
}

void TextSerializer::load(BeliefTree *&tree, std::istream &is) {
}
