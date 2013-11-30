#ifndef SERIALIZER_HPP
#define SERIALIZER_HPP

#include <iosfwd>

#include "Observation.hpp"
#include "Solver.hpp"
class ActionNode;
class BeliefNode;
class BeliefTree;
class Histories;
class HistoryEntry;
class HistorySequence;
class ObservationEdge;
class State;
class StatePool;
class StateWrapper;

class Serializer {
public:
    Serializer(Solver *solver) :
                solver(solver) {
    }
    virtual ~Serializer() = default;
    Serializer(const Serializer&) = delete;
    Serializer(Serializer&) = delete;
    Serializer &operator=(const Serializer&) = delete;
    Serializer &operator=(Serializer&) = delete;

    virtual void save(std::ostream &os) {
        save(*(solver->allStates), os);
        save(*(solver->allHistories), os);
        save(*(solver->policy), os);
    }
    virtual void load(std::istream &is) {
        load(*(solver->allStates), is);
        load(*(solver->allHistories), is);
        load(*(solver->policy), is);
    }

    virtual void save(State &state, std::ostream &os) = 0;
    virtual void load(State &state, std::istream &is) = 0;
    virtual void save(StateWrapper &wrapper, std::ostream &os) = 0;
    virtual void load(StateWrapper &wrapper, std::istream &is) = 0;
    virtual void save(StatePool &pool, std::ostream &os) = 0;
    virtual void load(StatePool &pool, std::istream &is) = 0;

    virtual void save(Observation &obs, std::ostream &os) = 0;
    virtual void load(Observation &obs, std::istream &is) = 0;
    virtual void save(HistoryEntry &entry, std::ostream &os) = 0;
    virtual void load(HistoryEntry &entry, std::istream &is) = 0;
    virtual void save(HistorySequence &seq, std::ostream &os) = 0;
    virtual void load(HistorySequence &seq, std::istream &is) = 0;
    virtual void save(Histories &histories, std::ostream &os) = 0;
    virtual void load(Histories &histories, std::istream &is) = 0;

    virtual void save(ObservationEdge &edge, std::ostream &os) = 0;
    virtual void load(ObservationEdge &edge, std::istream &is) = 0;
    virtual void save(ActionNode &node, std::ostream &os) = 0;
    virtual void load(ActionNode &node, std::istream &is) = 0;
    virtual void save(BeliefNode &node, std::ostream &os) = 0;
    virtual void load(BeliefNode &node, std::istream &is) = 0;
    virtual void save(BeliefTree &tree, std::ostream &os) = 0;
    virtual void load(BeliefTree &tree, std::istream &is) = 0;
protected:
    Solver *solver;
};

#endif /* SERIALIZER_HPP */
