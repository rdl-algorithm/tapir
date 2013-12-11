#ifndef BELIEFNODE_HPP
#define BELIEFNODE_HPP

#include <ctime>                        // for clock_t

#include <map>                          // for map, map<>::value_compare
#include <memory>                       // for unique_ptr
#include <queue>                        // for queue
#include <utility>                      // for pair
#include <vector>                       // for vector

#include "defs.hpp"                     // for RandomGenerator

#include "Action.hpp"                   // for Action
#include "Observation.hpp"              // for Observation

class ActionNode;
class HistoryEntry;
//#include "ActionNode.hpp"               // for ActionNode

class BeliefNode {
  public:
    friend class BeliefTree;
    friend class Solver;
    friend class TextSerializer;

    bool distChecked;
    static long maxParticles;
    static long nStVars;

    /** Constructs a new belief node. */
    BeliefNode();
    /** Constructs a new belief node with the given ID. */
    BeliefNode(long id);
    /** Default destructor. */
    ~BeliefNode();

    /* Copying and moving is disallowed. */
    BeliefNode(BeliefNode const &) = delete;
    BeliefNode(BeliefNode &&) = delete;
    BeliefNode &operator=(BeliefNode const &) = delete;
    BeliefNode &operator=(BeliefNode &&) = delete;

    /** Chooses a next action with the UCB algorithm. */
    Action getUCBAction();
    /** Chooses the action with the best expected value */
    Action getBestAction();
    /** Updates the calculation of which action is optimal. */
    void updateBestValue();
    /** Returns the best q-value */
    double getBestMeanQValue();

    /** Adds the given history entry to this belief node. */
    void add(HistoryEntry *newHistEntry);

    /** Adds a child for the given action and observation;
     * returns the child node, and a boolean representing
     */
    std::pair<BeliefNode *, bool> addChild(Action const &action,
            Observation const &obs);
    /** Samples a particle from this node. */
    HistoryEntry *sampleAParticle(RandomGenerator *randGen);

    /** Updates the q-value for the given action, with the given increase. */
    void updateQValue(Action &action, double increase);
    /** Updates the q-value for the given action, as would occur if replacing
     * the old value with the new value, and reducing the number of particles
     * for the action node if reduceParticles is true.
     */
    void updateQValue(Action &action, double oldValue, double newValue,
            bool reduceParticles);

    /** Calculates the distance between this belief node and another by
     * calculating the average pairwise distance between the individual
     * particles.
     */
    double distL1Independent(BeliefNode *b);

    /** Returns the belief node child corresponding to the given action and
     * observation
     */
    BeliefNode *getChild(Action const &action, Observation const &obs);

    /** Adds all the belief node children of this node to the queue. */
    void enqueueChildren(std::queue<BeliefNode *> &queue);

    /** Returns the next action to attempt. */
    Action getNextActionToTry();


    /** Returns the ID of this node. */
    long getId() {
        return id;
    }
    /** Returns the number of particles in this node. */
    long getNParticles() {
        return nParticles;
    }
    /** Returns the current number of action children of this node. */
    long getNActChildren() {
        return actChildren.size();
    }

  private:
    /** The ID for the next belief node. */
    static long currId;
    /** The exploration parameter for UCB. */
    static double exploreParam;
    /** The startup time */
    static std::clock_t startTime;

    /** The ID of this node. */
    long id;
    /** The number of particles in this node. */
    long nParticles;
    /** The next un-tried action to explore. */
    Action nextActionToTry;
    /** The best mean q-value of any action child. */
    double bestMeanQValue;
    /** The action corresponding to the highest expected value. */
    Action bestAction;

    /** The time at which the last particle was added. */
    double tLastAddedParticle;
    /** The time at which the last belief node comparison was done. */
    double tNNComp;
    /** ??? */
    double tEmdSig;
    /** A previously found near neighbor for this belief node. */
    BeliefNode *nnBel;

    /** The particles for this belief node. */
    std::vector<HistoryEntry *> particles;

    typedef std::map<Action, std::unique_ptr<ActionNode>> ActionMap;
    /** A mapping of actions to action children for this node. */
    ActionMap actChildren;
};

#endif /* BELIEFNODE_HPP */
