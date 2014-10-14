/** @file choosers.cpp
 *
 * Contains implementations of basic action selection functions.
 */
#include "solver/search/action-choosers/gps_choosers.hpp"

#include "solver/search/action-choosers/choosers.hpp"

#include "solver/BeliefNode.hpp"

#include "solver/mappings/actions/continuous_actions.hpp"

namespace solver {
namespace choosers {

namespace gps_detail {



/** The hierarchy tree used for gps search */
template<class PARENT, size_t DIMENSIONS, size_t ACTION_SIZE, size_t CHILDREN_SIZE>
class GpsHierarchyDataBase: public solver::ChooserDataBase<PARENT> {
	typedef GpsHierarchyDataBase This;
	typedef PARENT Parent;
public:
	typedef ContinuousActionMapEntry ActionEntry;

	/** The number of dimensions this search has */
	static const size_t dimensions = DIMENSIONS;

	/** The number of actions this stencil has */
	static const size_t actionSize = ACTION_SIZE;

	/** the number of children this stencil has.
	 *
	 * Note: usually actionSize==childrenSize. While other patterns are possible in theory,
	 * the search algorithm does currently not accommodate for that.
	 */
	static const size_t childrenSize = CHILDREN_SIZE;

protected:
	/** create the hierarchy data. */
	GpsHierarchyDataBase(): lower(), upper(), actions(), children() {
		lower.fill(0);
		upper.fill(0);
		actions.fill(nullptr);
		//children.fill(nullptr); // this is a uniq_ptr, that gets initialised with null anyway.
	};
	_NO_COPY_OR_MOVE(GpsHierarchyDataBase);


public:

	/** the lower values of the search rectangle. */
	std::array<double, dimensions> lower;

	/** the upper values of the search rectangle. */
	std::array<double, dimensions> upper;

	/** the action map entries we are dealing with here. */
	std::array<ActionEntry*, actionSize> actions;

	/** our children (if any). */
	std::array<std::unique_ptr<Parent>, childrenSize> children;

protected:
	// infrastructure for serialisation.

	/** a constructor to load the hierarchy data from a stream */
	explicit GpsHierarchyDataBase(std::istream& is);

	/** method to save the date to a stream */
	void saveToStream(std::ostream& os) const;
};



/** The hierarchy tree used for compass search */
template<size_t DIMENSIONS>
class CompassHierarchyData final: public GpsHierarchyDataBase<CompassHierarchyData<DIMENSIONS>, DIMENSIONS, 1+2*DIMENSIONS, 1+2*DIMENSIONS> {
	typedef CompassHierarchyData This;
	typedef GpsHierarchyDataBase<CompassHierarchyData<DIMENSIONS>, DIMENSIONS, 1+2*DIMENSIONS, 1+2*DIMENSIONS> Base;
public:
	using Base::dimensions;
	using Base::actionSize;
	using Base::lower;
	using Base::upper;
	using Base::actions;
	using Base::children;

	/** create the hierarchy data. */
	CompassHierarchyData(): point(), radius(0) {
		point.fill(0);
	};

	/** This method initialises the fields to those suitable for the root of the search tree. */
	void initialiseRoot(ContinuousActionMap& map) {
		auto boundingBox = map.getActionPool()->getInitialBoundingBox(map.getOwner());

		if (boundingBox.size() != dimensions) {
			class InitialBoundingBoxHasWrongDimensions: public std::exception {
				virtual const char* what() const noexcept {
					return "The bounding box returned by the action pool has wrong dimensions. There is either a bug or a configuration error.";
				}
			} e;
			throw e;
		}

		radius = std::numeric_limits<double>::infinity();

		for (size_t i=0; i<dimensions; i++) {
			lower[i] = boundingBox[i].first;
			upper[i] = boundingBox[i].second;
			point[i] = (boundingBox[i].first + boundingBox[i].second) / 2;

			double maxRadius = (boundingBox[i].second - boundingBox[i].first) / 2;
			if (radius > maxRadius) {
				radius = maxRadius;
			}
		}

		linkActions(map);

	}

	/** This function calculates the coordinates associated with the stencil.
	 *
	 * The values are used to initialise the actions field.
	 */
	std::array<double,dimensions> calculateActionCoordinates(const size_t index) const {
		// we use the convention that index 0 is the center point. Then we go through the dimensions with the lower points. Then comes the higher points.
		std::array<double, dimensions> result = point;
		if (index > 0) {
			if (index <= dimensions) {
				result[index-1] -= radius;
			} else {
				result[index-dimensions-1] += radius;

			}
		}
		return result;
	}

	void linkActions(ContinuousActionMap& map) {
		for (size_t i=0; i<actionSize; i++) {
			auto coordinates = calculateActionCoordinates(i);
			for (size_t dim=0; dim < dimensions; dim++) {
				if ( (coordinates[dim] < lower[dim]) || (coordinates[dim] > upper[dim]) ) {
					goto label_doNotCreateActionAndContinueWithNextAction; // use goto to break from this loop and continue with the outer one.
				}
			}
			actions[i] = map.createOrGetActionMapEntry(coordinates.data());

			label_doNotCreateActionAndContinueWithNextAction:;
		}
	}

	/** this function determines, whether another child is allowed to be created.
	 *
	 * The purpose is to evaluate the minimumChildCreationDistance option.
	 */
	bool mayCreateChild(const size_t index, const GpsChooserOptions& options) const {
		if (index==0) {
			return (radius > options.minimumChildCreationDistance);
		} else {
			return true;
		}
	}

	/** This function attempts to create the child with the correct index.
	 *
	 */
	void createChild(const size_t index, ContinuousActionMap& map) {

		std::unique_ptr<This>& child = children[index];

		if (child != nullptr) return;

		child = std::make_unique<This>();

		child->lower = lower;
		child->upper = upper;

		child->point = actions[index];

		if (index == 0) {
			child->radius = radius / 2;
		} else {
			child->radius = radius;
		}

		child->linkActions(map);
	}

private:
	/** The current center point. */
	std::array<double, dimensions> point;

	/** The radius if the compass rose */
	double radius = 0;

public:
	// infrastructure for serialisation.

	/** a constructor to load the hierarchy data from a stream */
	explicit CompassHierarchyData(std::istream& is);

	/** method to save the date to a stream */
	void saveToStream(std::ostream& os) const;
};


/** This class is a collection of static functions to perform the Gps search
 *
 * The main reason for making it a class is so the template parameter and typedefs
 * don't have to be repeated all the time. It is more like a templated namespace
 * than an actual class.
 *
 */
template<class GPS_STENCIL>
class GpsSearch {

	typedef GPS_STENCIL GpsStencil;
	typedef ContinuousActionMap ThisActionMap;
	typedef ContinuousActionMapEntry ThisActionMapEntry;
	typedef ContinuousActionConstructionDataBase ActionConstructionData;


	static void ensureChooserDataIsInitialised(ThisActionMap& map) {
		if (map.chooserData == nullptr) {
			map.chooserData = std::make_unique<GpsStencil>();
			static_cast<GpsStencil&>(*map.chooserData).initialiseRoot(map);
		}
	}

	static double calculateUcbScore(const ThisActionMapEntry& entry, const ThisActionMap& map, const GpsChooserOptions& options) {
		return entry.getMeanQValue() + sqrt(options.explorationCoefficient * std::log( map.getTotalVisitCount() ) / entry.getVisitCount()  );
	}

	static void gpsUcbAction_processFixed(BeliefNode const *node, const Model& model, const GpsChooserOptions& options, const ThisActionMap& mapping, double& bestPointScore, ThisActionMapEntry*& bestEntry ) {

		// Process the actions and calculate the ucb score.
		size_t unvisitedEntryCount = 0;
		for (ThisActionMapEntry* entry : mapping.getFixedEntries()) {

			if (entry->getVisitCount() == 0) {
				// count the unvisited entries.
				unvisitedEntryCount++;
			} else {

				// if we already have unvisited entries, the point score doesn't matter anyway, so don't bother calculating it.
				if (unvisitedEntryCount == 0) {

					double pointScore = calculateUcbScore(*entry, mapping, options);
					if (pointScore >= bestPointScore) {
						bestPointScore = pointScore;
						bestEntry = entry;
					}

				}

			}
		}

		// If we have unvisited entries, return one of them at random.
		if (unvisitedEntryCount > 0) {

			// decide which unvisited entry to take.
			size_t selectEntryNumber;
			if (unvisitedEntryCount == 1) {
				selectEntryNumber = 0;
			} else {
				selectEntryNumber = model.getRandomGenerator()->operator()() % unvisitedEntryCount;
			}

			// find the entry again.
			size_t count = 0;
			for (ThisActionMapEntry* currentEntry : mapping.getFixedEntries()) {
				if (currentEntry->getVisitCount() == 0) {
					if (count==selectEntryNumber) {

						// done. We found our entry, return it to the caller.
						bestPointScore = std::numeric_limits<double>::infinity();
						bestEntry = currentEntry;
						return;
					}
					count++;;
				}
			}

		}

	}

	static void gpsUcbAction_processGps(BeliefNode const *node, const Model& model, const GpsChooserOptions& options, ThisActionMap& mapping, double& bestPointScore, ThisActionMapEntry*& bestEntry ) {

		// we traverse the tree. If we decide to descend into a child, currentNode will be set to the child in the next round of the loop.
		GpsStencil* currentNode = static_cast<GpsStencil*>(mapping.chooserData.get());

		// when the loop is finished, currentNode will be null. Thus we keep a copy of it in previousNode so we know which leaf we reached in the end.
		GpsStencil* previousNode = nullptr;

		// We also want to know which child we want to add after the while loop.
		size_t bestEntryIndexAtPreviousNode = 0;

		// we need to know how far we descended into the tree.
		size_t hirarchyLevels = 0;

		// if we encounter actions that are not visited enough, we block child creation.
		bool blockCreationOfNewchild = false;


		while (currentNode != nullptr) {

			// keep track of our progress.
			hirarchyLevels++;
			previousNode = currentNode;


			// check if we have an unvisited entry. If more than one are unvisited, return one at random, otherwise the unvisited one.
			size_t unvisitedCount = 0;
			for (auto& action : currentNode->actions) {
				if ( (action != nullptr) && (action->getVisitCount() == 0) ) unvisitedCount++;
			}


			if (unvisitedCount > 0) {
				size_t selectEntryNumber;
				if (unvisitedCount == 1) {
					selectEntryNumber = 0;
				} else {
					selectEntryNumber = model.getRandomGenerator()->operator()() % unvisitedCount;
				}
				size_t count = 0;
				for (auto& entry : currentNode->actions) {
					if ( (entry != nullptr) && (entry->getVisitCount() == 0) ) {
						if (count == selectEntryNumber) {
							bestPointScore = std::numeric_limits<double>::infinity();
							bestEntry = entry;
							return;
						}
						count++;
					}
				}
			}


			// process the current actions.
			size_t bestEntryIndexAtCurrentNode = 0;
			double bestEntryMeanAtCurrentNode = -std::numeric_limits<double>::infinity();
			size_t index = 0;


			for (ThisActionMapEntry* entry : currentNode->actions) {

				double pointScore;

				if (entry != nullptr) {
					if (entry->getVisitCount() < std::max(long(1), long(options.minimumVisitsBeforeChildCreation))) {
						blockCreationOfNewchild = true;
					}

					pointScore = calculateUcbScore(*entry, mapping, options);

					if (pointScore >= bestPointScore) {
						bestPointScore = pointScore;
						bestEntry = entry;
					}

					if (entry->getMeanQValue() >= bestEntryMeanAtCurrentNode) {
						bestEntryMeanAtCurrentNode = entry->getMeanQValue();
						bestEntryIndexAtCurrentNode = index;
					}
				}

				index++;
			}

			// save the best entry index for future reference
			bestEntryIndexAtPreviousNode = bestEntryIndexAtCurrentNode;

			// Process the next child.
			currentNode = currentNode->children[bestEntryIndexAtCurrentNode].get();

		} // end of while loop


		// We reached a leaf of the tree now. Determine whether we want to create a child.

		// Calculate how deep the hirarchy is allowed to be.
		double desiredHirarchyLevels = 0;
		if (mapping.getTotalVisitCount() > 0) {
			desiredHirarchyLevels = log(mapping.getTotalVisitCount())/log(options.newSearchPointCoefficient);
		}


		// Create the child if we may.
		if (!blockCreationOfNewchild && (hirarchyLevels < desiredHirarchyLevels) && previousNode->mayCreateChild(bestEntryIndexAtPreviousNode, options) ) {

			previousNode->createChild(bestEntryIndexAtPreviousNode, mapping);


			// return an unvisited entry of the child at random.
			// If all are visited, we calculate the point scores and do normal UCP.
			currentNode = previousNode->children[bestEntryIndexAtPreviousNode].get();
			size_t unvisitedCount = 0;
			size_t nonNullCount = 0;
			for (auto& action : currentNode->actions) {
				if (action != nullptr) {
					nonNullCount++;
					if (action->getVisitCount() == 0) unvisitedCount++;
				}
			}

			if (unvisitedCount > 0) {
				// return an unvisited entry at random.
				size_t selectEntryNumber;
				if (unvisitedCount==1) {
					selectEntryNumber = 0;
				} else {
					selectEntryNumber = model.getRandomGenerator()->operator()() % unvisitedCount;
				}
				size_t count = 0;
				for (auto& entry : currentNode->actions) {
					if ( (entry != nullptr) && (entry->getVisitCount() == 0) ) {
						if (count == selectEntryNumber) {
							bestPointScore = std::numeric_limits<double>::infinity();
							bestEntry = entry;
							return;
						}
						count++;
					}
				}
			} else if (nonNullCount > 0){

				// calculate the point scores.
				for (auto& entry : currentNode->actions) {
					if (entry != nullptr) {
						double pointScore = calculateUcbScore(*entry, mapping, options);
						if (pointScore >= bestPointScore) {
							bestPointScore = pointScore;
							bestEntry = entry;
						}
					}
				}

			} else {
				debug::show_message("This should not happen. We must have created a child with only NULL actions.");
			}


		}

	}


public:

	static GpsChooserResponse gpsUcbAction(BeliefNode const *node, const Model& model, const GpsChooserOptions& options) {


		ThisActionMap& mapping = *(static_cast<ThisActionMap*>(node->getMapping()));


		// we traverse the tree to find the best node.
		double bestPointScore = -std::numeric_limits<double>::infinity();
		ThisActionMapEntry* bestEntry = nullptr;
		bool blockCreationOfNewchild = false;

		gpsUcbAction_processFixed(node, model, options, mapping, bestPointScore, bestEntry);

		// return if there is an unvisited fixed action.
		if (bestEntry->getVisitCount() == 0) {
			return GpsChooserResponse(bestEntry->getAction(), false);
		}

		// If Gps search is disabled, return what we have.
		if (options.disableGpsSearch) {
			if (bestEntry == nullptr) {
				class NoFixedActionsAndGpsSearchIsDisabled: public std::exception {
					virtual const char* what() const noexcept {
						return "Gps search is disabled and there are no fixed actions. This can't work. There is probably a wrong configuration option.";
					}
				} e;
				throw e;
			}

			return GpsChooserResponse(bestEntry->getAction(), bestEntry->getVisitCount() > 0);
		}


		ensureChooserDataIsInitialised(mapping);

		gpsUcbAction_processGps(node, model, options, mapping, bestPointScore, bestEntry);


		if (bestEntry == nullptr) {
			class NoActionCouldBeSelected: public std::exception {
				virtual const char* what() const noexcept {
					return "Gps search did not return any usable action. Something is not right.";
				}
			} e;
			throw e;
		}

		return GpsChooserResponse(bestEntry->getAction(), bestEntry->getVisitCount() > 0);

	}

	static GpsChooserResponse gpsMaxAction(BeliefNode const *node, const GpsChooserOptions& options) {


		ThisActionMap& mapping = *(static_cast<ThisActionMap*>(node->getMapping()));


		// variables to store the best current action.
		double bestPointScore = -std::numeric_limits<double>::infinity();
		ThisActionMapEntry* bestEntry = nullptr;

		// we check the fixed actions.
		for (ThisActionMapEntry* entry : mapping.getFixedEntries()) {
			if (entry->getVisitCount() == 0) continue;

			if (entry->getMeanQValue() >= bestPointScore) {
				bestPointScore = entry->getMeanQValue();
				bestEntry = entry;
			}
		}

		if (!options.disableGpsSearch) {

			// we traverse the tree. If we decide to descend into a child, currentNode will be set to the child in the next round of the loop.
			GpsStencil* currentNode = static_cast<GpsStencil*>(mapping.chooserData.get());

			while (currentNode != nullptr) {

				// process the current actions.
				size_t bestCurrentEntryIndex = 0;
				double bestCurrentEntryScore = -std::numeric_limits<double>::infinity();

				// check all the actions.
				for (size_t index=0; index < currentNode->actions.size(); index++) {

					ThisActionMapEntry* entry = currentNode->actions[index];

					if ( (entry != nullptr) && (entry->getVisitCount() > 0)) {

						double meanQValue = entry->getMeanQValue();

						if (meanQValue >= bestPointScore) {
							bestPointScore = meanQValue;
							bestEntry = entry;
						}
						if (meanQValue >= bestCurrentEntryScore) {
							bestCurrentEntryScore = meanQValue;
							bestCurrentEntryIndex = index;
						}
					}
				}

				// process the child next.
				currentNode = currentNode->children[bestCurrentEntryIndex].get();
			}
		}

		if (bestEntry != nullptr) {
			return bestEntry->getAction();
		} else {
			return choosers::max_action(node);
		}

	}


};



template<size_t DIMENSIONS>
void route_gpsUcbAction_compass(GpsChooserResponse& result, BeliefNode const *node, const Model& model, const GpsChooserOptions& options);

template<>
void route_gpsUcbAction_compass<GpsChooserOptions::maxDimensions+1>(GpsChooserResponse& /*result*/, BeliefNode const* /*node*/, const Model& /*model*/, const GpsChooserOptions& /*options*/) {
	class TooManyDimensions: public std::exception {
		virtual const char* what() const noexcept {
			return "route_gpsUcbAction_compass: the dimensionality you requested wasn't instanciated during compile time. Please adjust GpsChooserOptions::maxDimensions.";
		}
	} e;
	throw e;
}

template<size_t DIMENSIONS>
void route_gpsUcbAction_compass(GpsChooserResponse& result, BeliefNode const *node, const Model& model, const GpsChooserOptions& options) {
	if (DIMENSIONS==options.dimensions) {
		result = GpsSearch<CompassHierarchyData<DIMENSIONS>>::gpsUcbAction(node, model, options);
	} else {
		route_gpsUcbAction_compass<DIMENSIONS+1>(result, node, model, options);
	}
}


void route_gpsUcbAction_golden(GpsChooserResponse& result, BeliefNode const *node, const Model& model, const GpsChooserOptions& options) {
	if (options.dimensions==1) {
		debug::show_message("Warning: golden search hasn't been implemented. Using compass search instead.");
		result = GpsSearch<CompassHierarchyData<1>>::gpsUcbAction(node, model, options);
	} else {
		class TooManyDimensions: public std::exception {
			virtual const char* what() const noexcept {
				return "Golden section search only works in one dimension. Please change your options.";
			}
		} e;
		throw e;
	}
}

void route_gpsUcbAction(GpsChooserResponse& result, BeliefNode const *node, const Model& model, const GpsChooserOptions& options) {
	if (options.searchType == GpsChooserOptions::COMPASS) {
		route_gpsUcbAction_compass<1>(result, node, model, options);
	} else if (options.searchType == GpsChooserOptions::GOLDEN) {
		route_gpsUcbAction_golden(result, node, model, options);
	} else {
		class UnknownSearchType: public std::exception {
			virtual const char* what() const noexcept {
				return "the GPS search type you requested is unknown. Please change your options.";
			}
		} e;
		throw e;
	}
}

} // namespace gps_detail


GpsChooserResponse gps_max_action(BeliefNode const *node, const Model& model, const GpsChooserOptions& options) {
#warning not implemented yet.
}


GpsChooserResponse gps_ucb_action(BeliefNode const *node, const Model& model, const GpsChooserOptions& options) {
	GpsChooserResponse result;
	gps_detail::route_gpsUcbAction(result, node, model, options);
	return std::move(result);
}

} /* namespace choosers */
} /* namespace solver */
