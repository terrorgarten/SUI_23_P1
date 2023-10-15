#include "search-strategies.h"
#include <iostream>
#include <vector>
#include <queue>
#include <set>
#include <cassert>
#include "mem_watch.h"

// ----------------------------- CUSTOM UTILS ----------------------------

#define UNUSED(x) (void)(x) // ignore unused variable warnings
#define DEBUG 0
#define D_PRINT(x) if (DEBUG) std::cout << x << std::endl;

/**
 * @brief Not implemented exception for missing implementations of search strategies.
 */
class NotImplementedException : public std::logic_error {
public:
    NotImplementedException() : std::logic_error("Not implemented yet!") {}
};


/**
 * @brief SearchState-SearchState comparison operator overload
 * @param a SearchState a
 * @param b SearchState b
 * @return true if they describe the same game state, false otherwise
 */
bool operator==(const SearchState &a, const SearchState &b) {
    return a.state_ == b.state_;
}


// typedef std::pair <SearchState, SearchAction> SearchNode;

// ------------------------- SOLVER IMPLEMENTATIONS ------------------------------

/**
 * @brief BFS For FreeCell game. Work in progress...
 * @param init_state start state
 * @return std::vector<SearchAction> the action path to the solution state, empty vector if no solution found.
 */
std::vector <SearchAction> BreadthFirstSearch::solve(const SearchState &init_state) {

    // if the input node is final, return it.
    if (init_state.isFinal()) {
        return std::vector<SearchAction>();
    }

    // declare the frontier and add the first node to it
    std::queue <SearchState> frontier;
    frontier.push(init_state);
    // declare the explored set
    std::set <SearchState> explored;
    // declare the solution reference
    SearchState solutionState = init_state;

    while (!frontier.empty()) {

        // get the next node to explore and remove it from the frontier
        SearchState workingState(frontier.front());
        frontier.pop();
        // add the node to the explored set
        explored.insert(workingState);


        for (SearchAction nextAction: workingState.actions()) {

            SearchState child(nextAction.execute(workingState));

            // check for self-loops
            if (workingState == child) { // TODO check if this slows up too much
                continue;
            }

            if (explored.find(workingState) == explored.end() || frontier.find(workingState) == frontier.end()) {
                // if the child is final, return it
                if (child.isFinal()) {
                    solutionState = child;
                    break;
                }
                // add the child and its parent to the frontier TODO optimize for pointer
                frontier.push(child);
            }
        }
    }

    if (solutionState == init_state) {
        return std::vector<SearchAction>();
    } else {
        // TODO return the path to the solution
        std::cout << "naslo se!" << std::endl;
    }
}



    std::vector <SearchAction> DepthFirstSearch::solve(const SearchState &init_state) {
        // TODO
        UNUSED(init_state), throw NotImplementedException();
    }

    double StudentHeuristic::distanceLowerBound(const GameState &state) const {
        // TODO
        UNUSED(state), throw NotImplementedException();
    }

    std::vector <SearchAction> AStarSearch::solve(const SearchState &init_state) {
        // TODO
        UNUSED(init_state), throw NotImplementedException();
    }
