#include "search-strategies.h"
#include <iostream>
#include <vector>
#include <queue>
#include <set>


// ----------------------------- CUSTOM UTILS ----------------------------

#define UNUSED(x) (void)(x) // ignore unused variable warnings

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

// ------------------------- SOLVER IMPLEMENTATIONS ------------------------------

/**
 * @brief BFS For FreeCell game. Work in progress...
 * @param init_state start state
 * @return std::vector<SearchAction> the action path to the solution state, empty vector if no solution found.
 */
std::vector <SearchAction> BreadthFirstSearch::solve(const SearchState &init_state) {
    std::cout << "Running BFS.." << std::endl;
    std::vector <SearchAction> solution;
    std::queue <SearchState> queue;
    std::set <SearchState> visited;
    bool solution_found = false;

    queue.push(init_state);
    while (!queue.empty()) {
        SearchState working_state(queue.front());
        queue.pop();
        std::cout << "Working state looped: queue size: " << queue.size() << std::endl;

        // check if current state is final
        if (working_state.isFinal()) {
            solution_found = true;
            break;
        }

        for (const SearchAction &action: working_state.actions()) {
            // execute all actions
            SearchState new_state(action.execute(working_state));

            if (visited.find(new_state) == visited.end()) {
                visited.insert(new_state);
                queue.push(new_state);
            } else {
                std::cout << "State already visited" << std::endl;
            }
        }
    }


//    if (solution_found){
//        SearchState current_state(queue.back());
//        while(!(current_state == init_state)){
//            // find action that led to current state
//            for(const SearchAction action : current_state.actions()) {
//                auto new_state(action.execute(current_state));
//                if (new_state == current_state){
//                    solution.push_back(action);
//                    current_state = new_state; // FIXME
//                    break;
//                }
//            }
//        }
//    }

    // TODO zatím nevrací cestu akcí, jen vytiskne že našel řešení
    std::cout << "BFS finished: " << solution_found << std::endl;
    return solution;

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
