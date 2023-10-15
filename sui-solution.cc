#include "search-strategies.h"
#include <iostream>
#include <vector>
#include <queue>
#include <set>
#include <cassert>
#include "mem_watch.h"

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
        //if(this->mem_limit_ >= ) TODO porvnoat jestli se neblížím limitu a skončit jako fail
        SearchState working_state(queue.front());
        visited.insert(working_state);
        queue.pop();
        std::cout << "Working state looped: "<< std::endl << working_state << "\t Queue size: " << queue.size() << "\tVisited size:" << visited.size() << "\tAvailable actions: " << working_state.actions().size() << std::endl;

        // check if current state is final
        if (working_state.isFinal()) {
            solution_found = true;
            break;
        }

        auto visited_ctr = 0;
        auto pushed_ctr = 0;


        for (const SearchAction action : working_state.actions()) {
            // execute all actions
            SearchState new_state(action.execute(working_state));

            if (visited.find(new_state) == visited.end()) {
                visited.insert(new_state);
                queue.push(new_state);
                pushed_ctr++;
            } else {
                visited_ctr++;
            }
        }
        assert(size_t(visited_ctr + pushed_ctr) == working_state.actions().size());
        std::cout << "Visited: " << visited_ctr << " \tPushed: " << pushed_ctr << std::endl;

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
    std::cout << "BFS finished: " << solution_found << " Visited size: "<< visited.size() << " Q size: " << queue.size() <<std::endl;
    print(solution)
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
