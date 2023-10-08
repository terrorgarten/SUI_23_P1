#include "search-strategies.h"
#include <iostream>
#include <vector>
#include <queue>
#include <set>
#include <list>

struct StateWithCost{
    SearchState state;
    int cost;
    StateWithCost *father;
    const SearchAction action; // action that led to this state
};


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
    
    std::list<StateWithCost *> states {(new StateWithCost {init_state, 0, nullptr, init_state.actions()[0]})};
    std::vector<StateWithCost *> trash{}; // setting popped states aside for clean up later
    StateWithCost * victory = nullptr;


    while(!states.empty()){
        StateWithCost * father = states.front();
        states.pop_front();
        trash.push_back(father);
        if (father->state.isFinal()) {
            victory = father;
            break;
        }

        std::vector <SearchAction> actions = father->state.actions();
        for (const SearchAction &action: actions) {
            StateWithCost * new_state_with_cost = new StateWithCost{action.execute(father->state), 1, father, action};
            new_state_with_cost->cost = compute_heuristic(new_state_with_cost->state, *heuristic_);
            
            // in case list is empty, just insert
            if(states.empty()){
                states.push_front(new_state_with_cost);
                continue;
            }

            
            // find the first state with a bigger cost than the new one
            std::list<StateWithCost *>::iterator SWC_iterator = states.begin();
            for (;(*SWC_iterator)->cost < new_state_with_cost->cost; SWC_iterator++);
            
            // add the new state just before the found one 
            states.insert(SWC_iterator, new_state_with_cost );

            // entering each iteration, states should be arranged in ascending order 
            
            
        }

    }

    // reconstruct the victorious path
    std::vector <SearchAction> return_vec{};
    if (victory!=nullptr){
        for(StateWithCost * node = victory; node->father!=nullptr; node=node->father)
           return_vec.insert(return_vec.begin(), (node->action)); 
    }


    // clean up
    for(StateWithCost * s: trash)
        delete s;
    for(StateWithCost * s: states)
        delete s;
    init_state.actions();
    return return_vec;
}
