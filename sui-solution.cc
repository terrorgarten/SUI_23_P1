#include "search-strategies.h"
#include <iostream>
#include <vector>
#include <queue>
#include <set>
#include <list>
#include "memusage.h"

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

std::string actions_to_str(std::vector<SearchAction> actions){
    std::string return_val = "";

    for(SearchAction action: actions){
       return_val = return_val + std::to_string(action.to().id) +  std::to_string(action.from().id);
    }
    return return_val;
}

std::vector <SearchAction> AStarSearch::solve(const SearchState &init_state) {
    // TODO
    const int CYCLIC_CHECK_SIZE = 50000;
    int EXPAND_COUNT_LIMIT = 10000;
    long unsigned int QUEUE_LIMIT = 500;

    std::vector <SearchAction> return_vec{};
    std::list<StateWithCost *> states {(new StateWithCost {init_state, 0, nullptr, init_state.actions()[0]})};
    std::vector<StateWithCost *> trash{}; // setting popped states aside for clean up later
    StateWithCost * victory = nullptr;
    std::list<std::string> cyclic_check = {};

    int i = 0;
    while(!states.empty()){
        //std::cout << getCurrentRSS() << " " << mem_limit_ << std::endl;
        if (getCurrentRSS() > mem_limit_ - 2048) {
            return std::vector<SearchAction>();
        }
        // queue management
        if(i++>EXPAND_COUNT_LIMIT) goto clean_up;
        StateWithCost * father = states.front();
        states.pop_front();
        trash.push_back(father);
        if (father->state.isFinal()) {
            victory = father;
            break;
        }

        std::vector <SearchAction> actions = father->state.actions();
        std::string actions_str = actions_to_str(actions);
        bool do_continue = 0;
        for (auto cyclic_test: cyclic_check)
            if(actions_str.compare(cyclic_test) == 0){ 
                do_continue = 1;
                break;
            }
        if(do_continue) continue;
        
        cyclic_check.push_front(actions_str);
        if(cyclic_check.size()>CYCLIC_CHECK_SIZE)
            cyclic_check.pop_back();



        // expand
        for (const SearchAction &action: actions) {
            
            StateWithCost * new_state_with_cost = new StateWithCost{action.execute(father->state), 1, father, action};
            new_state_with_cost->cost = compute_heuristic(new_state_with_cost->state, *heuristic_);
            
            
            // find the first state with a bigger cost than the new one
            std::list<StateWithCost *>::iterator SWC_iterator = states.begin();
            long unsigned int i = 0;
            for (;i<states.size() && (*SWC_iterator)->cost < new_state_with_cost->cost ; SWC_iterator++){
                ++i;
                if(i > QUEUE_LIMIT){
                    delete new_state_with_cost;
                    break;
                }
            }

            if(states.size() > QUEUE_LIMIT){
                delete states.back();
                states.pop_back();
            }
            
            // add the new state just before the found one 
            if(i!=states.size())
                states.insert(SWC_iterator, new_state_with_cost );
            else
                states.push_front(new_state_with_cost);
            
            // entering each iteration, states should be arranged in ascending order 
            
            
        }

    }

    // reconstruct the victorious path
    if (victory!=nullptr){
        for(StateWithCost * node = victory; node->father!=nullptr; node=node->father)
           return_vec.insert(return_vec.begin(), (node->action)); 
    }


clean_up:
    for(StateWithCost * s: trash)
        delete s;
    for(StateWithCost * s: states)
        delete s;
    return return_vec;
}
