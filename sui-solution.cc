#include "search-strategies.h"
#include <queue>
#include <unordered_set>
#include <set>
#include "memusage.h"
#include <algorithm>
#include <stack>
#include <list>



using namespace std; // FIXME possible - bad practice, but saves a lot of typing
// shared pointer to the SearchState object
using StatePointer = shared_ptr<SearchState>;

struct StateWithCost{
    SearchState state;
    int cost;
    StateWithCost *father;
    const SearchAction action; // action that led to this state
};



// ----------------------------- MACROS --------------------------------------------------------------------------------

// ignore unused variable warnings
#define UNUSED(x) (void)(x)
// debug mode
#define DEBUG 1
// custom print macro for debug mode
#define D_PRINT(x) if (DEBUG) std::cout << x << std::endl;
// memory margin for BFS
#define BFS_MEM_MARGIN 50000000
// memory margin for DFS
#define DFS_MEM_MARGIN 50000000



// ----------------------------- FUNCTION DECLARATIONS -----------------------------------------------------------------

/**
 * @brief Reconstructs the path from the initial state to the final state.
 * @param init_state the input initial state
 * @param final_state the calculated final state
 * @param sourceActionMap the map of parents in the searched tree
 * @return vector of actions leading from init_state to final_state in this order
 */
vector <SearchAction> reconstructPath(StatePointer init_state,
                                      StatePointer final_state,
                                      map <StatePointer, StatePointer> *sourceActionMap);



// ----------------------------- STRUCTURES ----------------------------------------------------------------------------

/**
 * @brief Structure for storing the state, its parent and the depth of the state in the tree. Used for DFS.
 */
struct DLSState {
    const StatePointer state;
    const StatePointer parent;
    const short depth;

    DLSState(StatePointer state, StatePointer parent, int depth) :
            state(state), parent(parent), depth(depth) {}
};


// ----------------------------- OPERATORS -----------------------------------------------------------------------------

/**
 * @brief Compare two SearchState objects.
 * @param lhs first SearchState
 * @param rhs second SearchState
 * @return true if they refer to the same state, false otherwise
 */
bool operator==(const SearchState &lhs, const SearchState &rhs) {
    return lhs.state_ == rhs.state_;
}


/**
 * @brief Compare two shared pointers to SearchState objects by their referenced SearchState value.
 * @param lhs
 * @param rhs
 * @return
 */
bool operator==(const StatePointer &lhs, const StatePointer &rhs) {
    if (!lhs && !rhs) {
        return true;  // Both shared pointers are null, consider them equal
    }
    if (!lhs || !rhs) {
        return false;  // One of the shared pointers is null, consider them not equal
    }
    return *lhs == *rhs;  // Compare the dereference values
}



// ----------------------------- EXCEPTIONS ----------------------------------------------------------------------------

/**
 * @brief Not implemented exception for missing implementations of search strategies.
 */
class NotImplementedException : public std::logic_error {
public:
    NotImplementedException() : std::logic_error("Not implemented yet!") {}
};




// ------------------------- SOLVER FUNCTIONS IMPLEMENTATION -----------------------------------------------------------


/**
 * @brief BFS For FreeCell game. Work in progress...
 * @param init_state start state
 * @return std::vector<SearchAction> the action path to the solution state, empty vector if no solution found.
 */
std::vector <SearchAction> BreadthFirstSearch::solve(const SearchState &init_state) {

    D_PRINT("BFS: Starting search with MEM_LIMIT=" << this->mem_limit_ << " bytes and MEM_MARGIN=" << BFS_MEM_MARGIN
                                                   << " bytes.");
    // main state handling queue
    queue <StatePointer> queue;
    // visited state set - set has only unique elements. Could use unordered set for possible but small performance boost.
    set <StatePointer> visited;
    // map of child-parent states for reconstructing the path
    map <StatePointer, StatePointer> sourceActionMap;
    // state for holding the current iteration state
    StatePointer current_state;

    // initialize shared pointer to initial state, push it to queue and mark it as visited
    StatePointer init_state_shared_ptr = make_shared<SearchState>(init_state);
    queue.push(init_state_shared_ptr);
    visited.insert(init_state_shared_ptr);

    while (!queue.empty()) {

        // get the next state
        current_state = queue.front();
        queue.pop();

        // cycle through its actions
        for (const SearchAction action: current_state->actions()) {

            // execute the action and create a new state
            StatePointer new_state = make_shared<SearchState>(action.execute(*current_state));

            // if the new state is the final state, add it to parent mapping, reconstruct path and return the path.
            if (new_state->isFinal()) {
                sourceActionMap[new_state] = current_state;
                return reconstructPath(init_state_shared_ptr, new_state, &sourceActionMap);
            }

            // if the state is not yet visited, add it to visited, add it to parent mapping and push it to queue
            if (visited.find(new_state) == visited.end()) {
                visited.insert(new_state);
                sourceActionMap[new_state] = current_state;
                queue.push(new_state);
            }
        }

        // check for available memory with every iteration. If the limit is reached, abort the search.
        if (getCurrentRSS() > mem_limit_ - BFS_MEM_MARGIN) {
            D_PRINT("BFS: Memory limit reached, aborting search!")
            return vector<SearchAction>();
        }
    }

    // loop went through all states and no solution was found, therefore it does not exist. Return empty vector.
    D_PRINT("BFS: No solution found!")
    return vector<SearchAction>();
}


/**
 * @brief DFS For FreeCell game, can use the limited depth parameter.
 * @param init_state The starting state
 * @return vector of SearchActions that lead from the initial state to the final state
 */
std::vector <SearchAction> DepthFirstSearch::solve(const SearchState &init_state) {

    // stack for storing the handled states - uses DLSState structure for storing the depth
    stack <shared_ptr<DLSState>> stack;
    // mapping of child-parent states for reconstructing the path
    map <StatePointer, StatePointer> sourceActionMap;
    // create helper iteration DLSState pointer
    shared_ptr <DLSState> current_dls_state;

    // initialize shared pointer to initial state, push it to stack
    const StatePointer init_state_ptr = make_shared<SearchState>(init_state);
    stack.push(make_shared<DLSState>(DLSState(init_state_ptr, init_state_ptr, 0)));

    while (!stack.empty()) {
        // get the next state, add it to mapping and pop it from the stack
        current_dls_state = stack.top();
        sourceActionMap[current_dls_state->state] = current_dls_state->parent;
        stack.pop();

        // check depth limit with each iteration
        if (current_dls_state->depth > depth_limit_) {
            continue;
        } else {
            // create new states from the current state's actions and push them to stack, unless they're final.
            for (const SearchAction action: current_dls_state->state->actions()) {

                StatePointer new_state = make_shared<SearchState>(action.execute(*current_dls_state->state));

                // if the state is final, add it to parent mapping, reconstruct path and return the path.
                if (new_state->isFinal()) {

                    D_PRINT("DLS: Found solution using lookahead!");
                    sourceActionMap[new_state] = current_dls_state->state;
                    return reconstructPath(init_state_ptr, new_state, &sourceActionMap);

                } else {

                    stack.push(make_shared<DLSState>(DLSState(new_state, current_dls_state->state,
                                                              current_dls_state->depth + 1)));
                }
            }
        }

        // check for available memory with every iteration. If the limit is reached, abort the search.
        if (getCurrentRSS() > mem_limit_ - DFS_MEM_MARGIN) {
            D_PRINT("BFS: Memory limit reached, aborting search!")
            return vector<SearchAction>();
        }
    }
    // loop went through all states and no solution was found, therefore it does not exist. Return empty vector.
    return vector<SearchAction>();
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
    const int CYCLIC_CHECK_SIZE = 100;
    std::cout << "f" <<std::endl;

    std::vector <SearchAction> return_vec{};
    std::list<StateWithCost *> states {(new StateWithCost {init_state, 0, nullptr, init_state.actions()[0]})};
    std::vector<StateWithCost *> trash{}; // setting popped states aside for clean up later
    StateWithCost * victory = nullptr;
    std::list<std::string> cyclic_check = {};

    int i = 0;
    while(!states.empty()){
        // queue management
        if(i++>10000) goto clean_up;
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
        //if(cyclic_check.size()>CYCLIC_CHECK_SIZE)
        //    cyclic_check.pop_back();



        // expand
        for (const SearchAction &action: actions) {

            StateWithCost * new_state_with_cost = new StateWithCost{action.execute(father->state), 1, father, action};
            new_state_with_cost->cost = compute_heuristic(new_state_with_cost->state, *heuristic_);


            // find the first state with a bigger cost than the new one
            std::list<StateWithCost *>::iterator SWC_iterator = states.begin();
            long unsigned int i = 0;
            for (;i<states.size() && (*SWC_iterator)->cost < new_state_with_cost->cost ; SWC_iterator++)
                ++i;

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


// ------------------------- HELPER FUNCTIONS IMPLEMENTATION -----------------------------------------------------------

vector <SearchAction> reconstructPath(StatePointer init_state,
                                      StatePointer final_state,
                                      map <StatePointer, StatePointer> *sourceActionMap) {
    D_PRINT("Reconstructing path.")
    // output path
    vector <SearchAction> path;
    // state pointers for iterating through the tree
    StatePointer current_state = final_state;
    StatePointer parent_state;
    StatePointer new_state;

    // iterate through the tree from the final state to the initial state
    while (current_state != init_state) {

        parent_state = (*sourceActionMap)[current_state];

        // iterate through the parent state's actions and find the one that leads to the current child state
        for (SearchAction action: parent_state->actions()) {

            new_state = make_shared<SearchState>(action.execute(*parent_state));

            if (new_state == current_state) {
                path.push_back(action);
                break;
            }
        }

        current_state = parent_state;
    }

    // reverse the path, so it leads from the initial state to the final state and not the other way around, then return it.
    reverse(path.begin(), path.end());
    return path;
}