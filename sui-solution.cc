#include "search-strategies.h"
#include <queue>
#include <unordered_set>
#include <set>
#include "memusage.h"
#include <algorithm>
#include <stack>
#include <list>
#include <array>
#include "memusage.h"
#include "card.h"
#include "game.h"
#include "card-storage.h"


using namespace std;
// shared pointer to the SearchState object
using StatePointer = shared_ptr<SearchState>;
using CardPointer = shared_ptr<Card>;



// ----------------------------- MACROS --------------------------------------------------------------------------------

// ignore unused variable warnings
#define UNUSED(x) (void)(x)
// debug mode
#define DEBUG 1
// custom print macro for debug mode
#define D_PRINT(x) if (DEBUG) std::cout << x << std::endl;
// memory margin for BFS
#define BFS_MEM_MARGIN 2048
// memory margin for DFS
#define DFS_MEM_MARGIN 2048
// bad state multiplier for A* heuristic - defaults to 2
#define BAD_STATE_MULTIPLIER 2
// base score for Heineman heuristic
#define HEINEMAN_BASE_SCORE 96 // ((13 cards - 1 selected card) * 4 colors) * 2 bad state multiplier = 96



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

/**
 * @brief Get the work stacks (columns) from the GameState object.
 * @param state state to parse
 * @return vector of stacks of cards
 */
vector <vector<Card>> get_workstacks(const GameState &state);



/**
 * @brief Heineman heuristic for FreeCell game.
 * @param gameState state to evaluate
 * @return state score
 */
double heinemanHeuristic(const GameState &gameState);


/**
 * @brief Convert vector of actions to string.
 * @param actions vector of actions
 * @return string of actions
 */
std::string actions_to_str(std::vector <SearchAction> actions);


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


/**
 * @brief Structure used in A* implementation.
 */
struct StateWithCost {
    SearchState state;
    int cost;
    StateWithCost *father;
    const SearchAction action; // action that led to this state

    StateWithCost(SearchState state, int cost, StateWithCost *father, SearchAction action) :
            state(state), cost(cost), father(father), action(action) {}
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


/**
 * @brief A* heuristic for FreeCell game.
 * @param state evaluated state
 * @return score for the state - how close it could be to the solution
 */
double StudentHeuristic::distanceLowerBound(const GameState &state) const {
    return heinemanHeuristic(state);
}


/**
 * @brief A* search for FreeCell game.
 * @param init_state starting state
 * @return vector of SearchActions that lead from the initial state to the final state
 */
std::vector <SearchAction> AStarSearch::solve(const SearchState &init_state) {
    // TODO
    const int CYCLIC_CHECK_SIZE = 50000;
    int EXPAND_COUNT_LIMIT = 10000;
    long unsigned int QUEUE_LIMIT = 500;

    std::vector <SearchAction> return_vec{};
    std::list < StateWithCost * > states{(new StateWithCost{init_state, 0, nullptr, init_state.actions()[0]})};
    std::vector < StateWithCost * > trash{}; // setting popped states aside for clean up later
    StateWithCost *victory = nullptr;
    std::list <std::string> cyclic_check = {};

    int i = 0;
    while (!states.empty()) {
        //std::cout << getCurrentRSS() << " " << mem_limit_ << std::endl;


        // queue management
        if (getCurrentRSS() > mem_limit_ - 2048) goto clean_up;
        if (i++ > EXPAND_COUNT_LIMIT) goto clean_up;
        StateWithCost *father = states.front();
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
            if (actions_str.compare(cyclic_test) == 0) {
                do_continue = 1;
                break;
            }
        if (do_continue) continue;

        cyclic_check.push_front(actions_str);
        if (cyclic_check.size() > CYCLIC_CHECK_SIZE)
            cyclic_check.pop_back();



        // expand
        for (const SearchAction &action: actions) {

            StateWithCost *new_state_with_cost = new StateWithCost{action.execute(father->state), 1, father, action};
            new_state_with_cost->cost = compute_heuristic(new_state_with_cost->state, *heuristic_); // TODO tady dát pointer na heinemanHeuristic


            // find the first state with a bigger cost than the new one
            std::list<StateWithCost *>::iterator SWC_iterator = states.begin();
            long unsigned int i = 0;
            for (; i < states.size() && (*SWC_iterator)->cost < new_state_with_cost->cost; SWC_iterator++) {
                ++i;
                if (i > QUEUE_LIMIT) {
                    delete new_state_with_cost;
                    break;
                }
            }

            if (states.size() > QUEUE_LIMIT) {
                delete states.back();
                states.pop_back();
            }

            // add the new state just before the found one 
            if (i != states.size())
                states.insert(SWC_iterator, new_state_with_cost);
            else
                states.push_front(new_state_with_cost);

            // entering each iteration, states should be arranged in ascending order 


        }

    }

    // reconstruct the victorious path
    if (victory != nullptr) {
        for (StateWithCost *node = victory; node->father != nullptr; node = node->father)
            return_vec.insert(return_vec.begin(), (node->action));
    }


    clean_up:
    for (StateWithCost *s: trash)
        delete s;
    for (StateWithCost *s: states)
        delete s;
    return return_vec;
}



// ------------------------- HEURISTICS IMPLEMENTATION -----------------------------------------------------------------

double heinemanHeuristic(const GameState &gameState) {

    // load base (maximum) score
    double score = HEINEMAN_BASE_SCORE;

    // prepare helper set of all colors
    std::unordered_set <Color> allColors = {Color::Heart, Color::Diamond, Color::Club, Color::Spade};

    // prepare helper map of next home top cards. This holds the info about the next card that should be on top of the home of given color.
    map<Color, int> nextHomeTopCardMap;
    // this loop adds the top cards of the homes to the map and removes their colors from the set (so that we know which colors are missing)
    for (int i = 0; i < nb_homes; i++) {
        auto opt_top = gameState.homes[i].topCard();
        if (opt_top.has_value()) {
            nextHomeTopCardMap[opt_top.value().color] = (opt_top.value().value == 13) ? opt_top.value().value :
                                                        opt_top.value().value + 1;
            allColors.erase(opt_top.value().color);
        }
    }

    // if there are any colors left in the set, add them to the map with value 1 - ace. This means that the home was empty.
    for (auto color: allColors) {
        nextHomeTopCardMap[color] = 1;
    }

    // load work stacks
    vector <vector<Card>> work_stacks = get_workstacks(gameState);

    // calculate penalisation - count cards on top of the target cards in the working stacks
    int penalisation_ctr = 0;
    for (size_t i = 0; i < nb_stacks; i++) {
        for (size_t j = 0; j < work_stacks[i].size(); j++) {
            if (work_stacks[i][j].value == nextHomeTopCardMap[work_stacks[i][j].color]) {
                penalisation_ctr += work_stacks[i].size() - j - 1; // - 1 for index offset
            }
        }
    }

    // check if all the freeCells are occupied
    bool free_cells_are_full = true;
    for (auto i = 0; i < nb_freecells; i++) {
        if (!gameState.free_cells[i].topCard().has_value()) {
            free_cells_are_full = false;
            break;
        }
    }

    // check if all the homes are full
    bool homes_are_full = true;
    for (auto i = 0; i < nb_homes; i++) {
        // if any of the homes is empty, the homes can't be full
        if (!gameState.homes[i].topCard().has_value()) {
            homes_are_full = false;
            break;
        }
    }

    // if all freeCells are full or there is a free home, penalise the state by multiplying (default value is 2)
    if (free_cells_are_full || !homes_are_full) {
        penalisation_ctr = penalisation_ctr * BAD_STATE_MULTIPLIER;
    }

    score -= penalisation_ctr;

    D_PRINT("Heineman heuristic score=" << score << ", extra_badness=" << (free_cells_are_full || !homes_are_full) << " for state:" << endl)
    D_PRINT(gameState)

    return score;
}

// ------------------------- HELPER FUNCTIONS IMPLEMENTATION -----------------------------------------------------------

vector <vector<Card>> get_workstacks(const GameState &state) {
    vector <vector<Card>> result = {};
    for (const auto &work_stack: state.stacks) {
        vector <Card> my_stack = {};
        // load cards
        for (auto card = work_stack.storage().begin(); card != work_stack.storage().end(); card++) {
            my_stack.push_back(*card);
        }
        result.push_back(my_stack);
    }
    return result;
}


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


std::string actions_to_str(std::vector <SearchAction> actions) {
    std::string return_val = "";

    for (SearchAction action: actions) {
        return_val = return_val + std::to_string(action.to().id) + std::to_string(action.from().id);
    }
    return return_val;
}
