#include "search-strategies.h"
#include <iostream>
#include <vector>
#include <queue>
#include <unordered_set>
#include <set>
#include <cassert>
#include "mem_watch.h"
#include "memusage.h"
#include <algorithm>

using namespace std;

// ----------------------------- CUSTOM UTILS ----------------------------

// ignore unused variable warnings
#define UNUSED(x) (void)(x)
// debug mode
#define DEBUG 1
#define SIZE_DEBUG 1
// custom print macro for debug mode
#define D_PRINT(x) if (DEBUG) std::cout << x << std::endl;
#define S_PRINT(x) if (SIZE_DEBUG) std::cout << x << std::endl;
// memory margin for BFS
#define BFS_MEM_MARGIN 50000000


/**
 * @brief Reconstructs the path from the initial state to the final state.
 * @param init_state the input initial state
 * @param final_state the calculated final state
 * @param sourceActionMap the map of parents in the searched tree
 * @return vector of actions leading from init_state to final_state in this order
 */

// ----------------------------- FUNCTIONS ----------------------------

vector <SearchAction> reconstructPath(shared_ptr <SearchState> init_state,
                                      shared_ptr <SearchState> final_state,
                                      map <shared_ptr<SearchState>, shared_ptr<SearchState>> *sourceActionMap);



// ----------------------------- STRUCTURES ----------------------------

/**
 * @brief Compare two shared pointers to SearchState objects.
 * @details This is needed for the set of visited states in BFS.
 */
struct SearchStateSharedPtrCompare {
    bool operator()(const std::shared_ptr <SearchState> &a, const std::shared_ptr <SearchState> &b) const {
        return *a < *b;
    }
};



// ----------------------------- OPERATORS ----------------------------

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
bool operator==(const std::shared_ptr <SearchState> &lhs, const std::shared_ptr <SearchState> &rhs) {
    if (!lhs && !rhs) {
        return true;  // Both shared pointers are null, consider them equal
    }
    if (!lhs || !rhs) {
        return false;  // One of the shared pointers is null, consider them not equal
    }
    return *lhs == *rhs;  // Compare the dereference values
}



// ----------------------------- EXCEPTIONS ----------------------------

/**
 * @brief Not implemented exception for missing implementations of search strategies.
 */
class NotImplementedException : public std::logic_error {
public:
    NotImplementedException() : std::logic_error("Not implemented yet!") {}
};




// ------------------------- SOLVER IMPLEMENTATIONS ------------------------------

/**
 * @brief BFS For FreeCell game. Work in progress...
 * @param init_state start state
 * @return std::vector<SearchAction> the action path to the solution state, empty vector if no solution found.
 */
std::vector <SearchAction> BreadthFirstSearch::solve(const SearchState &init_state) {

    D_PRINT("BFS: Starting search with MEM_LIMIT=" << this->mem_limit_ << " bytes and MEM_MARGIN=" << BFS_MEM_MARGIN
                                                   << " bytes.");

    queue <shared_ptr<SearchState>> queue;
    set <shared_ptr<SearchState>, SearchStateSharedPtrCompare> visited;
    map <shared_ptr<SearchState>, shared_ptr<SearchState>> sourceActionMap;

    shared_ptr <SearchState> init_state_shared_ptr = make_shared<SearchState>(init_state);
    queue.push(init_state_shared_ptr);
    visited.insert(init_state_shared_ptr);

    while (!queue.empty()) {

        shared_ptr <SearchState> current_state = queue.front();
        queue.pop();

        for (const SearchAction action: current_state->actions()) {

            shared_ptr <SearchState> new_state = make_shared<SearchState>(action.execute(*current_state));

            if (new_state->isFinal()) {
                int q_size = sizeof(queue) + queue.size() * sizeof(queue.front());
                int v_size = sizeof(visited) + visited.size() * sizeof(visited.begin());
                int s_size = sizeof(sourceActionMap) + (sourceActionMap.size() * (sizeof(sourceActionMap.begin()) * 2));
                S_PRINT("BFS: Found solution using lookahead!" << endl
                                                               << "Total memory used: " << getCurrentRSS() << " bytes." << endl
                                                               << "Queue: " << q_size << " bytes." << endl
                                                                << "Visited: " << v_size << " bytes." << endl
                                                                << "SourceActionMap: " << s_size << " bytes." << endl
                                                                << "Missing: " << getCurrentRSS() - (q_size + v_size + s_size) << " bytes." << endl)
                sourceActionMap[new_state] = current_state;
                return reconstructPath(init_state_shared_ptr, new_state, &sourceActionMap);
            }

            if (visited.find(new_state) == visited.end()) {
                visited.insert(new_state);
                if (sourceActionMap.find(new_state) == sourceActionMap.end()) {
                    sourceActionMap[new_state] = current_state;
                } else {
                    D_PRINT("BFS: Duplicate solution found, aborting search!")
                    return vector<SearchAction>();
                }
                queue.push(new_state);
            }
        }

        if (getCurrentRSS() > mem_limit_ - BFS_MEM_MARGIN) {
            D_PRINT("BFS: Memory limit reached, aborting search!")
            return vector<SearchAction>();
        }
    }

    D_PRINT("BFS: No solution found!")
    return vector<SearchAction>();
}


/**
 * @brief algorithm for reconstructing the path from the initial state to the final state.
 * @param init_state shared_ptr to the initial state
 * @param final_state shared_ptr to the final state
 * @param sourceActionMap child-parent map of the searched tree
 * @return vector of SearchActions that lead from the initial state to the final state
 */
vector <SearchAction> reconstructPath(shared_ptr <SearchState> init_state,
                                      shared_ptr <SearchState> final_state,
                                      map <shared_ptr<SearchState>, shared_ptr<SearchState>> *sourceActionMap) {
    D_PRINT("Reconstructing path.")
    vector <SearchAction> path;
    shared_ptr <SearchState> current_state = final_state;
    shared_ptr <SearchState> parent_state;
    shared_ptr <SearchState> new_state;

    while (current_state != init_state) {

        parent_state = (*sourceActionMap)[current_state];

        for (SearchAction action: parent_state->actions()) {

            new_state = make_shared<SearchState>(action.execute(*parent_state));

            if (new_state == current_state) {
                path.push_back(action);
                break;
            }
        }

        current_state = parent_state;
    }

    reverse(path.begin(), path.end());

    return path;
}


std::vector <SearchAction> DepthFirstSearch::solve(const SearchState &init_state) {

}

double StudentHeuristic::distanceLowerBound(const GameState &state) const {
    // TODO
    UNUSED(state), throw NotImplementedException();
}

std::vector <SearchAction> AStarSearch::solve(const SearchState &init_state) {
    // TODO
    UNUSED(init_state), throw NotImplementedException();
}
