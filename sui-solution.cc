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

#define UNUSED(x) (void)(x) // ignore unused variable warnings
#define DEBUG 0
#define D_PRINT(x) if (DEBUG) std::cout << x << std::endl;

vector <SearchAction> reconstructPath(SearchState init_state,
                                      SearchState final_state,
                                      map <shared_ptr<SearchState>, shared_ptr<SearchAction>> *sourceAction);


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


typedef struct StatePair {
    SearchState state;
    SearchAction action;

    bool operator<(const StatePair &other) const {
        return state < other.state;
    }

} StatePair_t;



//typedef std::pair <SearchState, SearchAction> SearchNode;

// ------------------------- SOLVER IMPLEMENTATIONS ------------------------------

/**
 * @brief BFS For FreeCell game. Work in progress...
 * @param init_state start state
 * @return std::vector<SearchAction> the action path to the solution state, empty vector if no solution found.
 */



std::vector <SearchAction> BreadthFirstSearch::solve(const SearchState &init_state) {

    queue <shared_ptr<SearchState>> queue;
    unordered_set <shared_ptr<SearchState>> visited;
    map <shared_ptr<SearchState>, shared_ptr<SearchState>> sourceAction;

    queue.push(make_shared<SearchState>(init_state));
    visited.insert(make_shared<SearchState>(init_state));

    while (!queue.empty()) {
        shared_ptr<SearchState> current_state = queue.front();
        queue.pop();

        if (current_state->isFinal()) {
            cout << "Found solution!" << endl;
            return vector<SearchAction>();
        }

        for (SearchAction action : current_state->actions()) {
            shared_ptr<SearchState> new_state = make_shared<SearchState>(action.execute(*current_state));
            if (visited.find(new_state) == visited.end()) {
                visited.insert(new_state);
                sourceAction[new_state] = current_state;
                queue.push(new_state);
            }
        }
        cout << getCurrentRSS() << endl;
    }
}




vector <SearchAction> reconstructPath(SearchState init_state,
                                      SearchState final_state,
                                      map <shared_ptr<SearchState>, shared_ptr<SearchAction>> *sourceAction) {
    return vector<SearchAction>();
//    vector <SearchAction> path;
//    SearchState current_state = final_state;
//    while (current_state != init_state) {
//        SearchAction action = *(sourceAction[current_state]);
//        path.push_back(action);
//        current_state = action.from();
//    }
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
