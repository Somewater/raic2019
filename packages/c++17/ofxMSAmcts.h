/*
A very simple C++11 Templated MCTS (Monte Carlo Tree Search) implementation with examples for openFrameworks. 

MCTS Code Based on the Java (Simon Lucas - University of Essex) and Python (Peter Cowling, Ed Powley, Daniel Whitehouse - University of York) impelementations at http://mcts.ai/code/index.html
*/

#pragma once

#include "TreeNodeT.h"
#include "MSALoopTimer.h"
#include <cfloat>
#include <sstream>
#include "model/Action.h"

struct DrawSphere {
  double x, y, z;
  double radius;
  double r, g, b;
  double a;
};
extern std::vector<DrawSphere>* debug_draw;
extern std::string* debug_text;

using namespace std;

struct McAction {
  model::Action action;
  bool playout = false;
};

ostream& operator<<(ostream& stream, const model::Action& a);

template <class State>
bool is_best_node_or_parent(msa::mcts::TreeNodeT<State, McAction>* best_node,
    const msa::mcts::TreeNodeT<State, McAction>* node) {
  if (best_node == NULL) return false;
  msa::mcts::TreeNodeT<State, McAction>* n = best_node;
  while (n) {
    if (n == node) {
      return true;
    }
    n = n->get_parent();
  }
  return false;
}

template <class State>
bool is_teammate_by_initial_id(msa::mcts::TreeNodeT<State, McAction>& node) {
  int id = node.get_state().initial_id;
  for (const auto& r : node.get_state().state.robots) {
    if (r.id == id) {
      return r.is_teammate;
    }
  }
  throw runtime_error("Robot not found");
}

template <class State>
void debug_print_tree(msa::mcts::TreeNodeT<State, McAction>& node,
    msa::mcts::TreeNodeT<State, McAction>* best_node = NULL) {
  string indent;
  for (int i = 0; i < node.get_depth(); ++i) indent += "   ";

  auto& action = node.get_action().action;
  float uct_exploitation = (float)node.get_value() / (node.get_num_visits() + FLT_EPSILON);
  float uct_exploration = sqrt( log((float)(
         node.get_parent() ? node.get_parent()->get_num_visits() : node.get_num_visits()
      ) + 1) / (node.get_num_visits() + FLT_EPSILON) );
  float uct_score = uct_exploitation + UCT_K * uct_exploration;

  cout << "|" << indent << node.get_depth() << "."
    << (is_best_node_or_parent(best_node, &node) ? "*" : " ")
    << "[id=" << node.get_state().initial_id
    << " is_teammate=" << is_teammate_by_initial_id(node)
    << " me_id=" << node.get_state().me_id
    << " value=" << uct_exploitation
    << " exploration=" << uct_exploration
    << " uct=" << uct_score
    << " visits=" << node.get_num_visits()
    << action << "]" << "\n";

  for (int i = 0; i < node.get_num_children(); ++i) {
    debug_print_tree(*node.get_child(i), best_node);
  }
}

namespace msa {
    namespace mcts {

		// State must comply with State Interface (see IState.h)
		// Action can be anything (which your State class knows how to handle)
        template <class State, typename Action>
        class UCT {
            typedef TreeNodeT<State, Action> TreeNode;

        private:
            LoopTimer timer;
            int iterations;

        public:
            float uct_k;					// k value in UCT function. default = sqrt(2)
            unsigned int max_iterations;	// do a maximum of this many iterations (0 to run till end)
            unsigned int max_millis;		// run for a maximum of this many milliseconds (0 to run till end)
            unsigned int simulation_depth;	// how many ticks (frames) to run simulation for

            //--------------------------------------------------------------
            UCT() :
                iterations(0),
                uct_k( sqrt(2) ), 
                max_iterations( 100 ),
                max_millis( 0 ),
                simulation_depth( 10 )
            {}


            //--------------------------------------------------------------
            const LoopTimer & get_timer() const {
                return timer;
            }

            const int get_iterations() const {
                return iterations;
            }

            //--------------------------------------------------------------
            // get best (immediate) child for given TreeNode based on uct score
            TreeNode* get_best_uct_child(TreeNode* node) const {
                // sanity check
                if(!node->is_fully_expanded()) return NULL;

                float best_utc_score = -std::numeric_limits<float>::max();
                TreeNode* best_node = NULL;

                // iterate all immediate children and find best UTC score
                int num_children = node->get_num_children();
                for(int i = 0; i < num_children; i++) {
                    TreeNode* child = node->get_child(i);
                    float uct_exploitation = (float)child->get_value() / (child->get_num_visits() + FLT_EPSILON);
                    float uct_exploration = sqrt( log((float)node->get_num_visits() + 1) / (child->get_num_visits() + FLT_EPSILON) );
                    float uct_score = uct_exploitation + uct_k * uct_exploration;

                    if(uct_score > best_utc_score) {
                        best_utc_score = uct_score;
                        best_node = child;
                    }
                }

                return best_node;
            }

          TreeNode* get_best_child(TreeNode* node) const {
              float best_utc_score = -std::numeric_limits<float>::max();
              TreeNode* best_node = NULL;

              // iterate all immediate children and find best UTC score
              int num_children = node->get_num_children();
              for(int i = 0; i < num_children; i++) {
                  TreeNode* child = node->get_child(i);
                  float uct_exploitation = (float)child->get_value() / (child->get_num_visits() + FLT_EPSILON);
                  float uct_exploration = sqrt( log((float)node->get_num_visits() + 1) / (child->get_num_visits() + FLT_EPSILON) );
                  float uct_score = uct_exploitation;// + uct_k * uct_exploration;

                  if(uct_score > best_utc_score) {
                      best_utc_score = uct_score;
                      best_node = child;
                  }
              }

              return best_node;
          }


            //--------------------------------------------------------------
            TreeNode* get_most_visited_child(TreeNode* node) const {
                int most_visits = -1;
                TreeNode* best_node = NULL;

                // iterate all immediate children and find most visited
                int num_children = node->get_num_children();
                for(int i = 0; i < num_children; i++) {
                    TreeNode* child = node->get_child(i);
                    if(child->get_num_visits() > most_visits) {
                        most_visits = child->get_num_visits();
                        best_node = child;
                    }
                }

                return best_node;
            }



            //--------------------------------------------------------------
            Action run(const State& current_state, unsigned int seed = 1, vector<State>* explored_states = nullptr) {
                // initialize timer
                timer.init();

                // initialize root TreeNode with current state
                TreeNode root_node(current_state);

                TreeNode* best_node = NULL;
                int max_depth = 0;
                int max_my_depth = 0;

                // iterate
                iterations = 0;
                while(true) {
                    // indicate start of loop
                    timer.loop_start();

                    // 1. SELECT. Start at root, dig down into tree using UCT on all fully expanded nodes
                    TreeNode* node = &root_node;
                    while(!node->is_terminal() && node->is_fully_expanded()) {
                        node = get_best_uct_child(node);
//						assert(node);	// sanity check
                    }

                    // 2. EXPAND by adding a single child (if not terminal or not fully expanded)
                    if(!node->is_fully_expanded() && !node->is_terminal()) node = node->expand();

                    if (node->get_depth() > max_depth) max_depth = node->get_depth();
                    if (node->get_state().its_me() && node->get_depth() > max_my_depth) max_my_depth = node->get_depth();
                    
                    State state(node->get_state());

                    // 3. SIMULATE (if not terminal)
                    if(!node->is_terminal()) {
                        Action action;
                        for(int t = 0; t < simulation_depth; t++) {
                            if(state.is_terminal()) break;

                            if(state.get_random_action(action)) {
                              state.apply_action(action);
                            } else
                                break;
                        }
                    }

                    // get rewards vector for all agents
                    const std::vector<Evaluation> rewards = state.evaluate();

                    // add to history
                    if(explored_states) explored_states->push_back(state);

                    // 4. BACK PROPAGATION
                    int depth = 0;
                    while(node) {
                        node->update(rewards, depth);
                        node = node->get_parent();
                        depth++;
                    }

                    // indicate end of loop for timer
                    timer.loop_end();

                    // exit loop if current total run duration (since init) exceeds max_millis
#ifndef MY_DEBUG
                    if(max_millis > 0 && timer.check_duration(max_millis)) break;
#endif

                    // exit loop if current iterations exceeds max_iterations
                    if(max_iterations > 0 && iterations > max_iterations) break;
                    iterations++;
                }

                // find most visited child
                best_node = get_best_child(&root_node); // TODO: get_most_visited_child, get_best_uct_child ?

#ifdef MY_DEBUG
                debug_print_tree(root_node, best_node);
#endif

                // return best node's action
                if(best_node) {
                  std::stringstream ss;
                  //best_node->get_state().state.my_score(true);
#ifdef MY_DEBUG
                  cout << "best_value=" << (best_node->get_value() / (best_node->get_num_visits() + FLT_EPSILON))
                    << ", during=" << timer.run_duration_millis() << "ms"
                    << ", iterations=" << iterations
                    << ", visits=" << best_node->get_num_visits()
                    << ", max_depth=" << (max_depth - 1)
                    << ", max_my_depth=" << (max_my_depth - 1)
                    << ";" << endl;
#endif
                  return best_node->get_action();
                }

                // we shouldn't be here
                return Action();
            }


        };
    }
}
