/*
A TreeNode in the decision tree.
I tried to keep this independent of UCT/MCTS.
Only contains information / methods related to State, Action, Parent, Children etc. 

*/

#pragma once

#include <memory>
#include <math.h>
#include <vector>
#include <algorithm>
#include <random>
#include <ostream>
#include <iostream>
#include "const.h"

static std::random_device rd;
static std::mt19937 g(rd());

struct Evaluation {
    double value;
    double score;
    double ball_my_min_distance = 0.0;
    double ball_enemy_min_distance = 0.0;
    double ball_my_sum_distance = 0.0;
    double ball_enemy_sum_distance = 0.0;
    double ball_position_z = 0.0;
    double ball_position_x = 0.0;
    double ball_velocity_z = 0.0;
    int my_non_touch = 0;
    int enemy_non_touch = 0;
    double defender_z_pos = 0.0;
    //std::vector<Evaluation> childs;

    Evaluation(
        double score,
        double ball_my_min_distance,
        double ball_enemy_min_distance,
        double ball_my_sum_distance,
        double ball_enemy_sum_distance,
        double ball_position_z,
        double ball_position_x,
        double ball_velocity_z,
        int my_non_touch,
        int enemy_non_touch,
        double defender_z_pos) :
        score(score),
        ball_my_min_distance(ball_my_min_distance),
        ball_enemy_min_distance(ball_enemy_min_distance),
        ball_my_sum_distance(ball_my_sum_distance),
        ball_enemy_sum_distance(ball_enemy_sum_distance),
        ball_position_z(ball_position_z),
        ball_position_x(ball_position_x),
        ball_velocity_z(ball_velocity_z),
        my_non_touch(my_non_touch),
        enemy_non_touch(enemy_non_touch),
        defender_z_pos(defender_z_pos){

        score -= ball_my_min_distance * ball_my_min_distance * 0.02;
        score += ball_enemy_min_distance * ball_enemy_min_distance * 0.02;
        score -= ball_my_sum_distance * ball_my_sum_distance * 0.001;
        score += ball_enemy_sum_distance * ball_enemy_sum_distance * 0.0005;
        score += ball_position_z * 1;
        score += (ball_position_z > 0 ? -1 : 1) * abs(ball_position_x) * 0.5;
        score += ball_velocity_z * 0.1;
        score -= my_non_touch * my_non_touch * my_non_touch;
        score += enemy_non_touch * enemy_non_touch * enemy_non_touch;
        if (defender_z_pos > -15) {
          score -= pow(abs((-15 - defender_z_pos)), 3);
        }
//        std::cout <<
//            "SCORE: ball_my_min_distance=" << (ball_my_min_distance * ball_my_min_distance * 0.02) <<
//            " ball_enemy_min_distance=" << (ball_enemy_min_distance * ball_enemy_min_distance * 0.02) <<
//            " ball_my_sum_distance=" << (ball_my_sum_distance * ball_my_sum_distance * 0.001) <<
//            " ball_enemy_sum_distance=" << (ball_enemy_sum_distance * ball_enemy_sum_distance * 0.0005) <<
//            " ball_position_z=" << (ball_position_z * 1) <<
//            " ball_position_x=" << ((ball_position_z > 0 ? -1 : 1) * abs(ball_position_x) * 0.5) <<
//            " ball_velocity_z=" << (ball_velocity_z * 0.1) <<
//            " my_non_touch=" << (my_non_touch * my_non_touch * my_non_touch) <<
//            " enemy_non_touch=" << (enemy_non_touch * enemy_non_touch * enemy_non_touch) << std::endl;
        value = score * 0.5 + 0.5;
    }

    Evaluation(double v) : value(v) {}

    void add(const Evaluation& other) {
        value += other.value;
        //childs.push_back(other);
    }
    float sum() {
        return value;
    }

    Evaluation negative() const {
        Evaluation c = *this;
        c.value = -value;
        return c;
    }
};

std::ostream& operator<<(std::ostream& stream, const Evaluation& e);

namespace msa {
    namespace mcts {

        template <class State, typename Action>
        class TreeNodeT {
            typedef std::shared_ptr< TreeNodeT<State, Action> > Ptr;

        public:
            //--------------------------------------------------------------
            TreeNodeT(const State& state, TreeNodeT* parent = NULL):
                state(state),
                action(),
                parent(parent),
                agent_id(state.agent_id()),
                num_visits(0),
                value(0),
                depth(parent ? parent->depth + 1 : 0)
            {
                children.reserve(MAX_ACTIONS);
                actions.reserve(MAX_ACTIONS);
            }

            ~TreeNodeT(){
              for (auto& c : children) {
                delete c;
              }
            }


            //--------------------------------------------------------------
            // expand by adding a single child
            TreeNodeT* expand() {
                // sanity check that we're not already fully expanded
                if(is_fully_expanded()) return NULL;

                // sanity check that we don't have more children than we do actions
                //assert(children.size() < actions.size()) ;

                // if this is the first expansion and we haven't yet got all of the possible actions
                if(actions.empty()) {
                    // retrieve list of actions from the state
                    state.get_actions(actions);

                    // randomize the order
                    std::shuffle(actions.begin(), actions.end(), g);
                }

                // add the next action in queue as a child
                return add_child_with_action( actions[children.size()] );
            }


            // depth==0 - this node reward applied
            void update(const std::vector<Evaluation>& rewards, int depth) {
                this->value.add(rewards[agent_id]);
                num_visits++;
            }


            //--------------------------------------------------------------
            // GETTERS
            // state of the TreeNode
            const State& get_state() const { return state; }

            // the action that led to this state
            const Action& get_action() const { return action; }

            // all children have been expanded and simulated
            bool is_fully_expanded() const { return children.empty() == false && children.size() == actions.size(); }

            // does this TreeNode end the search (i.e. the game)
            bool is_terminal() const { return state.is_terminal(); }

            // number of times the TreeNode has been visited
            int get_num_visits() const { return num_visits; }

            // accumulated value (wins)
            float get_value() { return value.sum(); }

            // how deep the TreeNode is in the tree
            int get_depth() const { return depth; }

            // number of children the TreeNode has
            int get_num_children() const { return children.size(); }

            // get the i'th child
            TreeNodeT* get_child(int i) const { return children[i]; }

            // get parent
            TreeNodeT* get_parent() const { return parent; }

        private:
            State state;            // the state of this TreeNode
            Action action;            // the action which led to the state of this TreeNode
            TreeNodeT* parent;        // parent of this TreeNode
            int agent_id;            // agent who made the decision

            int num_visits;            // number of times TreeNode has been visited
            Evaluation value;            // value of this TreeNode
            int depth;

            std::vector< TreeNodeT<State, Action>* > children;    // all current children
            std::vector< Action > actions;            // possible actions from this state


            //--------------------------------------------------------------
            // create a clone of the current state, apply action, and add as child
            TreeNodeT* add_child_with_action(const Action& new_action) {
                // create a new TreeNode with the same state (will get cloned) as this TreeNode
                TreeNodeT* child_node = new TreeNodeT(state, this);

                // set the action of the child to be the new action
                child_node->action = new_action;

                // apply the new action to the state of the child TreeNode
                child_node->state.apply_action(new_action);

                // add to children
                children.push_back(child_node);

                return child_node;
            }

        };

    }
}
