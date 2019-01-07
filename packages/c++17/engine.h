#pragma once

#include "model/Game.h"
#include "model/Action.h"
#include "model/Robot.h"
#include "model/Rules.h"
#include "model/Ball.h"
#include "model/NitroPack.h"
#include "vector3d.h"
#include <optional>
#include <chrono>
#include <iostream>
#include <random>
#include "ofxMSAmcts.h"
#include "const.h"
#include <map>

#ifndef MYSTRATEGY_ENGINE_H
#define MYSTRATEGY_ENGINE_H

using namespace model;
using namespace std;
using namespace chrono;
using namespace msa::mcts;

static std::random_device rd2;
static std::mt19937 g2(rd2());

struct GameState {
  int my_score = 0;
  int enemy_score = 0;
  int current_tick = 0;

  static GameState from_game(const Game& game) {
    GameState g;
    for (auto& player : game.players) {
      if (player.me) {
        g.my_score = player.score;
      } else {
        g.enemy_score = player.score;
      }
    }
    g.current_tick = game.current_tick;
    return g;
  }
};

struct HistoryItem {
    int current_tick;
    Action action;
};

class Entity {
public:
  Entity(
      Vector3D position,
      double radius,
      double mass,
      Vector3D velocity,
      double radius_change_speed) :
        position(position),
        radius(radius),
        mass(mass),
        velocity(velocity),
        radius_change_speed(radius_change_speed) {}

  Vector3D position;
  double radius;
  double mass;
  Vector3D velocity;
  double radius_change_speed;
  double arena_e = 0;
  bool is_robot;
  bool is_ball;
};

struct CollideEntities {
  Entity entity1; // robot
  Entity entity2; // robot or ball
};

struct CollideArena {
  Entity entity;
  Vector3D point;
};

class RobotEntity : public Entity {
public:
  RobotEntity(Vector3D position,
              double radius,
              double mass,
              Vector3D velocity,
              double radius_change_speed,
              bool touch,
              Vector3D touch_normal,
              double nitro_amount,
              int id,
              bool is_teammate,
              Action action,
              double arena_e) :
                Entity(position, radius, mass, velocity, radius_change_speed),
                touch(touch),
                touch_normal(touch_normal),
                id(id),
                is_teammate(is_teammate),
                action(action) {
    this->arena_e = arena_e;
    this->is_robot = true;
    this->is_ball = false;
  }

  static RobotEntity from_robot(const Robot& robot, const Action& action, const Rules& rules) {
    return RobotEntity(
        Vector3D(robot.x, robot.y, robot.z),
        robot.radius,
        rules.ROBOT_MASS,
        Vector3D(robot.velocity_x, robot.velocity_y, robot.velocity_z),
        action.jump_speed,
        robot.touch,
        Vector3D(robot.touch_normal_x, robot.touch_normal_y, robot.touch_normal_z),
        robot.nitro_amount,
        robot.id,
        robot.is_teammate,
        action,
        rules.ROBOT_ARENA_E);
  }

  bool touch;
  Vector3D touch_normal;
  double nitro_amount;
  int id;
  bool is_teammate;
  Action action;
};

class BallEntity : public Entity {
public:
  BallEntity(
      Vector3D position,
      double radius,
      double mass,
      Vector3D velocity,
      double radius_change_speed,
      double arena_e) : Entity(position, radius, mass, velocity, radius_change_speed) {
    this->arena_e = arena_e;
    this->is_robot = false;
    this->is_ball = true;
  }

  static BallEntity from_ball(const Ball& ball, const Rules& rules) {
    return BallEntity(
        Vector3D(ball.x, ball.y, ball.z),
        ball.radius,
        rules.BALL_MASS,
        Vector3D(ball.velocity_x, ball.velocity_y, ball.velocity_z),
        0,
        rules.BALL_ARENA_E);
  }
};

class NitroEntity : public Entity {
public:
  NitroEntity(
      Vector3D position,
      double radius,
      double mass,
      Vector3D velocity,
      double radius_change_speed,
      int nitro_amount,
      int respawn_ticks) :
        Entity(position, radius, mass, velocity, radius_change_speed),
        nitro_amount(nitro_amount),
        respawn_ticks(respawn_ticks){}

  static NitroEntity from_nitro_pack(const NitroPack& nitro, const Rules& rules) {
    return NitroEntity(
        Vector3D(nitro.x, nitro.y, nitro.z),
        nitro.radius,
        0,
        Vector3D(0, 0, 0),
        0,
        nitro.alive ? rules.NITRO_PACK_AMOUNT : 0,
        nitro.respawn_ticks);
  }

  int nitro_amount;
  int respawn_ticks;
};

double collide_entities__random(double x, double y);

bool collide_entities(const Rules& rules, Entity& a, Entity& b);

pair<bool, Vector3D> collide_with_arena(const Rules& rules, const Entity& e);

void move_entity(const Rules& rules, Entity& e, const double delta_time);

void update(const Rules& rules, const double delta_time, vector<RobotEntity>& robots, BallEntity& ball,
    vector<NitroEntity>& nitros, GameState& game_state,
    bool register_collisions,
    vector<CollideArena>collision_arena,
    vector<CollideEntities> collision_entities);

void tick(const Rules& rules, vector<RobotEntity>& robots, BallEntity& ball,
    vector<NitroEntity>& nitros, GameState& game_state,double delta_time,bool microticks,
    bool register_collisions,
    vector<CollideArena>collision_arena,
    vector<CollideEntities> collision_entities);

class State {
public:
  State(const int me, const Rules& rules, const Game& game, const map<int, HistoryItem>& history) :
  rules(rules),
  ball( BallEntity::from_ball(game.ball, rules)) {
    game_state = GameState::from_game(game);
    robots.reserve(game.robots.size());
    for (auto& robot : game.robots) {
      if (history.count(robot.id)) {
        robots.push_back(RobotEntity::from_robot(robot, history.at(robot.id).action, rules));
      } else {
        Action action;
        action.target_velocity_x = robot.velocity_x;
        action.target_velocity_y = robot.velocity_y;
        action.target_velocity_z = robot.velocity_z;
        action.jump_speed = rules.ROBOT_MAX_JUMP_SPEED * (robot.radius - rules.ROBOT_MIN_RADIUS) /
                            (rules.ROBOT_MAX_RADIUS - rules.ROBOT_MIN_RADIUS);
        action.use_nitro = false;
        robots.push_back(RobotEntity::from_robot(robot, action, rules));
      }
    }
    sort(robots.begin(), robots.end(), [](const RobotEntity& a, const RobotEntity& b) { return a.id < b.id; });
    nitros.reserve(game.nitro_packs.size());
    for (auto& nitro : game.nitro_packs) {
      nitros.push_back(NitroEntity::from_nitro_pack(nitro, rules));
    }
  }

  void simulate(double dt = 0.0, bool microticks = true) {
    // steady_clock::time_point start = steady_clock::now();
    if (dt == 0.0) {
      dt = 1.0 / rules.TICKS_PER_SECOND;
    }
    tick(rules, robots, ball, nitros, game_state, dt, microticks, false, {}, {});
    // auto duration = steady_clock::now() - start;
    // double ms = duration_cast<nanoseconds>(duration).count() * 0.000001;
    // ms_sum += ms;
    // ms_count++;
    // cout << "Tick: " << ms << " ms, avg: " << (ms_sum / ms_count) << " ms" << endl;
  }

  double my_score() const {
    bool win = false;
    bool lose = false;
    double score = 0.0;
    vector<RobotEntity> robots = this->robots;
    BallEntity ball = this->ball;
    vector<NitroEntity> nitros = this->nitros;
    GameState game_state2;
    for (int  i = 1; i < 2; ++i) {
      double dt = 0.2;
      //update(rules, dt, robots, ball, nitros, game_state2);
      if (game_state2.my_score || game_state2.enemy_score) {
        if (game_state2.my_score > 0) {
          win = true;
          score += 10000000.0 / i;
        } else {
          lose = true;
          score -= 10000000.0 / i;
        }
        break;
      }
      //debug_draw->push_back({ball.position.x, ball.position.y, ball.position.z, 0.1, 1, 0, 0, 1.0});
    }
    double ball_my_min_distance = 1000000;
    double ball_my_sum_distance = 0;
    double ball_enemy_min_distance = 1000000;
    double ball_enemy_sum_distance = 0;
    double my_z_sum = 0;
    int my_touch = 0;
    int enemy_touch = 0;
    for (const RobotEntity& e : robots) {
      bool ball_between_player_and_gates = e.is_teammate ?
              e.position.z < ball.position.z - ball.radius - e.radius:
              e.position.z > ball.position.z + ball.radius + e.radius;
      double dist = e.position.plane().distance_to(ball.position.plane());
      if (!ball_between_player_and_gates) {
        dist *= 2;
      }
      if (!e.touch) {
        dist += 5;
      }
      if (e.is_teammate) {
        if (ball_my_min_distance > dist) ball_my_min_distance = dist;
        if (e.touch) my_touch++;
        ball_my_sum_distance += dist;
        my_z_sum += e.position.z;
      } else {
        if (ball_enemy_min_distance > dist) ball_enemy_min_distance = dist;
        if (e.touch) enemy_touch++;
        ball_enemy_sum_distance += dist;
      }
    }
    score -= ball_my_min_distance * ball_my_min_distance * 0.01;
    score += ball_enemy_min_distance * ball_enemy_min_distance * 0.005;
    score -= ball_my_sum_distance * ball_my_sum_distance * 0.001;
    score += ball_enemy_sum_distance * ball_enemy_sum_distance * 0.0005;
    score += ball.position.z * 1;
    score += ball.velocity.z * 0.1;
    score += my_touch * 1;
    score -= enemy_touch * 1;
//    cout << "SCORE: " <<
//      (ball_my_min_distance * ball_my_min_distance * 0.01) << "," <<
//      (ball_enemy_min_distance * ball_enemy_min_distance * 0.01) << "," <<
//      (ball_my_sum_distance * ball_my_sum_distance * 0.001) << "," <<
//      (ball_enemy_sum_distance * ball_enemy_sum_distance * 0.001) << "," <<
//      (ball.position.z * 100) << "," <<
//      (ball.velocity.z * 1) << "," <<
//      (my_touch * 1) << "," <<
//      (enemy_touch * 1) << endl;
    return score;
  }

    double my_score2(bool debug = false) const {
      bool win = false;
      bool lose = false;
      double score = 0.0;
      vector<RobotEntity> robots = this->robots;
      BallEntity ball = this->ball;
      vector<NitroEntity> nitros = this->nitros;
      GameState game_state2;
      for (int  i = 1; i < 2; ++i) {
        double dt = 1.0 / 60;
        //update_only_ball(rules, dt, robots, ball, nitros, game_state2);
        if (game_state2.my_score || game_state2.enemy_score) {
          if (game_state2.my_score > 0) {
            win = true;
            score += 10000000.0; // / i;
          } else {
            lose = true;
            score -= 10000000.0; // / i;
          }
          break;
        }
        debug_draw->push_back({ball.position.x, ball.position.y, ball.position.z, 0.1, 1, 0, 0, 1.0});
      }
      double ball_my_min_distance = 1000000;
      double ball_my_sum_distance = 0;
      double ball_enemy_min_distance = 1000000;
      double ball_enemy_sum_distance = 0;
      int my_touch = 0;
      int enemy_touch = 0;
      for (const RobotEntity& e : robots) {
        bool ball_between_player_and_gates = e.is_teammate ?
                                             e.position.z < ball.position.z - ball.radius - e.radius:
                                             e.position.z > ball.position.z + ball.radius + e.radius;
        double dist = e.position.plane().distance_to(ball.position.plane());
        if (!ball_between_player_and_gates) {
          dist *= 4;
        }
        if (!e.touch) {
          dist += 5;
        }
        if (e.is_teammate) {
          if (ball_my_min_distance > dist) ball_my_min_distance = dist;
          if (e.touch) my_touch++;
          ball_my_sum_distance += dist;
        } else {
          if (ball_enemy_min_distance > dist) ball_enemy_min_distance = dist;
          if (e.touch) enemy_touch++;
          ball_enemy_sum_distance += dist;
        }
      }
      double my_gate_dist =  rules.arena.depth * 0.5 + ball.radius + ball.position.z;
      double enemy_gate_dist = rules.arena.depth - my_gate_dist;

      score -= ball_my_min_distance * ball_my_min_distance * 0.001;
      score += ball_enemy_min_distance * ball_enemy_min_distance * 0.0001;
      score -= ball_my_sum_distance * ball_my_sum_distance * 0.00005;
      score += ball_enemy_sum_distance * ball_enemy_sum_distance * 0.00005;
      score += -rules.arena.depth * 10.0 /(my_gate_dist*my_gate_dist) + rules.arena.depth * 10.0/(enemy_gate_dist*enemy_gate_dist);//ball.position.z * 0.1;
      score += ball.velocity.z * 0.01;
      score -= (2 - my_touch) * 10;
      score -= enemy_touch * 1;
      if (debug) {
        cout << "SCORE: " <<
             (ball_my_min_distance * ball_my_min_distance * 0.001) << "," <<
             (ball_enemy_min_distance * ball_enemy_min_distance * 0.0001) << "," <<
             (ball_my_sum_distance * ball_my_sum_distance * 0.00005) << "," <<
             (ball_enemy_sum_distance * ball_enemy_sum_distance * 0.00005) << "," <<
             (-rules.arena.depth * 10.0 /(my_gate_dist*my_gate_dist) + rules.arena.depth * 10.0/(enemy_gate_dist*enemy_gate_dist)) << "," <<
             (ball.velocity.z * 0.01) << "," <<
             ((2 - my_touch) * 10) << "," <<
             (enemy_touch * 1)
             << endl;
      }
      return score;
    }

  bool is_terminal() const {
    return abs(ball.position.z) > rules.arena.depth * 0.5 + ball.radius;
  }

  State (const State &other) : rules(other.rules),
                               game_state(other.game_state),
                               robots(other.robots),
                               ball(other.ball),
                               nitros(other.nitros) {}

  State& operator=(const State& other) {
    //this->rules = other.rules;
    this->game_state = other.game_state;
    this->robots = other.robots;
    this->ball = other.ball;
    this->nitros = other.nitros;
    return *this;
  }

  static double ms_sum;
  static int ms_count;

  const Rules& rules;
  GameState game_state;
  vector<RobotEntity> robots;
  BallEntity ball;
  vector<NitroEntity> nitros;
};

struct StateEntry {
    State state;
    int id;
    bool is_teammate;
    Action action;
    StateEntry* prev;

    double my_score() const {
      return state.my_score();
    }
};

class McState {
public:
    McState(StateEntry e, int initial_id) : state(e.state), id(e.id), is_teammate(e.is_teammate),
    initial_id(initial_id), initial_game_tick(e.state.game_state.current_tick) {

    }

    void apply_action(const Action& action) {
      state.robots[id - 1].action = action;
      int new_id = (id % state.robots.size()) + 1;
      state.simulate(1.0/60/state.robots.size(), false);
      int depth = state.game_state.current_tick - initial_game_tick;
      if (is_teammate && (depth > 2 || rand() % 100 > 90)) {
        RobotEntity& e = state.robots[id - 1];
        stringstream ss;
        float r = depth % 3 == 0 ? 1.0 : 0.0;
        float g = depth % 3 == 1 ? 1.0 : 0.0;
        float b = depth % 3 == 2 ? 1.0 : 0.0;
        debug_draw->push_back({e.position.x, e.position.y, e.position.z, 1.0, r, g, b, 0.1});
      }
      //debug_draw->push_back({state.ball.position.x, state.ball.position.y, state.ball.position.z, 0.1, 0, 0, 1, 0.5});
      id = new_id;
      for (const RobotEntity& e : state.robots) {
        if (e.id == id) {
          is_teammate = e.is_teammate;
          break;
        }
      }
    }

    bool is_terminal() const {
      return state.is_terminal();
    }

    bool get_random_action(Action& action) const {
      int x = rand() % 200 - 100;
      int z = rand() % 200 - 100;
      action.target_velocity_x = x;
      action.target_velocity_z = z;
      if (!is_teammate) {
        action.jump_speed = (rand() % 100 > 90 ? 15 : 0);
      }
      return true;
    }

    const std::vector<float> evaluate() const {
      float score = (float) state.my_score();
      return {0, score, -score};
    }

    int agent_id() const {
      return (is_teammate ? 1 : 2);
    }

    void get_actions(std::vector<Action>& actions) const  {
      actions.clear();
      for (int x = -100; x <= 100; x += 25) {
        for (int z = -100; z <= 100; z += 25) {
          Action a;
          a.target_velocity_x = x;
          a.target_velocity_z = z;
          actions.push_back(a);
          if (is_teammate) {
            // pass
          } else {
            a.jump_speed = 15;
            actions.push_back(a);
          }
        }
      }
    }

    State state;
    int id;
    bool is_teammate;
    int initial_id;
    int initial_game_tick;
};

/**
 * MTD(f)
 * SSS*
 * NegaScout
 */
class Engine {
public:
  Engine(const Robot& me, const Rules& rules, const Game& game, const map<int, HistoryItem>& history) :
  current{State(me.id, rules, game, history), me.id, me.is_teammate, Action(), NULL} {}

  Action find_best() {
    return monte_carlo();
  }

private:
    Action monte_carlo() {
      McState state(current, current.id);
      UCT<McState, Action> uct;

      uct.uct_k = UCT_K;
      uct.max_millis = 20000;
      uct.max_iterations = UTC_MAX_ITERATIONS;
      uct.simulation_depth = UTC_SIMULATION_DEPTH;

      return uct.run(state);
    }

    StateEntry simulated_annealing(StateEntry instance, unsigned steps) {
      StateEntry current(instance);
      StateEntry best = current;

      double temperature;

      for (int k = 0; k <= steps; k++) {
        temperature = 1.0 - ((double) k / steps);
        double r = (double) std::rand() / RAND_MAX;

        StateEntry next(instance); //(instance, current);

        if (current.is_teammate ? next.my_score() < current.my_score() : next.my_score() > current.my_score()) {
          current = next;
        } else if (temperature > r) {
          current = next;
        }

        if (current.my_score() > best.my_score()) {
          best = current;
        }
      }

      return best;
    }

  StateEntry current;
};

#endif //MYSTRATEGY_ENGINE_H
