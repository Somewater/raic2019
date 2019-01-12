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
#include <math.h>
#include "predefined.h"

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
  bool is_defender = false;
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
    vector<CollideArena>& collision_arena,
    vector<CollideEntities>& collision_entities);

void tick(const Rules& rules, vector<RobotEntity>& robots, BallEntity& ball,
    vector<NitroEntity>& nitros, GameState& game_state,double delta_time,bool microticks,
    bool register_collisions,
    vector<CollideArena>& collision_arena,
    vector<CollideEntities>& collision_entities);

class State {
public:
  State(const int me, const Rules& rules, const Game& game, const map<int, HistoryItem>& history) :
  me_id(me),
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
    bool register_collisions = false;
    if (register_collisions) {
      collision_arena.clear();
      collision_entities.clear();
    }
    tick(rules, robots, ball, nitros, game_state, dt, microticks, register_collisions, collision_arena, collision_entities);
    // auto duration = steady_clock::now() - start;
    // double ms = duration_cast<nanoseconds>(duration).count() * 0.000001;
    // ms_sum += ms;
    // ms_count++;
    // cout << "Tick: " << ms << " ms, avg: " << (ms_sum / ms_count) << " ms" << endl;
  }

  // -1.0 .. 1.0
  Evaluation my_score() const {
    if (is_goal()) {
      if (ball.position.z > 0) {
        return {10000000};
      } else {
        return {-10000000};
      }
    }
    double score = 0.0;
    double ball_my_min_distance = 1000000;
    double ball_my_sum_distance = 0;
    double ball_enemy_min_distance = 1000000;
    double ball_enemy_sum_distance = 0;
    double defender_z_pos = -1000000;
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
        if (e.is_defender) defender_z_pos = e.position.z;
        ball_my_sum_distance += dist;
        my_z_sum += e.position.z;
      } else {
        if (ball_enemy_min_distance > dist) ball_enemy_min_distance = dist;
        if (e.touch) enemy_touch++;
        ball_enemy_sum_distance += dist;
      }
    }
    return Evaluation(score,
            ball_my_min_distance,
            ball_enemy_min_distance,
            ball_my_sum_distance,
            ball_enemy_sum_distance,
            ball.position.z,
            ball.position.x,
            ball.velocity.z,
            robots.size() / 2 - my_touch,
            robots.size() / 2 - enemy_touch,
            defender_z_pos);
  }

  bool is_goal() const {
    return abs(ball.position.z) > rules.arena.depth * 0.5 + ball.radius;
  }

  State (const State &other) : me_id(other.me_id),
                               rules(other.rules),
                               game_state(other.game_state),
                               robots(other.robots),
                               ball(other.ball),
                               nitros(other.nitros) {}

  State& operator=(const State& other) = delete;

  static double ms_sum;
  static int ms_count;

  const int me_id;
  const Rules& rules;
  GameState game_state;
  vector<RobotEntity> robots;
  BallEntity ball;
  vector<NitroEntity> nitros;
  vector<CollideArena> collision_arena;
  vector<CollideEntities> collision_entities;
};

struct StateEntry {
    State state;
    int id;
    bool is_teammate;
    Action action;
    StateEntry* prev;
};

class McState {
public:
    McState(State state, int id) :
      state(state),
      id(id),
      initial_id(id),
      ticks(0) {
      actualize_teammate_flag();
    }

    McState(const McState &ms) :
      state(ms.state),
      id(ms.id),
      initial_id(ms.id),
      ticks(ms.ticks) {
      actualize_teammate_flag();
    }


    void apply_action(const McAction& action) {
      for (RobotEntity& r : state.robots) {
        if (r.id == id) {
          r.action = action.action;
          break;
        }
      }
      // 0.004166666
      double dt = (action.playout ? SIMULATION_PLAYOUT_DT : SIMULATION_DT); // * (sqrt(1 + depth));
      state.simulate(dt, false);
#ifdef MY_DEBUG
      if (is_teammate && id == state.me_id) {
        RobotEntity* e;
        for (RobotEntity& r : state.robots) {
          if (r.id == id) {
            e = &r;
          }
        }
        stringstream ss;
        float b = depth() % 3 == 0 ? 1.0 : 0.0;
        float g = depth() % 3 == 1 ? 1.0 : 0.0;
        float r = depth() % 3 == 2 ? 1.0 : 0.0;
        //cout << "depth=" << depth << endl;
        float a = 0.3;
        if (action.playout) {
          r = 1;
          g = 1;
          b = 0;
          a = 0.1;
        }
        if (!action.playout) {
          debug_draw->push_back({e->position.x, e->position.y, e->position.z, 1.0, r, g, b, a});
          debug_draw->push_back({state.ball.position.x, state.ball.position.y, state.ball.position.z, 2.0, r, g, b, 0.1});
        }
      }
#endif
      id = (id % state.robots.size()) + 1;
      ticks++;
      actualize_teammate_flag();
    }

    void actualize_teammate_flag() {
      for (const RobotEntity& e : state.robots) {
        if (e.id == id) {
          is_teammate = e.is_teammate;
          break;
        }
      }
    }

    // in real ticks
    int depth() {
      return ticks / state.robots.size();
    }

    bool is_terminal() const {
      return state.is_goal(); // || (depth > 2 && state.is_entity_collision());
    }

    bool get_random_action(McAction& action) const {
      vector<McAction> actions;
      actions.reserve(MAX_ACTIONS);
      get_actions(actions);
      if (actions.empty()) {
        return false;
      } else {
        action = actions[rand() % actions.size()];
        action.playout = true;
        return true;
      }
      for (const RobotEntity& robot : state.robots) {
        if (robot.id == id) {

          if (robot.touch) {
            bool jump =
                state.ball.position.distance_to(robot.position) <
                (state.rules.BALL_RADIUS + state.rules.ROBOT_MAX_RADIUS)
                && robot.position.y < state.ball.position.y;
            int x = ((rand() % 3 == 0) - 1) * 100;
            int z = ((rand() % 3 == 0) - 1) * 100;
            action.action.target_velocity_x = x;
            action.action.target_velocity_z = z;
            action.action.jump_speed = (jump && rand() % 2 == 0 ? 15 : 0);
          } else {
            action.action.target_velocity_x = 0.0;
            action.action.target_velocity_z = 0.0;
            action.action.target_velocity_y = -state.rules.MAX_ENTITY_SPEED;
            action.action.jump_speed        = 0.0;
            action.action.use_nitro         = true;
          }
          action.playout = true;
          return true;
        }
      }
      throw runtime_error("Robot not found");
    }

    const std::vector<Evaluation> evaluate() const {
      Evaluation evaluation = state.my_score();
      return {evaluation, evaluation, evaluation.negative()};
    }

    int agent_id() const {
      return (is_teammate ? 1 : 2);
    }

    void get_actions(std::vector<McAction>& actions) const  {
      actions.clear();
      actions.reserve(MAX_ACTIONS);
      for (const RobotEntity& robot : state.robots) {
        if (robot.id == id) {
          Action prev_action;
          if (robot.touch) {
            prev_action = robot.action;
          } else {
            prev_action.target_velocity_x = 0.0;
            prev_action.target_velocity_z = 0.0;
            prev_action.target_velocity_y = -state.rules.MAX_ENTITY_SPEED;
            prev_action.jump_speed        = 0.0;
            prev_action.use_nitro         = true;
          }
          actions.push_back({prev_action});

          if (robot.touch) {
            for (int x = -100; x <= 100; x += 100) {
              for (int z = -100; z <= 100; z += 100) {
                Action a;
                a.target_velocity_x = x;
                a.target_velocity_z = z;
                if (!equal(a, prev_action)) actions.push_back({a});
                bool jump =
                    state.ball.position.distance_to(robot.position) <
                      (state.rules.BALL_RADIUS + state.rules.ROBOT_MAX_RADIUS)
                      && robot.position.y < state.ball.position.y;
                if (jump) {
                  a.jump_speed = 15;
                  if (!equal(a, prev_action)) actions.push_back({a});
                }
              }
            }
          }
          return;
        }
      }
      throw runtime_error("Robot not found");
    }

    bool its_me() const {
      return initial_id == state.me_id;
    }

    State state;
    int id;
    int initial_id = -1;
    bool is_teammate;
    int ticks; // not real, simulation tree "ticks", use depth() instead
};

/**
 * MTD(f)
 * SSS*
 * NegaScout
 */
class Engine {
public:
  Engine(const Robot& me, const Rules& rules, const Game& game, const map<int, HistoryItem>& history) :
  current{State(me.id, rules, game, history), me.id} {}

  Action find_best() {
    return monte_carlo();
  }

  bool apply_defender() {
    bool result = false;
    RobotEntity& me = this->me();
    for (RobotEntity& e : current.state.robots) {
      if (e.is_teammate && e.id != me.id && (
            e.position.z <= me.position.z - 1 || (e.id < me.id && abs(e.position.z - me.position.z) < 1)
          )) {
        return false;
      }
    }
    me.is_defender = true;
    return true;
  }

  Action defend();

  RobotEntity& me() {
    for (RobotEntity& e : current.state.robots)
      if (e.id == current.id)
        return e;
    throw runtime_error("Me not found");
  }

private:
    Action monte_carlo() {
      UCT<McState, McAction> uct;

      uct.uct_k = UCT_K;
      uct.max_millis = 20000;
      uct.max_iterations = UTC_MAX_ITERATIONS;
      uct.simulation_depth = UTC_SIMULATION_DEPTH;

      return uct.run(current).action;
    }

  McState current;
};

#endif //MYSTRATEGY_ENGINE_H
