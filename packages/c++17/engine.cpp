//
// Created by pav on 29.12.18.
//

#include "engine.h"
#include "math.h"
#include "predefined.h"
#include <optional>
#include <random>
#include <vector>
#include <algorithm>

double collide_entities__random(double x, double y) {
  return min(x, y) + abs(x - y) * 0.5;
  double r = (double) std::rand() / RAND_MAX;
  return min(x, y) + abs(x - y) * r;
}

bool collide_entities(const Rules& rules, Entity& a, Entity& b) {
  Vector3D delta_position = b.position.sub(a.position);
  double distance = delta_position.len();
  double penetration = a.radius + b.radius - distance;
  if (penetration > 0) {
    double k_a = (1.0 / a.mass) / ((1.0 / a.mass) + (1.0 / b.mass));
    double k_b = (1.0 / b.mass) / ((1.0 / a.mass) + (1.0 / b.mass));
    Vector3D normal = delta_position.normalize();
    a.position = a.position.sub(normal.mul(penetration * k_a));
    b.position = b.position.add(normal.mul(penetration * k_b));
    double delta_velocity = normal.dot(b.velocity.sub(a.velocity)) - b.radius_change_speed - a.radius_change_speed;
    if (delta_velocity < 0) {
      Vector3D impulse = normal.mul((1.0 + collide_entities__random(rules.MIN_HIT_E, rules.MAX_HIT_E)) * delta_velocity);
      a.velocity = a.velocity.add(impulse.mul(k_a));
      b.velocity = b.velocity.sub(impulse.mul(k_b));
    }
    return true;
  }
  return false;
}

pair<bool, Vector3D> collide_with_arena(const Rules& rules, Entity& e) {
  Dan dan = dan_to_arena(rules.arena, e.position);
  double penetration = e.radius - dan.distance;
  if (penetration > 0) {
    e.position = e.position.add(dan.normal.mul(penetration));
    double velocity = e.velocity.dot(dan.normal) - e.radius_change_speed;
    if (velocity < 0) {
      e.velocity = e.velocity.sub(dan.normal.mul((1 + e.arena_e) * velocity));
      return make_pair(true, dan.normal);
    }
  }
  return make_pair(false, dan.normal);
}

void move_entity(const Rules& rules, Entity& e, const double delta_time) {
  e.velocity = e.velocity.clamp(rules.MAX_ENTITY_SPEED);
  e.position = e.position.add(e.velocity.mul(delta_time));
  e.position.y -= rules.GRAVITY * delta_time * delta_time / 2;
  e.velocity.y -= rules.GRAVITY * delta_time;
}

void update(const Rules& rules, const double delta_time, vector<RobotEntity>& robots, BallEntity& ball,
            vector<NitroEntity>& nitros, GameState& game_state,
            bool register_collisions,
            vector<CollideArena>& collision_arena,
            vector<CollideEntities>& collision_entities) {
  // std::shuffle(robots.begin(), robots.end(), g2);
  for (RobotEntity& robot : robots) {
    Vector3D target_velocity = Vector3D(robot.action.target_velocity_x, robot.action.target_velocity_y,
                                        robot.action.target_velocity_z);
    if (robot.touch) {
      target_velocity = target_velocity.clamp(rules.ROBOT_MAX_GROUND_SPEED);
      target_velocity = target_velocity.sub(robot.touch_normal.mul(robot.touch_normal.dot(target_velocity)));
      Vector3D target_velocity_change = target_velocity.sub(robot.velocity);
      if (target_velocity_change.len() > 0) {
        double acceleration = rules.ROBOT_ACCELERATION * max(0.0, robot.touch_normal.y);
        robot.velocity = robot.velocity.add(
            (target_velocity_change.normalize().mul(acceleration * delta_time)).clamp(target_velocity_change.len()));
      }
    }
    if (robot.action.use_nitro) {
      Vector3D target_velocity_change = (target_velocity.sub(robot.velocity)).clamp(
          robot.nitro_amount * rules.NITRO_POINT_VELOCITY_CHANGE);
      if (target_velocity_change.len() > 0) {
        Vector3D acceleration = target_velocity_change.normalize().mul(rules.ROBOT_NITRO_ACCELERATION);
        Vector3D velocity_change = (acceleration.mul(delta_time)).clamp(target_velocity_change.len());
        robot.velocity = robot.velocity.add(velocity_change);
        robot.nitro_amount -= velocity_change.len() / rules.NITRO_POINT_VELOCITY_CHANGE;
      }
    }
    move_entity(rules, robot, delta_time);
    robot.radius = rules.ROBOT_MIN_RADIUS +
                   (rules.ROBOT_MAX_RADIUS - rules.ROBOT_MIN_RADIUS) * robot.action.jump_speed /
                   rules.ROBOT_MAX_JUMP_SPEED;
    robot.radius_change_speed = robot.action.jump_speed;
  }

  move_entity(rules, ball, delta_time);

  for (int i = 0; i < robots.size(); ++i) {
    for (int j = 0; j < i; ++j) {
      if (register_collisions) {
        bool is_collide = collide_entities(rules, robots[i], robots[j]);
        if (is_collide) {
          collision_entities.push_back({robots[i], robots[j]});
        }
      } else {
        collide_entities(rules, robots[i], robots[j]);
      }
    }
  }

  for (RobotEntity& robot : robots) {
    if (register_collisions) {
      bool is_collide = collide_entities(rules, robot, ball);
      if (is_collide) {
        collision_entities.push_back({robot, ball});
      }
    } else {
      collide_entities(rules, robot, ball);
    }
    pair<bool, Vector3D> collision_normal = collide_with_arena(rules, robot);
    if (collision_normal.first) {
      robot.touch = true;
      robot.touch_normal = collision_normal.second;
      if (register_collisions) {
        collision_arena.push_back({robot, collision_normal.second});
      }
    } else {
      robot.touch = false;
    }
  }
  collide_with_arena(rules, ball);
  if (abs(ball.position.z) > rules.arena.depth / 2 + ball.radius) {
    if (ball.position.z > 0) // TODO: check it
        game_state.my_score += 1;
    else
        game_state.enemy_score += 1;
  }

  for (RobotEntity& robot : robots) {
    if (robot.nitro_amount != rules.MAX_NITRO_AMOUNT) {
      for (auto &nitro : nitros) {
        if (nitro.nitro_amount > 0) {
          if ((robot.position.sub(nitro.position)).len() <= robot.radius + nitro.radius) {
            robot.nitro_amount = rules.MAX_NITRO_AMOUNT;
            nitro.nitro_amount = 0;
            nitro.respawn_ticks = rules.NITRO_PACK_RESPAWN_TICKS;
          }
        }
      }
    }
  }
}

void tick(const Rules& rules, vector<RobotEntity>& robots, BallEntity& ball,
          vector<NitroEntity>& nitros, GameState& game_state, double delta_time, bool microticks,
          bool register_collisions,
          vector<CollideArena>& collision_arena,
          vector<CollideEntities>& collision_entities) {
  if (microticks) {
    for (int i = 0; i < rules.MICROTICKS_PER_TICK; ++i) {
      update(rules, delta_time / rules.MICROTICKS_PER_TICK, robots, ball, nitros, game_state, register_collisions, collision_arena, collision_entities);
    }
  } else {
    update(rules, delta_time, robots, ball, nitros, game_state, register_collisions, collision_arena, collision_entities);
  }
  for (auto& nitro : nitros) {
    if (nitro.nitro_amount == 0) {
      nitro.respawn_ticks -= 1;
      if (nitro.respawn_ticks == 0) {
        nitro.nitro_amount = rules.NITRO_PACK_AMOUNT;
      }
    }
  }
  game_state.current_tick++;
}

double State::ms_sum = 0.0;
int State::ms_count = 0;

Action Engine::defend() {
  const RobotEntity& me = this->me();

  // Стратегия защитника (или атакующего, не нашедшего хорошего момента для удара):
  // Будем стоять посередине наших ворот
  Vector3D target_pos(0.0, 0, -(current.state.rules.arena.depth / 2.0) + current.state.rules.arena.bottom_radius);
  // Причем, если мяч движется в сторону наших ворот
  if (current.state.ball.velocity.z < DBL_EPSILON) {
    // Найдем время и место, в котором мяч пересечет линию ворот
    State state = current.state;
    for (int i = 0; i < 100; ++i) {
      state.simulate(0, false);
      if (state.game_state.enemy_score > current.state.game_state.enemy_score) {
        target_pos = state.ball.position;
        stringstream ss;
        ss << "GOAL: " << target_pos << "\\n";
        debug_draw->push_back({target_pos.x, target_pos.y, target_pos.z, 2, 1, 0, 0, 0.9});
        *debug_text += ss.str();
        break;
      }
    }
  }

  // Установка нужных полей для желаемого действия
  Vector3D target_velocity = target_pos.plane().sub(me.position.plane()).clamp(current.state.rules.ROBOT_MAX_GROUND_SPEED);
  bool jump =
      current.state.ball.position.distance_to(me.position) < (current.state.rules.BALL_RADIUS + current.state.rules.ROBOT_MAX_RADIUS)
      && me.position.y < current.state.ball.position.y;

  Action action;
  action.target_velocity_x = target_velocity.x;
  action.target_velocity_z = target_velocity.z;
  action.target_velocity_y = 0.0;
  action.jump_speed = jump ? current.state.rules.ROBOT_MAX_JUMP_SPEED : 0.0;
  action.use_nitro = false;

  return action;
}