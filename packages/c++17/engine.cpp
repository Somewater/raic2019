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
}

void collide_entities(const Rules& rules, Entity& a, Entity& b) {
  Vector3D delta_position = b.position.sub(a.position);
  double distance = delta_position.len();
  double penetration = a.radius + b.radius - distance;
  if (penetration > 0) {
    double k_a = (1.0 / a.mass) / ((1.0 / a.mass) + (1.0 / b.mass));
    double k_b = (1.0 / b.mass) / ((1.0 / a.mass) + (1.0 / b.mass));
    Vector3D normal = delta_position.normalize();
    a.position = a.position.sub(normal.mul(penetration * k_a));
    b.position = b.position.sub(normal.mul(penetration * k_b));
    double delta_velocity = normal.dot(b.velocity.sub(a.velocity)) + b.radius_change_speed - a.radius_change_speed;
    if (delta_velocity < 0) {
      Vector3D impulse = normal.mul((1.0 + collide_entities__random(rules.MIN_HIT_E, rules.MAX_HIT_E)) * delta_velocity);
      a.velocity = a.velocity.add(impulse.mul(k_a));
      b.velocity = a.velocity.sub(impulse.mul(k_b));
    }
  }
}

optional<Vector3D> collide_with_arena(const Rules& rules, Entity& e) {
  Dan dan = dan_to_arena(rules.arena, e.position);
  double penetration = e.radius - dan.distance;
  if (penetration > 0) {
    e.position = e.position.add(dan.normal.mul(penetration));
    double velocity = e.velocity.dot(dan.normal) - e.radius_change_speed;
    if (velocity < 0) {
      e.velocity = e.velocity.sub(dan.normal.mul((1 + e.arena_e) * velocity));
      return dan.normal;
    }
  }
  return {};
}

void move_entity(const Rules& rules, Entity& e, const double delta_time) {
  e.velocity = clamp_vector(e.velocity, rules.MAX_ENTITY_SPEED);
  e.position = e.position.add(e.velocity.mul(delta_time));
  e.position.y -= rules.GRAVITY * delta_time * delta_time / 2;
  e.velocity.y -= rules.GRAVITY * delta_time;
}

void update(const Rules& rules, const double delta_time, vector<RobotEntity>& robots, BallEntity& ball,
            vector<NitroEntity>& nitros, GameState& game_state) {
  // random.shuffle(robots[:]) // TODO
  for (RobotEntity& robot : robots) {
    Vector3D target_velocity = Vector3D(robot.action.target_velocity_x, robot.action.target_velocity_y,
                                        robot.action.target_velocity_z);
    if (robot.touch) {
      target_velocity = target_velocity.min(rules.ROBOT_MAX_GROUND_SPEED);
      target_velocity = target_velocity.sub(robot.touch_normal.mul(robot.touch_normal.dot(target_velocity)));
      Vector3D target_velocity_change = target_velocity.sub(robot.velocity);
      if (target_velocity_change.len() > 0) {
        double acceleration = rules.ROBOT_ACCELERATION * max(0.0, robot.touch_normal.y);
        robot.velocity = robot.velocity.add(
            (target_velocity_change.normalize().mul(acceleration * delta_time)).min(target_velocity_change.len()));
      }
    }
    if (robot.action.use_nitro) {
      Vector3D target_velocity_change = (target_velocity.sub(robot.velocity)).min(
          robot.nitro_amount * rules.NITRO_POINT_VELOCITY_CHANGE);
      if (target_velocity_change.len() > 0) {
        Vector3D acceleration = target_velocity_change.normalize().mul(rules.ROBOT_NITRO_ACCELERATION);
        Vector3D velocity_change = (acceleration.mul(delta_time)).min(target_velocity_change.len());
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
      collide_entities(rules, robots[i], robots[j]);
    }
  }

  for (RobotEntity& robot : robots) {
    collide_entities(rules, robot, ball);
    optional<Vector3D> collision_normal = collide_with_arena(rules, robot);
    if (collision_normal.has_value()) {
      robot.touch = true;
      robot.touch_normal = collision_normal.value();
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
          vector<NitroEntity>& nitros, GameState& game_state) {
  double delta_time = 1.0 / rules.TICKS_PER_SECOND;
  for (int i = 0; i < rules.MICROTICKS_PER_TICK; ++i) {
    update(rules, delta_time / rules.MICROTICKS_PER_TICK, robots, ball, nitros, game_state);
  }
  for (auto& nitro : nitros) {
    if (nitro.nitro_amount == 0) {
      nitro.respawn_ticks -= 1;
      if (nitro.respawn_ticks == 0) {
        nitro.nitro_amount = rules.NITRO_PACK_AMOUNT;
      }
    }
  }
}

double Engine::ms_sum = 0.0;
int Engine::ms_count = 0;