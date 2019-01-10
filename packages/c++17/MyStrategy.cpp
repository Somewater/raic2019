#include "MyStrategy.h"
#include "engine.h"
#include "ofxMSAmcts.h"

using namespace model;

MyStrategy::MyStrategy() { }

void show_state(State& state) {
  stringstream ss;
  for (RobotEntity& e : state.robots) {
    if (e.is_teammate) {
      debug_draw->push_back({e.position.x, e.position.y, e.position.z, 1, 0, 1, 0, 0.1});
    } else {
      debug_draw->push_back({e.position.x, e.position.y, e.position.z, 1, 1, 0, 0, 0.1});
    }
  }
  debug_draw->push_back({state.ball.position.x, state.ball.position.y, state.ball.position.z, 2, 1, 1, 1, 0.1});
}

void check_engine_correctness(const Robot& me, const Rules& rules, const Game& game, Action& action, map<int,HistoryItem>& history) {

  State state(me.id, rules, game, history);
  for (int i = 0; i < 100; i++) {
//    for (RobotEntity& r : state.robots) {
//      bool jump = (state.ball.position.distance_to(r.position) < (BALL_RADIUS + ROBOT_MAX_RADIUS) && r.position.y < state.ball.position.y);
//      if (jump) {
//        r.action.jump_speed = ROBOT_MAX_JUMP_SPEED;
//      }
//    }

    state.simulate(0, false);
    show_state(state);
  }

  stringstream ss;
  vector<Robot> robots = game.robots;
  sort(robots.begin(), robots.end(), [](Robot& r1, Robot& r2){ return r1.id < r2.id; });
  for (const Robot& r : robots) {

    ss << "R" << r.id << ": velocity=(" << r.velocity_x << "," << r.velocity_y << "," << r.velocity_z << ")" <<
     ", radius=" << r.radius << ", touch=";
    if (r.touch) {
      ss << "(" << r.touch_normal_x << "," << r.touch_normal_y << "," << r.touch_normal_z << ")";
    } else {
      ss << "None";
    }
    ss << "\\n";
  }
  ss << "BALL: velocity=(" << game.ball.velocity_x << "," << game.ball.velocity_y << "," << game.ball.velocity_z << ")\\n";
  *debug_text += ss.str();
}

#ifdef MY_DEBUG
string MyStrategy::custom_rendering() {
  stringstream ss;
  ss << "[";
  for (const DrawSphere& s : *debug_draw) {
    ss << "  {"
          "    \"Sphere\": {"
          "      \"x\": " << s.x << "," <<
          "      \"y\": " << s.y << "," <<
          "      \"z\": " << s.z << "," <<
          "      \"radius\": " << s.radius << "," <<
          "      \"r\": " << s.r << "," <<
          "      \"g\": " << s.g << "," <<
          "      \"b\": " << s.b << "," <<
          "      \"a\": " << s.a << "" <<
          "    }"
          "  },";
  }
  ss << "{\"Text\":\"" + (*debug_text) + "\"}]";
  return ss.str();
}
#endif

void MyStrategy::act(const Robot& me, const Rules& rules, const Game& game, Action& action) {
  //if (use_prev_action(me, rules, game, action)) return;
  //if (game.current_tick < 10000) return;

  debug_text->clear();
  debug_draw->clear();
  //check_engine_correctness(me, rules, game, action,history); return;
  //if (game.current_tick > 100) print_r1_r2_positions(me, rules, game, action);
  Engine engine(me, rules, game, history);
  engine.apply_defender();
  action = engine.find_best();
  history[me.id] = {game.current_tick, action};
  return;

  ball.set(game.ball.x, game.ball.z, game.ball.y);
  ball_v.set(game.ball.velocity_x, game.ball.velocity_z, game.ball.velocity_y);

  // Наша стратегия умеет играть только на земле
  // Поэтому, если мы не касаемся земли, будет использовать нитро
  // чтобы как можно быстрее попасть обратно на землю
  if ( !me.touch ) {
    action.target_velocity_x = 0.0;
    action.target_velocity_z = 0.0;
    action.target_velocity_y = -MAX_ENTITY_SPEED;
    action.jump_speed        = 0.0;
    action.use_nitro         = true;
    return;
  }

  // Если при прыжке произойдет столкновение с мячом, и мы находимся
  // с той же стороны от мяча, что и наши ворота, прыгнем, тем самым
  // ударив по мячу сильнее в сторону противника
  bool jump = (   ball.distTo(me.x, me.z, me.y) < (BALL_RADIUS + ROBOT_MAX_RADIUS)
                  && me.y < ball.y );

  // Так как роботов несколько, определим нашу роль - защитник, или нападающий
  // Нападающим будем в том случае, если есть дружественный робот,
  // находящийся ближе к нашим воротам
  bool is_attacker = false; // = (game.robots.size() == 2);
  for (const Robot &robot : game.robots) {
    if (   robot.is_teammate
           && robot.id != me.id) {
      if (robot.z < me.z) {
        is_attacker = true;
      }
    }
  }

  if (is_attacker) {
    // Стратегия нападающего:
    // Просимулирем примерное положение мяча в следующие 10 секунд, с точностью 0.1 секунда
    for (int i = 0; i < 100; ++i) {
      double t = i * 0.1;
      Point3D ball_pos = ball + ball_v*t;
      // Если мяч не вылетит за пределы арены
      // (произойдет столкновение со стеной, которое мы не рассматриваем),
      // и при этом мяч будет находится ближе к вражеским воротам, чем робот,
      if (   ball_pos.z > me.z
             && abs(ball.x) < (rules.arena.width / 2.0)
             && abs(ball.z) < (rules.arena.depth / 2.0) ) {
        // Посчитаем, с какой скоростью робот должен бежать,
        // Чтобы прийти туда же, где будет мяч, в то же самое время
        Point2D delta_pos(ball_pos.x - me.x, ball_pos.z - me.z);
        double delta_pos_dist = delta_pos.dist();
        double need_speed = delta_pos_dist / t;
        // Если эта скорость лежит в допустимом отрезке
        if (0.5 * ROBOT_MAX_GROUND_SPEED < need_speed
            && need_speed < ROBOT_MAX_GROUND_SPEED ) {
          // То это и будет наше текущее действие
          Point2D target_velocity(delta_pos.normalize(delta_pos_dist)*need_speed);
          action.target_velocity_x = target_velocity.x;
          action.target_velocity_z = target_velocity.z;
          action.target_velocity_y = 0.0;
          action.jump_speed = jump ? ROBOT_MAX_JUMP_SPEED : 0.0;
          action.use_nitro = false;
          return;
        }
      }
    }
  }
  // Стратегия защитника (или атакующего, не нашедшего хорошего момента для удара):
  // Будем стоять посередине наших ворот
  Point2D target_pos(0.0, -(rules.arena.depth / 2.0) + rules.arena.bottom_radius);
  // Причем, если мяч движется в сторону наших ворот
  if (ball_v.z < -EPS) {
    // Найдем время и место, в котором мяч пересечет линию ворот
    double t = (target_pos.z - ball.z) / ball_v.z;
    double x = ball.x + ball_v.x * t;
    // Если это место - внутри ворот
    if (abs(x) < (rules.arena.goal_width / 2.0)) {
      // То пойдем защищать его
      target_pos.x = x;
    }
  }
  // Установка нужных полей для желаемого действия
  Point2D target_velocity(target_pos.x - me.x, target_pos.z - me.z);
  target_velocity *= ROBOT_MAX_GROUND_SPEED;

  action.target_velocity_x = target_velocity.x;
  action.target_velocity_z = target_velocity.z;
  action.target_velocity_y = 0.0;
  action.jump_speed = jump ? ROBOT_MAX_JUMP_SPEED : 0.0;
  action.use_nitro = false;
}
