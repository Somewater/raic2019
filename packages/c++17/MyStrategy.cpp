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

double ms_sum = 0.0;
int ms_count = 0;

void engine_tick(const Robot& me, const Rules& rules, const Game& game, Action& action, map<int,HistoryItem>& history) {
  Engine engine(me, rules, game, history);
  //engine.apply_defender();
  action = engine.find_best();
  history[me.id] = {game.current_tick, action};
}

void MyStrategy::act(const Robot& me, const Rules& rules, const Game& game, Action& action) {
  steady_clock::time_point start = steady_clock::now();
  //if (use_prev_action(me, rules, game, action)) return;
  //if (game.current_tick < 10000) return;

  debug_text->clear();
  debug_draw->clear();
  //check_engine_correctness(me, rules, game, action,history); return;
  //if (game.current_tick > 100) print_r1_r2_positions(me, rules, game, action);
  engine_tick(me, rules, game, action, history);

  double ms = duration_cast<nanoseconds>(steady_clock::now() - start).count() * 0.000001;
  ms_sum += ms;
  ms_count++;
  if (ms_count % 1000 == 0) {
    cout << "Act: " << ms << " ms, avg: " << (ms_sum / ms_count) << " ms, sum: " << ms_sum << " ms by "
     << ms_count << " ticks" << endl;
  }
}

ostream& operator<<(ostream& stream, const model::Action& a) {
  stream << "(" << a.target_velocity_x << "," << a.target_velocity_y << "," << a.target_velocity_z << ")";
  return stream;
}
