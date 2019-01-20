#include "MyStrategy.h"
#include "engine.h"
#include "ofxMSAmcts.h"
#include "Starter.h"

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

  Vector3D last_pos, last_velocity;
  double last_time;
  char collide_type = state.corridor_last_point(state.ball, last_pos, last_velocity, last_time);
  *debug_text += "ball collide type: ";
  *debug_text += collide_type ? collide_type : '0';
  *debug_text += "\\n";
  if (collide_type) {
    debug_draw->push_back({last_pos.x, last_pos.y, last_pos.z, 2, 1, 0, 0, 1});
  }
  return;

  for (int i = 0; i < 200; i++) {
//    for (RobotEntity& r : state.robots) {
//      r.action.target_velocity_y = 1000;
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
int starter_startegy_cooldown = 0;

int normalized_tick(const MyStrategy* self, const model::Game& game, const Rules& rules) {
  return max(0, game.current_tick - self->last_goal_tick);
}

void engine_tick(MyStrategy* self, const Robot& me, const Rules& rules, const Game& game, Action& action, map<int,HistoryItem>& history) {
  Engine engine(me, rules, game, history);
  if (normalized_tick(self, game, rules) < 100) {
    engine.apply_defender();
  }
  action = engine.find_best();
  history[me.id] = {game.current_tick, action};
}

void MyStrategy::act(const Robot& me, const Rules& rules, const Game& game, Action& action) {
  steady_clock::time_point start = steady_clock::now();
  apply_score_changes(me, rules, game, action);
  //if (use_prev_action(me, rules, game, action)) return;
  //if (game.current_tick < 10000) return;

  debug_text->clear();
  debug_draw->clear();

  int available_ms = ms_count * 20 + 20000 - ms_sum;
#ifdef MY_DEBUG
  check_engine_correctness(me, rules, game, action,history);
  engine_tick(this, me, rules, game, action, history);
#else
  if (false && (starter_startegy_cooldown > 0 || available_ms < 1000 /*0*/)) {
    if (starter_startegy_cooldown <= 0) {
      starter_startegy_cooldown += 50;
      cout << "Time exceeded, use Starter by " << 50 << " next ticks. Tick=" << ms_count << ", ms_sum="
        << ms_sum << ", available_ms="
        << available_ms << ", diff=" << (available_ms - ms_sum) << endl;
    }
    starter_startegy_cooldown--;
    // simple strategy
    Starter starter;
    starter.act(me, rules, game, action);
  } else {
    engine_tick(this, me, rules, game, action, history);
  }
#endif

  double ms = duration_cast<nanoseconds>(steady_clock::now() - start).count() * 0.000001;
  ms_sum += ms;
  ms_count++;
  if (ms_count % 2000 == 0) {
    cout << "tick=" << ms_count << " ms=" << ms << " avg=" << (ms_sum / ms_count) << " sum=" << ms_sum << " available=" << available_ms << endl;
  }
}

ostream& operator<<(ostream& stream, const model::Action& a) {
  stream << "vx=" << a.target_velocity_x << " vy=" << a.target_velocity_y << " vz=" << a.target_velocity_z
    << " j=" << a.jump_speed;
  return stream;
}
