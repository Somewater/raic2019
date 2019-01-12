#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif

#ifndef _MY_STRATEGY_H_
#define _MY_STRATEGY_H_

#include "Strategy.h"
#include "math.h"
#include "engine.h"
#include <map>
#include "Starter.h"

class MyStrategy : public Strategy {
public:

  MyStrategy();

  void act(const model::Robot& me, const model::Rules& rules, const model::Game& world, model::Action& action) override;
  bool use_prev_action(const Robot& me, const Rules& rules, const Game& game, Action& action) {
    if (history.count(me.id)) {
      HistoryItem& it = history.at(me.id);
      if (game.current_tick - it.current_tick < 3) {
        action = it.action;
        return true;
      }
    }
    return false;
  }

#ifdef MY_DEBUG
  std::string custom_rendering() override;
#endif
  map<int, HistoryItem> history;
};

#endif
