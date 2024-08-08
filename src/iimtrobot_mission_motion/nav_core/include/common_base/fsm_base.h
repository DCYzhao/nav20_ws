#ifndef FSM_BASE_H_
#define FSM_BASE_H_
#include <ros/ros.h>

#include <functional>
#include <map>
#include <utility>
template <class T_STATE, class T_COMMAD> class FsmMachine {
public:
  FsmMachine() = default;
  ~FsmMachine() = default;

  T_STATE GetState() { return current_state_; }
  bool IsState(T_STATE state) { return (current_state_ == state); }
  void SetState(T_STATE state) {
    current_state_ = state;
    state_start_time_ = ros::Time::now();
  }
  void Set(const T_STATE &state) {
    current_state_ = state;
    state_start_time_ = ros::Time::now();
  }
  void Command(const T_COMMAD &command) {
    auto found =
        state_command_func_.find(std::make_pair(current_state_, command));
    if (found != state_command_func_.end()) {
      found->second();
    }
  }
  std::function<void()> &On(const T_STATE &state, const T_COMMAD &command) {
    return state_command_func_[std::make_pair(state, command)];
  }

  double GetStateKeepSec() {
    return (ros::Time::now() - state_start_time_).toSec();
  }

private:
  T_STATE current_state_;
  T_STATE last_state_;
  ros::Time state_start_time_;
  std::map<std::pair<T_STATE, T_COMMAD>, std::function<void()>>
      state_command_func_;
};
#endif
