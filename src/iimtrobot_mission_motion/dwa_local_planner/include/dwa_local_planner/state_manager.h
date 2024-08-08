#ifndef DWA_STATE_MANAGER_H
#define DWA_STATE_MANAGER_H
#include <common/common.h>
#include <common_base/fsm_base.h>
class StateManager {
 public:
  StateManager(){};
  ~StateManager() = default;

  void Reset() { controller_fsm_.Set(CONTROLLER_STATE::FREE_PATH); }
  CONTROLLER_STATE GetState() { return controller_fsm_.GetState(); }
  void SetState(CONTROLLER_STATE state) { controller_fsm_.Set(state); }
  void SetCommand(CONTROLLER_EVENT event) { controller_fsm_.Command(event); }

 private:
  FsmMachine<CONTROLLER_STATE, CONTROLLER_EVENT> controller_fsm_;
};

#endif