// Copyright 2019 Markus Hoehnerbach
#ifndef ANA_COMM_H_
#define ANA_COMM_H_ ANA_COMM_H_

#include <cassert>
#include <map>
#include <vector>

#include "ir.h"
#include "ir_visitor.h"

namespace potc {

namespace ana {

struct AnalyzeCommunication {
  std::vector<std::vector<PerAtomActionIRStmt*>> stage_assignment;
  enum Direction { Forward = 1, Reverse = 2, Both = 3 };
  std::vector<PerAtomActionIRStmt::Direction> stage_direction;
  std::map<PerAtomActionIRStmt*, int> stage_assignment_reverse;
  std::map<PerAtomActionIRStmt*, bool> should_generate;
  PerAtomActionIRStmt* last = nullptr;

  void Visit(CompoundIRStmt* comp) {
    for (auto& stmt : comp->body) {
      auto comm = stmt->asa<PerAtomActionIRStmt>();
      if (comm && !comm->IsComm()) comm = nullptr;
      if (comm) {
        // if (last && last->kind != comm->kind) {
        //   last = nullptr;
        // }
        if (!last) {
          stage_assignment.push_back({});
          stage_direction.push_back(PerAtomActionIRStmt::None);
          last = comm;
          should_generate[comm] = true;
        }
        stage_assignment_reverse[comm] = stage_assignment.size() - 1;
        stage_assignment.back().push_back(comm);
        stage_direction.back() = comm->AugmentDirection(stage_direction.back());
      } else if (last) {
        last = nullptr;
      }
    }
  }

  bool IsForward(int stage) {
    return stage_direction[stage] & PerAtomActionIRStmt::Forward;
  }
  bool IsReverse(int stage) {
    return stage_direction[stage] & PerAtomActionIRStmt::Reverse;
  }
};

}  // namespace ana

}  // namespace potc

#endif  // ANA_COMM_H_
