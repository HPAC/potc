// Copyright 2018 Markus Hoehnerbach

#ifndef DER_H_
#define DER_H_

#include <memory>
#include <utility>

class CompoundIRStmt;
class PotentialDef;
class IRIdentifierContext;
class IRExpr;
class Expr;

namespace potc {

namespace der {

std::unique_ptr<CompoundIRStmt> GenerateCode(
    PotentialDef* def, IRIdentifierContext* next_identifier);
std::pair<std::unique_ptr<CompoundIRStmt>, std::unique_ptr<IRExpr>>
GenerateCode(PotentialDef* def, Expr* expr);

}  // namespace der

}  // namespace potc

#endif  // DER_H_
