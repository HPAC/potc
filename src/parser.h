// Copyright 2018 Markus Hoehnerbach

#ifndef PARSER_H_
#define PARSER_H_

#include <memory>

class PotentialDef;

namespace potc {

std::unique_ptr<PotentialDef> Parse(const char *);
}  // namespace potc

#endif  // PARSER_H_
