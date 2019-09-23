// Copyright 2018 Markus Hoehnerbach
#ifndef GEN_INTERNAL_H_
#define GEN_INTERNAL_H_ GEN_INTERNAL_H_

#include <cassert>
#include <cstdarg>

#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace potc {

namespace gen {

// Templating library:
// Each template is a function
// Each function takes parameters
// And returns a string
// In each function there is a single string
// Needs: How to handle indentation?

inline std::string Undent(const std::string& from) {
  int count = 0;
  for (auto c : from) {
    if (c == '\n') continue;
    if (c == ' ')
      count += 1;
    else
      break;
  }
  std::stringstream result;
  bool first_linebreak = true;
  int current_count = 0;
  for (auto c : from) {
    if (first_linebreak) {
      assert(c == '\n');
      first_linebreak = false;
    } else if (c == '\n') {
      result << c;
      current_count = 0;
    } else if (current_count < count) {
      assert(c == ' ');
      current_count += 1;
    } else {
      result << c;
    }
  }
  return result.str();
}

inline std::string Indent(const std::string& from,
                          const std::string theIndent = "  ") {
  std::stringstream result;
  bool add_indent = true;
  for (auto c : from) {
    if (add_indent && c != '\n') {
      add_indent = false;
      result << theIndent;
    }
    result << c;
    if (c == '\n') {
      add_indent = true;
    }
  }
  return result.str();
}

inline std::string Format(const std::string& from,
                          const std::map<std::string, std::string>& args) {
  std::stringstream result;
  std::stringstream num_str;
  bool parse_num = false;
  for (auto c : from) {
    if (parse_num) {
      if (c == '@') {
        parse_num = false;
        result << args.at(num_str.str());
        num_str.str("");
      } else {
        num_str << c;
      }
    } else {
      if (c == '@') {
        parse_num = true;
      } else {
        result << c;
      }
    }
  }
  return result.str();
}

inline std::string FormatTemplate(
    const std::string& format, std::map<std::string, std::string> args,
    const std::vector<std::map<std::string, std::string>>& loop_args) {
  // printf("FMT >%s<\n", format.c_str());
  // #loop/# #if:tes/#
  // #loopname/# #ifname:tes/# @/@ @/@
  const bool DEBUG = false;
  std::stringstream result;
  std::stringstream cmd_str;
  std::stringstream matching_str;
  bool parse_cmd = false;
  bool parse_matching = false;
  int depth = 0;
  const char TRIGGER = '@';
  std::string LOOP_TRIGGER = "loop";
  std::string REVERSE_LOOP_TRIGGER = "rloop";
  std::string IF_TRIGGER = "if";
  std::string END_TRIGGER = "/";
  std::string matching_cmd;
  auto Split = [](char needle, const std::string& haystack, std::string* left,
                  std::string* right) {
    auto pos = haystack.find(needle);
    if (pos == std::string::npos) return false;
    *left = haystack.substr(0, pos);
    *right = haystack.substr(pos + 1);
    return true;
  };
  auto StartsWith = [](const std::string& haystack, const std::string& needle) {
    return haystack.substr(0, needle.size()) == needle;
  };
  auto IsCommand = [&](const std::string& s) {
    return StartsWith(s, LOOP_TRIGGER) || StartsWith(s, REVERSE_LOOP_TRIGGER) ||
           StartsWith(s, IF_TRIGGER);
  };
  for (auto c : format) {
    if (parse_cmd) {
      if (c == TRIGGER) {
        parse_cmd = false;
        if (parse_matching) {
          if (DEBUG)
            printf("MATCH: <%s> <%s>\n", matching_cmd.c_str(),
                   cmd_str.str().c_str());
          if (cmd_str.str() == END_TRIGGER) {
            depth -= 1;
            if (depth == 0) {
              parse_matching = false;
              if (DEBUG)
                printf("END: <%s> <%s>\n", matching_cmd.c_str(),
                       matching_str.str().c_str());
              if (StartsWith(matching_cmd, REVERSE_LOOP_TRIGGER)) {
                auto name = matching_cmd.substr(REVERSE_LOOP_TRIGGER.size());
                for (auto it = loop_args.rbegin(); it != loop_args.rend();
                     it++) {
                  auto& loop_arg = *it;
                  for (auto& arg : loop_arg) {
                    args[name + arg.first] = arg.second;
                  }
                  result << FormatTemplate(matching_str.str(), args, loop_args);
                }
              } else if (StartsWith(matching_cmd, LOOP_TRIGGER)) {
                auto name = matching_cmd.substr(LOOP_TRIGGER.size());
                for (auto& loop_arg : loop_args) {
                  for (auto& arg : loop_arg) {
                    args[name + arg.first] = arg.second;
                  }
                  result << FormatTemplate(matching_str.str(), args, loop_args);
                }
              } else if (StartsWith(matching_cmd, IF_TRIGGER)) {
                bool is_true = true;
                auto condition = matching_cmd.substr(IF_TRIGGER.size());
                std::string left, right;
                if (Split('=', condition, &left, &right)) {
                  if (args[left] != args[right]) {
                    is_true = false;
                  }
                } else if (Split('!', condition, &left, &right)) {
                  if (args[left] == args[right]) {
                    is_true = false;
                  }
                } else if (Split('<', condition, &left, &right)) {
                  std::stringstream converter;
                  converter.str(args[left]);
                  int left_num;
                  converter >> left_num;
                  converter.clear();
                  converter.str(args[right]);
                  int right_num;
                  converter >> right_num;
                  if (DEBUG)
                    printf("%d %d <%s> <%s> <%s> <%s>\n", left_num, right_num,
                           left.c_str(), right.c_str(), args[left].c_str(),
                           args[right].c_str());
                  if (left_num >= right_num) {
                    is_true = false;
                  }
                } else if (Split('>', condition, &left, &right)) {
                  std::stringstream converter;
                  converter.str(args[left]);
                  int left_num;
                  converter >> left_num;
                  converter.clear();
                  converter.str(args[right]);
                  int right_num;
                  converter >> right_num;
                  if (DEBUG)
                    printf("%d %d <%s> <%s> <%s> <%s>\n", left_num, right_num,
                           left.c_str(), right.c_str(), args[left].c_str(),
                           args[right].c_str());
                  if (left_num <= right_num) {
                    is_true = false;
                  }
                } else if (args[condition].empty()) {
                  is_true = false;
                }
                if (is_true) {
                  result << FormatTemplate(matching_str.str(), args, loop_args);
                }
              } else {
                assert(0);
              }
              matching_str.str("");
              cmd_str.str("");
              continue;
            }  // depth == 0
          } else if (IsCommand(cmd_str.str())) {
            depth += 1;
          }
          matching_str << TRIGGER << cmd_str.str() << TRIGGER;
          cmd_str.str("");
        } else {  // ! parse_matching
          if (DEBUG) printf("CMD: <%s>\n", cmd_str.str().c_str());
          if (cmd_str.str() == END_TRIGGER) {
            assert(0);
          } else if (IsCommand(cmd_str.str())) {
            depth += 1;
            parse_matching = true;
            matching_cmd = cmd_str.str();
            matching_str.str("");
          } else if (cmd_str.str()[0] == ' ' || cmd_str.str()[0] == '\n') {
            // skip whitespace markers
          } else {
            result << args[cmd_str.str()];
          }
          cmd_str.str("");
        }
      } else {  // c != TRIGGER
        cmd_str << c;
      }
    } else {  // ! parse_cmd
      if (c == TRIGGER) {
        parse_cmd = true;
      } else {  // c != TRIGGER
        if (parse_matching) {
          matching_str << c;
        } else {
          result << c;
        }
      }
    }
  }
  assert(depth == 0);
  assert(!parse_cmd);
  assert(!parse_matching);
  if (DEBUG) printf("RETURN <%s>\n", result.str().c_str());
  return result.str();
}

struct CodeBuilder {
  FILE* out;
  std::stringstream content_;
  CodeBuilder* Fragment(std::string str) {
    if (out) fputs(str.c_str(), out);
    content_ << str;
    return this;
  }

  std::string indent_;
  CodeBuilder* StartFragment() { return Fragment(indent_); }
  CodeBuilder* Indent() {
    indent_ += "  ";
    return this;
  }
  CodeBuilder* Undent() {
    indent_ = indent_.substr(2);
    return this;
  }

  CodeBuilder* EndFragment() { return Fragment("\n"); }
  CodeBuilder* Line(std::string str) {
    if (str.empty()) return EndFragment();
    return StartFragment()->Fragment(str)->EndFragment();
  }
  CodeBuilder* If(std::string cond) {
    return Line("if (" + cond + ") {")->Indent();
  }
  CodeBuilder* Else() { return Undent()->Line("} else {")->Indent(); }
  CodeBuilder* End() { return Undent()->Line("}"); }
  CodeBuilder* For(std::string inner) {
    return Line("for (" + inner + ") {")->Indent();
  }
  CodeBuilder* Raw(std::string fmt, ...) {
    va_list args;
    va_list args_alt;
    va_start(args, fmt);
    va_copy(args_alt, args);
    const int MAX_BUF = 256;
    char buf[MAX_BUF];
    char* buf_ptr = buf;
    char* buf_alt = nullptr;
    int ret = vsnprintf(buf, MAX_BUF, fmt.c_str(), args);
    if (ret >= MAX_BUF) {
      buf_alt = new char[ret + 1];
      vsnprintf(buf_alt, ret + 1, fmt.c_str(), args_alt);
      buf_ptr = buf_alt;
    }
    // assert(ret < MAX_BUF);  // <, not <= since also \0 needs to be added
    va_end(args);
    va_end(args_alt);
    auto self = Fragment(buf_ptr);
    delete[] buf_alt;
    return self;
  }
  template <typename... Args>
  CodeBuilder* RawL(std::string fmt, Args... args) {
    return this->StartFragment()->Raw(fmt, args...)->EndFragment();
  }
};

// class UnitBuilder {
//
//  CodeBuilder* AddClassMethod(std::string name_and_args, std::string ret);
//  CodeBuilder* AddStandaloneMethod(std::string name_and_args, std::string
//  ret);
//  void AddVariable(std::string decl);
//  std::string Uniquify(std::string str);
//
//};
//
// class RegularGenerator {
//  void WriteSplineEval();
//  void WriteSplineInit();
//  void WriteSplineHeader();
//  void WriteSplineZero();
//  void WriteSplineAlloc();
//  void WriteSplineDealloc();
//  void WriteFileProcessLine();
//  void WriteHeader();
//  void WriteBody();
//  void WriteConstructor();
//  void WriteDestructor();
//  void WriteCompute();
//  void WriteInitStyle();
//  void WriteAllocate();
//  void WriteInitOne();
//  void WriteSettings();
//};

}  // namespace gen

}  // namespace potc

#endif  // GEN_INTERNAL_H_
