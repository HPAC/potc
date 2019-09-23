// Copyright 2018 Markus Hoehnerbach

#include "parser.h"  // NOLINT

#include <cassert>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <vector>

#include "expr.h"  // NOLINT

namespace potc {

namespace {

struct Token {
  const char* str;
  size_t len;
  enum Kind {
    kUnknown,
    kIdentifier,
    kNumericalConstant,
    kParenLeft,
    kParenRight,
    kEqual,
    kColon,
    kAdd,
    kSub,
    kMul,
    kDiv,
    kPow,
    kLess,
    kGreater,
    kKwParameter,
    kKwFunction,
    kKwPeratom,
    kKwEnergy,
    kKwAtomType,
    kKwAtom,
    kKwDistance,
    kKwAngle,
    kKwReal,
    kKwSum,
    kKwAllAtoms,
    kKwNeighborsHalf,
    kKwNeighbors,
    kKwImplicit,
    kKwLet,
    kKwPiecewise,
    kKwTypematch,
    kKwFile,
    kKwSpline,
    kKwSplineInteger,
    kKwSplineGrid,
    kEof,
    kSof,
    kSemicolon,
    kComma
  };
  Kind kind;

  Token(const char* s, size_t l, Kind k) : str(s), len(l), kind(k) {}
  void Print() { printf("<%d:%s>\n", kind, std::string(str, len).c_str()); }
};

static bool between(char c, char lo, char hi) { return c >= lo && c <= hi; }

// TODO(markus) make sure start with _ only occurs internally
static bool id_first(char c) {
  return between(c, 'a', 'z') || between(c, 'A', 'Z') || c == '_';
}

// TODO(markus) disallow '
static bool id_rest(char c) {
  return id_first(c) || between(c, '0', '9') || c == '\'' || c == '_';
}

struct Lexer {
  const char* const src_;
  size_t len_;
  size_t offset_;
  size_t row_;
  size_t col_;
  size_t offset_row_;
  Token current_token_;
  std::unordered_map<std::string, Token::Kind> matchers_;
  static const bool DEBUG = false;

  void SetToken(size_t token_len, Token::Kind token_kind) {
    current_token_ = Token(&src_[offset_], token_len, token_kind);
    if (DEBUG) current_token_.Print();
    offset_ += token_len;
    col_ += token_len;
  }

  void AdvanceWhitespace() {
    if (src_[offset_] == '\0') return;
    while (src_[offset_] == ' ' || src_[offset_] == '\n') {
      if (src_[offset_] == '\n') {
        row_ += 1;
        col_ = 0;
        offset_ += 1;
        offset_row_ = offset_;
      } else {
        col_ += 1;
        offset_ += 1;
      }
    }
  }

  void AdvanceComment() {
    do {
      AdvanceWhitespace();
      if (src_[offset_] == '#') {
        offset_ += 1;
        col_ += 1;
        while (src_[offset_] != '\n' && src_[offset_] != '\0') {
          offset_ += 1;
          col_ += 1;
        }
        if (src_[offset_] == '\n') {
          row_ += 1;
          col_ = 0;
          offset_ += 1;
          offset_row_ = offset_;
        }
      } else {
        break;
      }
    } while (1);
  }

 public:
  explicit Lexer(const char* src)
      : src_(src),
        len_(strlen(src)),
        offset_(0),
        row_(0),
        col_(0),
        offset_row_(0),
        current_token_(nullptr, 0, Token::kSof) {
    matchers_["piecewise"] = Token::kKwPiecewise;
    matchers_["typematch"] = Token::kKwTypematch;
    matchers_["parameter"] = Token::kKwParameter;
    matchers_["function"] = Token::kKwFunction;
    matchers_["peratom"] = Token::kKwPeratom;
    matchers_["energy"] = Token::kKwEnergy;
    matchers_["atom_type"] = Token::kKwAtomType;
    matchers_["atom"] = Token::kKwAtom;
    matchers_["distance"] = Token::kKwDistance;
    matchers_["angle"] = Token::kKwAngle;
    matchers_["sum"] = Token::kKwSum;
    matchers_["all_atoms"] = Token::kKwAllAtoms;
    matchers_["file"] = Token::kKwFile;
    matchers_["spline"] = Token::kKwSpline;
    matchers_["spline_integer"] = Token::kKwSplineInteger;
    matchers_["spline_grid"] = Token::kKwSplineGrid;
    matchers_["neighbors_half"] = Token::kKwNeighborsHalf;
    matchers_["neighbors"] = Token::kKwNeighbors;
    matchers_["implicit"] = Token::kKwImplicit;
    matchers_["let"] = Token::kKwLet;
    matchers_["real"] = Token::kKwReal;
  }

  void AdvanceToken() {
    AdvanceComment();
    if (offset_ == len_) {
      assert(src_[offset_] == '\0');
      SetToken(0, Token::kEof);
      return;
    }
    assert(src_[offset_] != '\0');
    assert(offset_ < len_);
    switch (src_[offset_]) {
      case '(':
        SetToken(1, Token::kParenLeft);
        return;
      case ')':
        SetToken(1, Token::kParenRight);
        return;
      case ';':
        SetToken(1, Token::kSemicolon);
        return;
      case ',':
        SetToken(1, Token::kComma);
        return;
      case ':':
        SetToken(1, Token::kColon);
        return;
      case '+':
        SetToken(1, Token::kAdd);
        return;
      case '-':
        SetToken(1, Token::kSub);
        return;
      case '*':
        SetToken(1, Token::kMul);
        return;
      case '/':
        SetToken(1, Token::kDiv);
        return;
      case '^':
        SetToken(1, Token::kPow);
        return;
      case '=':
        SetToken(1, Token::kEqual);
        return;
      case '<':
        SetToken(1, Token::kLess);
        return;
      case '>':
        SetToken(1, Token::kGreater);
        return;
      case '0':
      case '1':
      case '2':
      case '3':
      case '4':
      case '5':
      case '6':
      case '7':
      case '8':
      case '9': {
        size_t match_len = 1;
        while (match_len < len_ - offset_ && src_[offset_ + match_len] >= '0' &&
               src_[offset_ + match_len] <= '9')
          match_len += 1;
        SetToken(match_len, Token::kNumericalConstant);
        return;
      }
      default:
        if (id_first(src_[offset_])) {
          size_t idx = 1;
          while (idx < len_ - offset_ && id_rest(src_[offset_ + idx])) idx += 1;
          std::string name(&src_[offset_], idx);
          if (matchers_.count(name) > 0) {
            SetToken(idx, matchers_[name]);
          } else {
            SetToken(idx, Token::kIdentifier);
          }
          return;
        }
    }
    printf("Unrecognized: %c\n", src_[offset_]);
    SetToken(0, Token::kUnknown);
  }

  Token GetToken() {
    if (current_token_.kind == Token::kSof) AdvanceToken();
    return current_token_;
  }

  bool IsEnd() {
    return GetToken().kind == Token::kEof || GetToken().kind == Token::kUnknown;
  }
};

template <typename T>
struct ParseResult {
  std::unique_ptr<T> value;
  bool matched;
  bool error;
  // const char* error_message;
  std::string error_message;

  ParseResult() : matched(false), error(false) {}
  explicit ParseResult(T* t) : value(t), matched(true), error(false) {}
  template <typename S>
  ParseResult(ParseResult<S>&& t)
      : value(t.value.release()),
        matched(t.matched),
        error(t.error),
        error_message(t.error_message) {}

  template <typename S>
  ParseResult<T>& operator=(ParseResult<S>&& t) {
    value.reset(t.value.release());
    matched = t.matched;
    error = t.error;
    error_message = t.error_message;
    return *this;
  }

  template <typename S, typename TT = T>
  void Lift(ParseResult<S> result,
            void (std::enable_if<std::is_class<TT>::value, TT>::type::*
                      ptr)(             // NOLINT
                std::unique_ptr<S> s),  // void (T::*ptr)(std::unique_ptr<S> s),
            bool expect = false) {
    if (error) return;
    if (result.error) {
      error = true;
      error_message = result.error_message;
    } else if (result.matched) {
      (value.get()->*ptr)(std::move(result.value));
    } else if (expect) {
      error = true;
      error_message = "Expected match.\n" + result.error_message;
    }
  }
};

class Parser {
  Lexer lex_;
  bool has_progressed_;

  bool Accept(Token::Kind k) {
    if (lex_.GetToken().kind == k) {
      lex_.AdvanceToken();
      has_progressed_ = true;
      return true;
    }
    return false;
  }

  template <typename T>
  bool Accept(const ParseResult<T>& r, Token::Kind k) {
    return !r.error && Accept(k);
  }

  bool Check(Token::Kind k) { return lex_.GetToken().kind == k; }

  template <typename T>
  bool Check(const ParseResult<T>& r, Token::Kind k) {
    return !r.error && Check(k);
  }

  template <typename T>
  void Expect(ParseResult<T>& r, Token::Kind k,  // NOLINT
              void (T::*ptr)(const char* str, size_t len) = nullptr) {
    if (r.error) return;
    if (lex_.GetToken().kind == k) {
      if (ptr)
        ((r.value).get()->*ptr)(lex_.GetToken().str, lex_.GetToken().len);
      lex_.AdvanceToken();
      has_progressed_ = true;
    } else {
      r.error = true;
      r.error_message = "Unexpected token.";
    }
  }

  ParseResult<ParameterSpec> ParseSpline(
      const ParseResult<ParameterDecl>& parent, Token::Kind kind) {
    ParseResult<SplineParameterSpec> result(new SplineParameterSpec);
    result.value->is_integer = kind == Token::kKwSplineInteger;
    Expect(result, Token::kParenLeft);
    Expect<SplineParameterSpec>(result, Token::kIdentifier,
                                &SplineParameterSpec::SetName);
    Expect(result, Token::kComma);
    Expect<SplineParameterSpec>(result, Token::kNumericalConstant,
                                &SplineParameterSpec::SetOrder);
    // for (int i = 0; i < parent.value->GetArguments().size(); i++) {
    //  Expect(result, Token::kComma);
    //  Expect<SplineParameterSpec>(result, Token::kNumericalConstant,
    //                              &SplineParameterSpec::AddNodesAlong);
    //}
    while (Check(result, Token::kComma)) {
      Expect<SplineParameterSpec>(result, Token::kComma,
                                  &SplineParameterSpec::AddDerivTerm);
      while (Check(result, Token::kNumericalConstant)) {
        Expect<SplineParameterSpec>(result, Token::kNumericalConstant,
                                    &SplineParameterSpec::AddDerivElem);
      }
    }
    Expect(result, Token::kParenRight);
    return result;
  }
  ParseResult<ParameterSpec> ParseParameterSpec(
      const ParseResult<ParameterDecl>& parent) {
    if (Accept(Token::kKwFile)) {
      ParseResult<FileParameterSpec> result(new FileParameterSpec);
      Expect(result, Token::kParenLeft);
      Expect<FileParameterSpec>(result, Token::kNumericalConstant,
                                &FileParameterSpec::SetFileIndex);
      Expect(result, Token::kParenRight);
      return result;
    } else if (Accept(Token::kKwSpline)) {
      return ParseSpline(parent, Token::kKwSpline);
    } else if (Accept(Token::kKwSplineInteger)) {
      return ParseSpline(parent, Token::kKwSplineInteger);
    } else if (Accept(Token::kKwSplineGrid)) {
      ParseResult<SplineParameterSpec> result(new SplineParameterSpec);
      result.value->is_grid = true;
      Expect(result, Token::kParenLeft);
      Expect<SplineParameterSpec>(result, Token::kIdentifier,
                                  &SplineParameterSpec::SetName);
      Expect(result, Token::kComma);
      Expect<SplineParameterSpec>(result, Token::kNumericalConstant,
                                  &SplineParameterSpec::SetOrder);
      Expect(result, Token::kComma);
      result.Lift<Expr>(ParseExpr(), &SplineParameterSpec::SetLow, true);
      Expect(result, Token::kComma);
      result.Lift<Expr>(ParseExpr(), &SplineParameterSpec::SetStep, true);
      while (Check(result, Token::kComma)) {
        Expect<SplineParameterSpec>(result, Token::kComma,
                                    &SplineParameterSpec::AddDerivTerm);
        while (Check(result, Token::kNumericalConstant)) {
          Expect<SplineParameterSpec>(result, Token::kNumericalConstant,
                                      &SplineParameterSpec::AddDerivElem);
        }
      }
      Expect(result, Token::kParenRight);
      return result;
    }
    return ParseResult<ParameterSpec>();
  }

  ParseResult<Decl> ParseParameterDecl() {
    if (!Accept(Token::kKwParameter)) return ParseResult<Decl>();
    ParseResult<ParameterDecl> result(new ParameterDecl);
    Expect<ParameterDecl>(result, Token::kIdentifier, &ParameterDecl::SetName);
    if (Accept(result, Token::kParenLeft)) {
      do {
        Expect<ParameterDecl>(result, Token::kIdentifier,
                              &ParameterDecl::AddArgument);
        Expect(result, Token::kColon);
        if (Accept(result, Token::kKwAtomType)) {
          // NOOP
          result.value->AddArgumentType(Type::kAtomType);
        } else if (Accept(result, Token::kKwDistance)) {
          result.value->AddArgumentType(Type::kDistance);
          // NOOP
        } else {
          Expect(result, Token::kKwReal);
          result.value->AddArgumentType(Type::kReal);
        }
      } while (Accept(result, Token::kSemicolon));
      Expect(result, Token::kParenRight);
    }
    Expect(result, Token::kEqual);
    result.Lift<ParameterSpec>(ParseParameterSpec(result),
                               &ParameterDecl::SetParameterSpec, true);
    Expect(result, Token::kSemicolon);
    return result;
  }

  ParseResult<Type> ParseType() {
    if (Accept(Token::kKwAtomType))
      return ParseResult<Type>(new Type(Type::kAtomType));
    else if (Accept(Token::kKwAtom))
      return ParseResult<Type>(new Type(Type::kAtom));
    else if (Accept(Token::kKwDistance))
      return ParseResult<Type>(new Type(Type::kDistance));
    else if (Accept(Token::kKwAngle))
      return ParseResult<Type>(new Type(Type::kAngle));
    else if (Accept(Token::kKwReal))
      return ParseResult<Type>(new Type(Type::kReal));
    else
      return ParseResult<Type>();
  }

  ParseResult<Decl> ParseFunctionDecl() {
    ParseResult<FunctionDecl> result;
    if (Accept(Token::kKwFunction)) {
      result = ParseResult<FunctionDecl>(new FunctionDecl);
    } else if (Accept(Token::kKwPeratom)) {
      result = ParseResult<PerAtomDecl>(new PerAtomDecl);
    } else {
      return ParseResult<Decl>();
    }

    // TODO(markus) enforce just one atom parameter if is_peratom
    Expect<FunctionDecl>(result, Token::kIdentifier, &FunctionDecl::SetName);
    if (Accept(result, Token::kParenLeft)) {
      do {
        Expect<FunctionDecl>(result, Token::kIdentifier,
                             &FunctionDecl::AddArgument);
        Expect(result, Token::kColon);
        result.Lift<Type>(ParseType(), &NamedDecl::SetLastArgType, true);
      } while (Accept(result, Token::kSemicolon));
      Expect(result, Token::kParenRight);
    }
    Expect(result, Token::kEqual);
    result.Lift(ParseExpr(), &FunctionDecl::SetExpr, true);
    Expect(result, Token::kSemicolon);
    return result;
  }

  inline ParseResult<Expr> ParseExpr();

  ParseResult<Expr> ParseIter() {
    bool neigh_half = false, neigh_full = false;
    if (Accept(Token::kKwAllAtoms)) {
      return ParseResult<Expr>(new AllAtomsListExpr);
    } else if (Accept(Token::kKwNeighborsHalf)) {
      neigh_half = true;
    } else if (Accept(Token::kKwNeighbors)) {
      neigh_full = true;
    }
    if (neigh_half || neigh_full) {  // TODO(markus) structure appropriately.
      ParseResult<NeighborsListExpr> result(new NeighborsListExpr(neigh_half));
      Expect(result, Token::kParenLeft);
      Expect(result, Token::kIdentifier, &NeighborsListExpr::SetIndexStr);
      Expect(result, Token::kComma);
      result.Lift(ParseExpr(), &NeighborsListExpr::SetCutoff, true);
      while (!result.error && !Accept(result, Token::kParenRight)) {
        Expect(result, Token::kComma);
        if (Accept(result, Token::kGreater)) {
          Expect(result, Token::kIdentifier, &NeighborsListExpr::SetGreaterStr);
          continue;
        }
        Expect(result, Token::kIdentifier, &NeighborsListExpr::AddExclusionStr);
      }
      return result;
    }
    return ParseResult<Expr>();
  }

  ParseResult<PiecewiseExpr::Piece> ParsePiece() {
    ParseResult<PiecewiseExpr::Piece> result(new PiecewiseExpr::Piece);
    result.Lift(ParseExpr(), &PiecewiseExpr::Piece::SetFirst, true);
    if (Accept(result, Token::kLess)) {
      result.value->greater = false;
    } else {
      Expect(result, Token::kGreater);
      result.value->greater = true;
    }
    result.value->first_equal = Accept(result, Token::kEqual);
    result.Lift(ParseExpr(), &PiecewiseExpr::Piece::SetMid, true);
    result.value->last_equal = false;
    if (result.value->greater) {
      if (Accept(result, Token::kGreater)) {
        result.value->last_equal = Accept(result, Token::kEqual);
        result.Lift(ParseExpr(), &PiecewiseExpr::Piece::SetLast, true);
      }
    } else {
      if (Accept(result, Token::kLess)) {
        result.value->last_equal = Accept(result, Token::kEqual);
        result.Lift(ParseExpr(), &PiecewiseExpr::Piece::SetLast, true);
      }
    }
    Expect(result, Token::kColon);
    result.Lift(ParseExpr(), &PiecewiseExpr::Piece::SetExpr, true);
    return result;
  }

  ParseResult<Expr> ParseAtom() {
    if (Check(Token::kNumericalConstant)) {
      ParseResult<NumericalConstantExpr> result(new NumericalConstantExpr);
      Expect(result, Token::kNumericalConstant,
             &NumericalConstantExpr::SetValueStr);
      return result;
    } else if (Check(Token::kIdentifier)) {
      ParseResult<IdentifierExpr> result(new IdentifierExpr);
      Expect(result, Token::kIdentifier, &IdentifierExpr::SetName);
      if (Accept(result, Token::kParenLeft)) {
        if (Accept(result, Token::kParenRight)) return result;
        do {
          result.Lift(ParseExpr(), &IdentifierExpr::AddArgument, true);
        } while (Accept(result, Token::kComma));
        Expect(result, Token::kParenRight);
      }
      return result;
    } else if (Accept(Token::kParenLeft)) {
      auto result = ParseExpr();
      Expect(result, Token::kParenRight);
      return result;
    } else if (Accept(Token::kKwSum)) {
      ParseResult<SumExpr> result(new SumExpr);
      Expect(result, Token::kParenLeft);
      Expect(result, Token::kIdentifier, &SumExpr::SetIndex);
      Expect(result, Token::kColon);
      result.Lift(ParseIter(), &SumExpr::SetIter, true);
      Expect(result, Token::kParenRight);
      result.Lift(ParseExpr(), &SumExpr::SetExpr, true);
      return result;
    } else if (Accept(Token::kKwLet)) {
      ParseResult<LetExpr> result(new LetExpr);
      Expect(result, Token::kParenLeft);
      Expect(result, Token::kIdentifier, &LetExpr::SetNameStr);
      Expect(result, Token::kColon);
      result.Lift(ParseExpr(), &LetExpr::SetValue, true);
      Expect(result, Token::kParenRight);
      result.Lift(ParseExpr(), &LetExpr::SetExpr, true);
      return result;
    } else if (Accept(Token::kKwImplicit)) {
      ParseResult<ImplicitExpr> result(new ImplicitExpr);
      Expect(result, Token::kParenLeft);
      do {
        Expect(result, Token::kIdentifier, &ImplicitExpr::AddImplicitStr);
        Expect(result, Token::kColon);
        Expect(result, Token::kIdentifier, &ImplicitExpr::SetLastAliasStr);
      } while (Accept(result, Token::kSemicolon));
      Expect(result, Token::kParenRight);
      result.Lift(ParseExpr(), &ImplicitExpr::SetExpr, true);
      return result;
    } else if (Accept(Token::kKwPiecewise)) {
      ParseResult<PiecewiseExpr> result(new PiecewiseExpr);
      Expect(result, Token::kParenLeft);
      do {
        result.Lift(ParsePiece(), &PiecewiseExpr::AddPiece, true);
      } while (Accept(result, Token::kSemicolon));
      Expect(result, Token::kParenRight);
      return result;
    } else if (Accept(Token::kKwTypematch)) {
      ParseResult<TypematchExpr> result(new TypematchExpr);
      Expect(result, Token::kParenLeft);
      do {
        Expect(result, Token::kIdentifier, &TypematchExpr::AddArg);
      } while (Accept(result, Token::kComma));
      Expect(result, Token::kSemicolon);
      do {
        ParseResult<TypematchExpr::Piece> piece(new TypematchExpr::Piece);
        for (int i = 0; i < result.value->args_.size(); i++) {
          if (i > 0) Expect(piece, Token::kComma);
          Expect(piece, Token::kIdentifier, &TypematchExpr::Piece::AddType);
        }
        Expect(piece, Token::kColon);
        piece.Lift(ParseExpr(), &TypematchExpr::Piece::SetExpr, true);
        result.Lift(std::move(piece), &TypematchExpr::AddPiece, true);
      } while (Accept(result, Token::kSemicolon));
      Expect(result, Token::kParenRight);
      return result;
    }
    return ParseResult<Expr>();
  }

  ParseResult<Expr> ParsePow() {
    ParseResult<PowExpr> result(new PowExpr);
    result.Lift(ParseAtom(), &PowExpr::SetLeft, true);
    if (Accept(result, Token::kPow)) {
      result.Lift(ParseAtom(), &PowExpr::SetRight, true);
    }
    return result;
  }

  ParseResult<Expr> ParseMul() {
    ParseResult<MulExpr> result(new MulExpr);
    result.Lift(ParsePow(), &MulExpr::Add, true);
    while (Check(result, Token::kMul) || Check(result, Token::kDiv)) {
      if (Accept(result, Token::kDiv)) {
        result.Lift(ParsePow(), &MulExpr::AddInv, true);
      } else if (Accept(result, Token::kMul)) {
        result.Lift(ParsePow(), &MulExpr::Add, true);
      }
    }
    return result;
  }

  ParseResult<Expr> ParseSum() {
    ParseResult<AddExpr> result(new AddExpr);
    do {
      if (Accept(result, Token::kSub)) {
        result.Lift(ParseMul(), &AddExpr::AddNeg, true);
      } else if (Accept(result, Token::kAdd)) {
        result.Lift(ParseMul(), &AddExpr::Add, true);
      } else {
        result.Lift(ParseMul(), &AddExpr::Add, true);
      }
    } while (Check(result, Token::kSub) || Check(result, Token::kAdd));
    return result;
  }

  ParseResult<Decl> ParseEnergyDecl() {
    if (!Accept(Token::kKwEnergy)) return ParseResult<Decl>();
    ParseResult<EnergyDecl> result(new EnergyDecl);
    result.Lift(ParseExpr(), &EnergyDecl::SetExpr, true);
    Expect(result, Token::kSemicolon);
    return result;
  }

  ParseResult<PotentialDef> ParsePotentialDef() {
    ParseResult<PotentialDef> result(new PotentialDef);
    while (!lex_.IsEnd() && has_progressed_ && !result.error) {
      has_progressed_ = false;
      result.Lift(ParseParameterDecl(), &PotentialDef::Add);
      result.Lift(ParseFunctionDecl(), &PotentialDef::Add);
      result.Lift(ParseEnergyDecl(), &PotentialDef::Add);
    }
    if (!result.error) {
      if (!lex_.IsEnd()) {
        result.error = true;
        result.error_message = "Trailing token.";
      } else if (!result.value->IsValid()) {
        result.error = true;
        result.error_message = "No energy declaration found.";
      }
    }
    return result;
  }

 public:
  explicit Parser(const char* str) : lex_(str), has_progressed_(true) {}

  std::unique_ptr<PotentialDef> Parse() {
    auto result = ParsePotentialDef();
    // result.value->Print();
    if (result.error) {
      result.value.reset(nullptr);
      printf("Error(%zd, %zd): %s\n", lex_.row_ + 1, lex_.col_ + 1,
             result.error_message.c_str());
    }
    return std::move(result.value);
  }
};

inline ParseResult<Expr> Parser::ParseExpr() { return ParseSum(); }

}  // namespace

std::unique_ptr<PotentialDef> Parse(const char* str) {
  Parser p(str);
  return p.Parse();
}

}  // namespace potc
