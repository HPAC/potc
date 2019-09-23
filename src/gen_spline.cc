#include "gen_spline.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <map>
#include <set>
#include <vector>

#include "gen.h"

namespace potc {

namespace gen {

namespace {

std::vector<int> SplineNew(int dims, int order) {
  return std::vector<int>(PowInt(order + 1, dims), 1);
}

std::vector<std::vector<int>> SplineIterator(int dims, int order) {
  if (dims == 0) {
    return {{}};
  }
  std::vector<std::vector<int>> result;
  for (auto& elem : SplineIterator(dims - 1, order)) {
    for (int i = 0; i < order + 1; i++) {
      std::vector<int> next_elem = elem;
      next_elem.push_back(i);
      result.push_back(std::move(next_elem));
    }
  }
  return result;
}

int SplinePos(int dims, int order, const std::vector<int>& orders) {
  assert(orders.size() == dims);
  int result = 0;
  for (int i = 0; i < dims; i++) {
    result += orders[i] * PowInt(order + 1, i);
  }
  return result;
}

std::vector<std::vector<int>> Hypercube(int dims) {
  return SplineIterator(dims, 1);
}

std::vector<int> DerivativeAt(int dims, int order, std::vector<int> spline,
                              std::vector<int> derivative,
                              std::vector<int> point) {
  for (auto orders : SplineIterator(dims, order)) {
    int pos = SplinePos(dims, order, orders);
    int factor = 1;
    for (auto der : derivative) {
      if (orders[der] >= 0) {
        factor *= orders[der];
        orders[der] -= 1;
      }
    }
    for (int dim = 0; dim < dims; dim++) {
      if (point[dim] == 0 && orders[dim] > 0) {
        factor = 0;
      }
    }
    spline[pos] *= factor;
  }
  return spline;
}

std::vector<std::vector<int>> Choices(int n, int k,
                                      std::vector<int> rest = {}) {
  if (n == k) {
    for (int i = n - 1; i >= 0; i--) {
      rest.push_back(i);
    }
    return {rest};
  }
  if (k == 0) {
    return {rest};
  }
  auto no_n = Choices(n - 1, k, rest);
  rest.push_back(n - 1);
  auto yes_n = Choices(n - 1, k - 1, rest);
  no_n.insert(no_n.end(), yes_n.begin(), yes_n.end());
  return no_n;
}

struct SortedIntVectorComparator {
  bool operator()(const std::vector<int>& lhs,
                  const std::vector<int>& rhs) const {
    if (lhs.size() != rhs.size()) return lhs.size() < rhs.size();
    for (int i = 0; i < lhs.size(); i++) {
      if (lhs[i] != rhs[i]) return lhs[i] < rhs[i];
    }
    return false;
  }
};

std::vector<std::vector<int>> Derivatives(int dims, int order) {
  if (order == 1) {
    return {{}};
  }
  int repeat = (order - 1) / 2;
  std::set<std::vector<int>, SortedIntVectorComparator> result = {{}};
  std::vector<std::vector<int>> once;
  for (int i = 0; i < dims + 1; i++) {
    for (auto& elem : Choices(dims, i)) {
      once.push_back(elem);
    }
  }
  for (int i = 0; i < repeat; i++) {
    std::set<std::vector<int>, SortedIntVectorComparator> new_result;
    for (auto& elem1 : once) {
      for (auto elem2 : result) {
        elem2.insert(elem2.end(), elem1.begin(), elem1.end());
        std::sort(elem2.begin(), elem2.end());
        new_result.insert(elem2);
      }
    }
    result = new_result;
  }
  std::vector<std::vector<int>> result_vec;
  result_vec.insert(result_vec.end(), result.begin(), result.end());
  return result_vec;
}

int RatGCD(int a, int b) {
  while (b != 0) {
    int tmp = b;
    b = a % b;
    a = tmp;
  }
  return a;
}

struct Rational {
  int num, den;
};

Rational RatNorm(Rational a) {
  int g = RatGCD(a.num, a.den);
  a.num /= g;
  a.den /= g;
  if (a.num == 0) a.den = 1;
  if (a.den < 0) return {-a.num, -a.den};
  return a;
}

Rational RatAddMul(Rational a, Rational b, Rational c) {
  a = RatNorm(a);
  b = RatNorm(b);
  c = RatNorm(c);
  Rational d = {b.num * c.num, b.den * c.den};
  d = RatNorm(d);
  Rational e = {a.num * d.den + d.num * a.den, a.den * d.den};
  return RatNorm(e);
}

// two elements, first num, then den
// Almost completely NR 2.1, but on integer rationals
std::vector<std::vector<int>> GaussJ(std::vector<int> A_in, int N) {
  std::vector<Rational> A(N * N, {1, 1});
  for (int i = 0; i < N * N; i++) {
    A[i].num = A_in[i];
  }
  std::vector<int> indxc(N, 0);
  std::vector<int> indxr(N, 0);
  std::vector<int> ipiv(N, 0);
  for (int i = 0; i < N; i++) {
    double big = 0;
    int irow, icol;
    for (int j = 0; j < N; j++) {
      if (ipiv[j] != 1) {
        for (int k = 0; k < N; k++) {
          if (ipiv[k] == 0) {
            if (fabs(A[N * j + k].num * 1.0 / A[N * j + k].den) > big) {
              big = fabs(A[N * j + k].num * 1.0 / A[N * j + k].den);
              irow = j;
              icol = k;
            }
          }
        }
      }
    }
    assert(big > 0);
    ipiv[icol] += 1;
    if (irow != icol) {
      for (int l = 0; l < N; l++) {
        auto tmp = A[N * irow + l];
        A[N * irow + l] = A[N * icol + l];
        A[N * icol + l] = tmp;
      }
    }
    indxr[i] = irow;
    indxc[i] = icol;
    assert(A[N * icol + icol].num != 0);
    auto pivinv = A[N * icol + icol];
    pivinv = {pivinv.den, pivinv.num};
    A[N * icol + icol] = {1, 1};
    for (int l = 0; l < N; l++) {
      A[N * icol + l] = RatAddMul({0, 1}, A[N * icol + l], pivinv);
    }
    for (int ll = 0; ll < N; ll++) {
      if (ll != icol) {
        auto dum = A[N * ll + icol];
        dum.num *= -1;
        A[N * ll + icol] = {0, 1};
        for (int l = 0; l < N; l++) {
          A[N * ll + l] = RatAddMul(A[N * ll + l], A[N * icol + l], dum);
        }
      }
    }
  }
  for (int ll = 0; ll < N; ll++) {
    int l = N - 1 - ll;
    if (indxr[l] != indxc[l]) {
      for (int k = 0; k < N; k++) {
        auto tmp = A[N * k + indxr[l]];
        A[N * k + indxr[l]] = A[N * k + indxc[l]];
        A[N * k + indxc[l]] = tmp;
      }
    }
  }
  std::vector<int> num(N * N), den(N * N);
  for (int i = 0; i < N * N; i++) {
    num[i] = A[i].num;
    den[i] = A[i].den;
  }
  return {num, den};
}

void DebugPrint(std::vector<int> data, const char* name) {
  printf("[%s:", name);
  for (auto i : data) {
    printf(" %d", i);
  }
  printf("]\n");
}

}  // namespace

std::vector<std::vector<std::vector<int>>> GetSplineFittingMatrix(
    int order, int dims, std::vector<std::vector<int>> derivatives) {
  std::vector<std::vector<int>> num, den, ordering;
  std::vector<int> coeffs;
  int N;
  std::map<std::vector<int>, int, SortedIntVectorComparator> input_derivatives;
  std::set<std::vector<int>, SortedIntVectorComparator> output_derivatives;
  for (auto& der : Derivatives(dims, order)) {
    // DebugPrint(der, "der");
    output_derivatives.insert(der);
  }
  for (int i = 0; i < derivatives.size(); i++) {
    std::sort(derivatives[i].begin(), derivatives[i].end());
    input_derivatives[derivatives[i]] = i;
    // DebugPrint(derivatives[i], "ind");
    assert(output_derivatives.count(derivatives[i]) == 1);
  }
  assert(input_derivatives.size() == derivatives.size());

  for (auto& point : Hypercube(dims)) {
    for (auto& der : Derivatives(dims, order)) {
      auto spline =
          DerivativeAt(dims, order, SplineNew(dims, order), der, point);
      N = spline.size();
      coeffs.insert(coeffs.end(), spline.begin(), spline.end());

      if (input_derivatives.count(der) == 1) {
        std::vector<int> order_entry;
        order_entry.insert(order_entry.end(), point.begin(), point.end());
        order_entry.push_back(input_derivatives[der]);
        ordering.push_back(order_entry);
      }
    }
  }
  auto inverted = GaussJ(coeffs, N);
  for (int i = 0; i < N; i++) {
    int pos = 0;
    std::vector<int> line_num, line_den;
    for (auto& point : Hypercube(dims)) {
      for (auto& der : Derivatives(dims, order)) {
        if (input_derivatives.count(der) == 1) {
          line_num.push_back(inverted[0][N * i + pos]);
          line_den.push_back(inverted[1][N * i + pos]);
        }
        pos += 1;
      }
    }
    num.push_back(line_num);
    den.push_back(line_den);
  }
  return {num, den, ordering};
}

}  // namespace gen

}  // namespace potc
