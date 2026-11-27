// Standalone diagnostic linked to the INSTALLED DART; no native library edits.
// This prescribed SPD problem is not the recorded robot's actual LCP.
#include <dart/constraint/DantzigBoxedLcpSolver.hpp>
#include <dart/constraint/PgsBoxedLcpSolver.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace {
constexpr int N = 3;
constexpr int Stride = 4;  // DART's ODE dPAD(3), confirmed against installed API.
using Three = std::array<double, N>;

void number(double value)
{
  if (std::isfinite(value)) std::cout << value;
  else std::cout << "null";
}

template<class T> void array(const T& values)
{
  std::cout << '[';
  bool first = true;
  for (const auto value : values) {
    if (!first) std::cout << ',';
    number(static_cast<double>(value));
    first = false;
  }
  std::cout << ']';
}

void run(dart::constraint::BoxedLcpSolver& solver,
         const std::string& label, const bool early)
{
  // A positive definite matrix: leading principal minors 1, 1.75, 3.5.
  const std::array<double, 9> originalA{1., 1.5, 0., 1.5, 4., 0., 0., 0., 2.};
  const Three originalB{1., 4., 0.};
  const Three originalLo{0., -.7, -.7};
  const Three originalHi{std::numeric_limits<double>::infinity(), .7, .7};
  const std::array<int, N> originalIndex{-1, 0, 0};
  std::array<double, N*Stride> a{};
  for (int i=0; i<N; ++i)
    for (int j=0; j<N; ++j) a[i*Stride+j] = originalA[i*N+j];
  auto b = originalB, lo = originalLo, hi = originalHi;
  auto findex = originalIndex;
  Three x{0., 0., 0.};
  const bool success = solver.solve(N, a.data(), x.data(), b.data(), 0,
                                    lo.data(), hi.data(), findex.data(), early);
  Three w{}, effectiveLo{}, effectiveHi{}, violation{};
  for (int i=0; i<N; ++i) {
    w[i] = -originalB[i];
    for (int j=0; j<N; ++j) w[i] += originalA[i*N+j]*x[j];
    if (originalIndex[i] >= 0) {
      // The initial coefficients are signed +/-mu; use final normal magnitude.
      const double normal = std::abs(x[originalIndex[i]]);
      effectiveLo[i] = originalLo[i]*normal;
      effectiveHi[i] = originalHi[i]*normal;
    } else {
      effectiveLo[i] = originalLo[i]; effectiveHi[i] = originalHi[i];
    }
    violation[i] = std::max({0., effectiveLo[i]-x[i], x[i]-effectiveHi[i]});
  }
  std::cout << "{\"label\":\"" << label << "\",\"solver\":\"" << solver.getType()
            << "\",\"n\":3,\"stride\":4,\"nub\":0,\"early_termination\":"
            << (early ? "true" : "false") << ",\"success\":"
            << (success ? "true" : "false")
            << ",\"post_auxiliary_arrays_may_remain_permuted\":true,\"A_original_row_major\":";
  array(originalA); std::cout << ",\"b_original\":"; array(originalB);
  std::cout << ",\"lo_original\":"; array(originalLo);
  std::cout << ",\"hi_original_null_is_infinity\":"; array(originalHi);
  std::cout << ",\"findex_original\":"; array(originalIndex);
  std::cout << ",\"x\":"; array(x);
  std::cout << ",\"w_original_Ax_minus_b\":"; array(w);
  std::cout << ",\"lo_after\":"; array(lo);
  std::cout << ",\"hi_after_null_is_infinity\":"; array(hi);
  std::cout << ",\"findex_after\":"; array(findex);
  std::cout << ",\"effective_lo_from_final_normal\":"; array(effectiveLo);
  std::cout << ",\"effective_hi_from_final_normal_null_is_infinity\":"; array(effectiveHi);
  std::cout << ",\"final_dependent_bound_violation\":"; array(violation);
  std::cout << '}';
}
}  // namespace

int main()
{
  std::cout << std::setprecision(17);
  dart::constraint::DantzigBoxedLcpSolver dantzigTrue, dantzigFalse;
  dart::constraint::PgsBoxedLcpSolver pgs;
  std::cout << "{\"scope\":\"controlled prescribed SPD witness, not original scene\","
               "\"initial_x\":[0,0,0],\"runs\":[";
  run(dantzigTrue, "dantzig_early_true", true); std::cout << ',';
  run(dantzigFalse, "dantzig_early_false", false); std::cout << ',';
  run(pgs, "pgs_default_early_false", false);
  std::cout << "]}\n";
  return 0;
}
