#ifndef QPOASESHELPER_H
#define QPOASESHELPER_H

#include <qpOASES.hpp>
#include <matrix/math.hpp>

USING_NAMESPACE_QPOASES

namespace qp_helper {

// ===== 기본 변환 함수 =====

template<size_t M, size_t N>
inline void toQpArray(const matrix::Matrix<double, M, N>& mat, real_t* arr) {
    mat.copyTo(arr);
}

template<size_t N>
inline void toQpArray(const matrix::Vector<double, N>& vec, real_t* arr) {
    vec.copyTo(arr);
}

template<size_t N>
inline matrix::Vector<double, N> toVector(const real_t* arr) {
    return matrix::Vector<double, N>(arr);
}

// ===== 제약조건 A*x 바운드 (lbA, ubA) =====

// A*x <= b
template<size_t nC>
inline void setLeq(const matrix::Vector<double, nC>& b, real_t* lbA, real_t* ubA) {
    for (size_t i = 0; i < nC; i++) {
        lbA[i] = -INFTY;
        ubA[i] = b(i);
    }
}

// A*x >= b
template<size_t nC>
inline void setGeq(const matrix::Vector<double, nC>& b, real_t* lbA, real_t* ubA) {
    for (size_t i = 0; i < nC; i++) {
        lbA[i] = b(i);
        ubA[i] = INFTY;
    }
}

// A*x = b
template<size_t nC>
inline void setEq(const matrix::Vector<double, nC>& b, real_t* lbA, real_t* ubA) {
    for (size_t i = 0; i < nC; i++) {
        lbA[i] = b(i);
        ubA[i] = b(i);
    }
}

// b_low <= A*x <= b_up
template<size_t nC>
inline void setConstraintBounds(const matrix::Vector<double, nC>& b_low, 
                                const matrix::Vector<double, nC>& b_up,
                                real_t* lbA, real_t* ubA) {
    b_low.copyTo(lbA);
    b_up.copyTo(ubA);
}

// 스칼라 버전 (제약 1개)
inline void setLeq(double b, real_t* lbA, real_t* ubA) {
    lbA[0] = -INFTY;
    ubA[0] = b;
}

inline void setGeq(double b, real_t* lbA, real_t* ubA) {
    lbA[0] = b;
    ubA[0] = INFTY;
}

inline void setEq(double b, real_t* lbA, real_t* ubA) {
    lbA[0] = b;
    ubA[0] = b;
}

// ===== 변수 바운드 (lb, ub): -C1 <= x <= C2 =====

// -c1 <= x <= c2 (벡터)
template<size_t nV>
inline void setVariableBounds(const matrix::Vector<double, nV>& c1,
                              const matrix::Vector<double, nV>& c2,
                              real_t* lb, real_t* ub) {
    for (size_t i = 0; i < nV; i++) {
        lb[i] = -c1(i);
        ub[i] = c2(i);
    }
}

// lb <= x <= ub (직접 지정)
template<size_t nV>
inline void setVariableBoundsDirect(const matrix::Vector<double, nV>& lower,
                                    const matrix::Vector<double, nV>& upper,
                                    real_t* lb, real_t* ub) {
    lower.copyTo(lb);
    upper.copyTo(ub);
}

// 대칭: -c <= x <= c
template<size_t nV>
inline void setVariableBoundsSymmetric(const matrix::Vector<double, nV>& c,
                                       real_t* lb, real_t* ub) {
    for (size_t i = 0; i < nV; i++) {
        lb[i] = -c(i);
        ub[i] = c(i);
    }
}

// 모든 변수에 동일한 바운드: -c <= x_i <= c
template<size_t nV>
inline void setVariableBoundsUniform(double c, real_t* lb, real_t* ub) {
    for (size_t i = 0; i < nV; i++) {
        lb[i] = -c;
        ub[i] = c;
    }
}

// 바운드 없음 (무한대)
template<size_t nV>
inline void setVariableUnbounded(real_t* lb, real_t* ub) {
    for (size_t i = 0; i < nV; i++) {
        lb[i] = -INFTY;
        ub[i] = INFTY;
    }
}

} // namespace qp_helper

#endif