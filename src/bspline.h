#ifndef __BSPLINE_H__
#define __BSPLINE_H__

#include "bspline.h"

#include <iostream>
#include<algorithm>

#include <vector>
#include <tuple>
#include <numeric>

#include <cmath>

#include <Eigen/Dense>

Eigen::RowVectorXd basisfun(
    const size_t &iv,
    const double &uv,
    const size_t &p,
    const Eigen::VectorXd &U) {
    Eigen::RowVectorXd B = Eigen::RowVectorXd::Zero(p+1);

    const size_t i = iv;
    const double u = uv;

    Eigen::VectorXd left = Eigen::VectorXd::Zero(p+1);
    Eigen::VectorXd right = Eigen::VectorXd::Zero(p+1);

    Eigen::RowVectorXd N = Eigen::RowVectorXd::Zero(p+1);
    N[0] = 1;

    for (size_t j = 0; j < p; j++) {
        left[j+1] = u - U[i-j];
        right[j+1] = U[i+1+j] - u;

        double saved = 0;

        for (size_t r = 0; r <= j; r++) {
            double temp = N[r] / (right[r+1] + left[j-r+1]);
            N[r] = saved + right[r+1]*temp;
            saved = left[j-r+1]*temp;
        }
        N[j+1] = saved;
    }

    B = N;

    return B;
}

Eigen::MatrixXd basisfun(
    const Eigen::VectorXd &iv,
    const Eigen::VectorXd &uv,
    const size_t &p,
    const Eigen::VectorXd &U) {
    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(uv.size(), p+1);
    for (size_t i = 0; i < uv.size(); i++) {
        auto result = basisfun(iv[i], uv[i], p, U);
        B.row(i) = basisfun(iv[i], uv[i], p, U);
    }
    return B;
}

size_t findspan(
    const size_t &n,
    const size_t &p,
    const double &u,
    const Eigen::VectorXd &U) {

    std::vector<double> data_vector(U.data(), U.data() + U.size());

    const auto value = u;

    if (value > data_vector.back() || value < data_vector.front()) {
        std::cerr << "Some value is outside the knot span" << std::endl;
    }

    if (u == U[n+1]) {
        return n;
    }

    auto iter = std::find_if(data_vector.rbegin(), data_vector.rend(), [value](double member) -> bool {
        return value >= member;
    });

    const size_t index = data_vector.size() - std::distance(data_vector.rbegin(), iter) - 1;

    return index;
}






std::tuple<Eigen::MatrixXd, Eigen::VectorXd> GlobalCurveInterp(
        const size_t &n,
        const size_t &p,
        const Eigen::RowVectorXd &x,
        const Eigen::RowVectorXd &y,
        const Eigen::RowVectorXd &z) {
    auto m = n+p+1;

    Eigen::VectorXd D = Eigen::VectorXd::Zero(n);

    for (size_t i = 1; i < n; i++) {
        D[i] = sqrt( pow((y[i]-y[i-1]), 2) + pow((x[i]-x[i-1]), 2) + pow((z[i]-z[i-1]), 2));
    }

    const double d = D.sum();

    Eigen::VectorXd uk = Eigen::VectorXd::Zero(n);

    for (size_t k = 1; k < n; k++) {
        uk[k] = uk[k-1] + D[k]/d;
    }


    Eigen::VectorXd U = Eigen::VectorXd::Zero(m);

    for (size_t l = m-p-1; l < m ; l++) {
        U[l] = 1;
    }

    for (size_t o = 1; o < n-p; o++){
        U[o+p] = (uk.segment(o, p).sum())/p;
    }

    Eigen::MatrixXd N1 = Eigen::MatrixXd::Zero(n, p+1);

    Eigen::VectorXd sp = Eigen::VectorXd::Zero(n);

    for (size_t r = 0; r < n; r++) {
        size_t span = findspan(n, p, uk[r], U);
        sp[r] = span;
        N1.row(r) = basisfun(span, uk[r], p, U);
    }

    Eigen::MatrixXd N = Eigen::MatrixXd::Zero(n, n);
    N(n-1, n-1) = 1;

    for (size_t s1 = 0; s1 < n-1; s1++) {
        for (size_t s2 = 0; s2 < p+1; s2++) {
            //N(s1,s2+sp(s1)-p)=N1(s1,s2);
            N(s1, s2+sp[s1]-p) = N1(s1, s2);
        }
    }

    Eigen::VectorXd Px = N.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(x.transpose());
    Eigen::VectorXd Py = N.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(y.transpose());
    Eigen::VectorXd Pz = N.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(z.transpose());

    Eigen::MatrixXd P(3, n);
    P.row(0) = Px.transpose();
    P.row(1) = Py.transpose();
    P.row(2) = Pz.transpose();

    return std::tuple<Eigen::MatrixXd, Eigen::VectorXd>(P,U);
}

Eigen::MatrixXd CurvePoint(
        const size_t &d,
        const Eigen::MatrixXd &c,
        const Eigen::VectorXd &k,
        const Eigen::RowVectorXd &u) {
    const size_t nu = u.size();
    const size_t mc = c.rows();
    const size_t nc = c.cols();

    Eigen::VectorXd s = Eigen::VectorXd::Zero(nu);
    Eigen::MatrixXd N = Eigen::MatrixXd::Zero(nu, d+1);

    for (size_t i = 0; i < nu; i++) {
        s[i] = findspan(nc-1, d, u[i], k);
    }

    N = basisfun(s, u, d, k);

    Eigen::VectorXd tmp1 = s - d*Eigen::VectorXd::Ones(nu); // + Eigen::VectorXd::Ones(nu);

    Eigen::MatrixXd p = Eigen::MatrixXd::Zero(mc, nu);

    for (size_t i = 0; i <= d; i++) {
        Eigen::MatrixXd c_slice = Eigen::MatrixXd::Zero(mc, nu);

        for (size_t j = 0; j < nu; j++) {
            c_slice.col(j) = c.col(tmp1[j] + i);
        }

        //Eigen::RowVectorXd N_slice = Eigen::RowVectorXd(nu);
        const Eigen::RowVectorXd N_slice = N.col(i).transpose();

        Eigen::MatrixXd N_rep = Eigen::MatrixXd::Zero(mc, nu);
        for (size_t j = 0; j < mc; j++) {
            N_rep.row(j) = N_slice;
        }

        p += N_rep.cwiseProduct(c_slice);
    }

    return p;
}


#endif
