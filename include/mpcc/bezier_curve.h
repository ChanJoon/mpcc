#ifndef BEZIER_CURVE_H
#define BEZIER_CURVE_H

#include <Eigen/Dense>
#include <cmath>
#include <vector>

using namespace Eigen;

static constexpr int N = 20;
static constexpr double TIME_STEP = 0.1;

class BEZIER {
	public:
		int traj_order = 5;
		Matrix<double, 3, 6> points;
		Matrix<double, 3, N> bezier_points;
		Matrix<double, 3, N> bezier_velocity;
		VectorXd C;
		VectorXd C_v;
		VectorXd C_a;
		VectorXd C_j;


		void binCoeff();
		void bernstein();
		void calculateBezierCurve(Matrix<double, 3, 6> points);
		void calculateDerivative();
		void getPos(Ref<Matrix<double, 3, N>> return_input);
		void getVel(Ref<Matrix<double, 3, N>> return_input);

		BEZIER() {
			binCoeff();
		}

};

void BEZIER::binCoeff() {
	C.resize  (traj_order + 1);
	C_v.resize(traj_order    );
	
	if(traj_order > 1)
		C_a.resize(traj_order - 1);

	if(traj_order > 2)
		C_j.resize(traj_order - 2);
		
	const static auto factorial = [](int n)
	{
        int fact = 1;

        for(int i = n; i > 0 ; i--)
          fact *= i;

        return fact; 
	};

	const static auto combinatorial = [](int n, int k)
	{
		return factorial(n) / (factorial(k) * factorial(n - k));
	};

	for(int k = 0; k <= traj_order; k ++ )
	{
		C(k)   = combinatorial(traj_order, k);
		
		if( k <= (traj_order - 1) )
			C_v(k) = combinatorial(traj_order - 1, k);
		if( k <= (traj_order - 2) )
			C_a(k) = combinatorial(traj_order - 2, k);
		if( k <= (traj_order - 3) )
			C_j(k) = combinatorial(traj_order - 3, k);
	}
}

void BEZIER::calculateBezierCurve(Matrix<double, 3, 6> points_) {
	points = points_;
	MatrixXd bernstein_basis(traj_order + 1, 1);

	for(double i = 0; i < N; ++i) {
		double t = i / N;	// [0, 1]

		for(int j = 0; j <= traj_order; ++j) {
			bernstein_basis(j) = C(j) * std::pow(t, j) * std::pow(1 - t, traj_order - j);
		}

		bezier_points.col(i) = points_ * bernstein_basis;
	}
	calculateDerivative();
}

void BEZIER::calculateDerivative() {
	Matrix<double, 3, 5> delta;
	MatrixXd bernstein_basis(traj_order, 1);

	for (int i = 0; i < traj_order; ++i) {
		delta.col(i) = points.col(i + 1) - points.col(i);
	}

	for(double i = 0; i < N; ++i) {
		double t = i / N;  // [0, 1]

		for(int j = 0; j < traj_order; ++j) {  // Degree is now (n-1)
			bernstein_basis(j) = C_v(j) * std::pow(t, j) * std::pow(1 - t, traj_order - j - 1);
		}

		bezier_velocity.col(i) = traj_order * (delta * bernstein_basis);
	}
	// std::cout << bezier_velocity << "\n";
}


void BEZIER::getPos(Ref<Matrix<double, 3, N>> return_input) { return_input = bezier_points; }
void BEZIER::getVel(Ref<Matrix<double, 3, N>> return_input) { return_input = bezier_velocity; }

#endif