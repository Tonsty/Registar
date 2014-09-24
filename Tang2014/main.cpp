#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/SVD>

#include "SRoMCPS.h"

int main(int argc, char**argv)
{
	ScanIndexPairs sipairs;
	sipairs.push_back(ScanIndexPair(0,1));

	std::vector<PointPairWithWeights> ppairwwss;
	PointPairWithWeights ppairwws;

	Eigen::AngleAxis<Scalar> angleAxis = Eigen::AngleAxis<Scalar>(0.5 * M_PI, Eigen::Matrix<Scalar, 3, 1>::UnitZ());
	Eigen::Matrix<Scalar, 3, 1> translation = Eigen::Matrix<Scalar, 3, 1>(10.0, 0.0, 0.0);

	PointPairWithWeight ppairww1;
	ppairww1.w = 1.0;
	ppairww1.ppair.first = Eigen::Matrix<Scalar, 3, 1>(-1.0, 0, 0);
	ppairww1.ppair.second = angleAxis.toRotationMatrix() * ppairww1.ppair.first + translation;
	ppairwws.push_back(ppairww1);

	PointPairWithWeight ppairww2;
	ppairww2.w = 1.0;
	ppairww2.ppair.first = Eigen::Matrix<Scalar, 3, 1>(0, 1.5, 0);
	ppairww2.ppair.second = angleAxis.toRotationMatrix() * ppairww2.ppair.first + translation;
	ppairwws.push_back(ppairww2);

	PointPairWithWeight ppairww3;
	ppairww3.w = 1.0;
	ppairww3.ppair.first = Eigen::Matrix<Scalar, 3, 1>(1.0, 0, 0);
	ppairww3.ppair.second = angleAxis.toRotationMatrix() * ppairww3.ppair.first + translation;
	ppairwws.push_back(ppairww3);


	PointPairWithWeight ppairww4;
	ppairww4.w = 1.0;
	ppairww4.ppair.first = Eigen::Matrix<Scalar, 3, 1>(0.0, 0.75, 1.0);
	ppairww4.ppair.second = angleAxis.toRotationMatrix() * ppairww4.ppair.first + translation;
	ppairwws.push_back(ppairww4);

	PointPairWithWeight ppairww5;
	ppairww5.w = 1.0;
	ppairww5.ppair.first = Eigen::Matrix<Scalar, 3, 1>(0.3, -0.5, 0.7);
	ppairww5.ppair.second = angleAxis.toRotationMatrix() * ppairww5.ppair.first + translation;
	ppairwws.push_back(ppairww5);	

	PointPairWithWeight ppairww6;
	ppairww6.w = 1.0;
	ppairww6.ppair.first = Eigen::Matrix<Scalar, 3, 1>(-0.4, -0.8, -0.6);
	ppairww6.ppair.second = angleAxis.toRotationMatrix() * ppairww6.ppair.first + translation;
	ppairwws.push_back(ppairww6);	

	ppairwwss.push_back(ppairwws);

	int M = 2;
	SRoMCPS sromcps(sipairs, ppairwwss, M);

	Scalar sum_error = 0.0;
	for (int u = 0; u < ppairwwss.size(); ++u)
	{
		Eigen::Matrix<Scalar, 3, 3> Ra = sromcps.R.block<3,3>(0, sipairs[u].first * 3);
		Eigen::Matrix<Scalar, 3, 3> Rb = sromcps.R.block<3,3>(0, sipairs[u].second * 3);

		Eigen::Matrix<Scalar, 3, 1> ta = sromcps.T.block<3,1>(sipairs[u].first * 3, 0);
		Eigen::Matrix<Scalar, 3, 1> tb = sromcps.T.block<3,1>(sipairs[u].second * 3, 0);	
		for (int i = 0; i < ppairwwss[u].size(); ++i)
		{
			sum_error += (Ra * ppairwwss[u][i].ppair.first + ta - Rb * ppairwwss[u][i].ppair.second - tb).squaredNorm();
		}
	}
	std::cout << "sum_error: " << sum_error << std::endl;

	Eigen::Matrix<Scalar, 3, 3> R1 = sromcps.R.block<3,3>(0,0);
	Eigen::Matrix<Scalar, 3, 3> R2 = sromcps.R.block<3,3>(0,3);

	Eigen::Matrix<Scalar, 3, 1> t1 = sromcps.T.block<3,1>(0,0);
	Eigen::Matrix<Scalar, 3, 1> t2 = sromcps.T.block<3,1>(3,0);	

	std::cout << Eigen::AngleAxis<Scalar>(R2.transpose() * R1).axis() << std::endl;
	std::cout << Eigen::AngleAxis<Scalar>(R2.transpose() * R1).angle() << std::endl;
	std::cout << R2.transpose() * (t1 - t2) << std::endl;


	return 0;
}


	// Eigen::Matrix<Scalar, 3, 3> R1 = sromcps.R.block<3,3>(0,0);
	// Eigen::Matrix<Scalar, 3, 3> R2 = sromcps.R.block<3,3>(0,3);

	// Eigen::Matrix<Scalar, 3, 1> t1 = sromcps.T.block<3,1>(0,0);
	// Eigen::Matrix<Scalar, 3, 1> t2 = sromcps.T.block<3,1>(3,0);	

	// std::cout << R1 << std::endl;
	// std::cout << R2 << std::endl;	

	// std::cout << t1 << std::endl;
	// std::cout << t2 << std::endl;

	// Eigen::AngleAxis<Scalar> angleAxis1(R1);
	// Eigen::AngleAxis<Scalar> angleAxis2(R2);

	// Eigen::Matrix<Scalar, 4, 4> T1 = Eigen::Matrix<Scalar, 4, 4>::Identity(), T2 = Eigen::Matrix<Scalar, 4, 4>::Identity();
	// T1.block<3,3>(0,0) = R1;
	// T2.block<3,3>(0,0) = R2;

	// T1.block<3,1>(0,3) = t1;
	// T2.block<3,1>(0,3) = t2;

	// Eigen::Matrix<Scalar, 3, 3> R21 = (T1.inverse() * T2).block<3, 3>(0, 0);
	// Eigen::Matrix<Scalar, 3, 1> t21 = (T1.inverse() * T2).block<3, 1>(0, 3);

	// Eigen::AngleAxis<Scalar> angleAxis21(R21);

	// std::cout << "axis: \n" << angleAxis21.axis() << std::endl;
	// std::cout << "angle: \n" << angleAxis21.angle() << std::endl;
	// std::cout << "translation: \n" << t21 << std::endl;


// struct Pair
// {
// 	int a;
// 	int b;
// };
// Pair S[] = { {1,2}, {2,3}, {3,4}, {4,1}, {1,5}, {2,5}, {3,5}, {4,5}, {1,6}, {2,6}, {3,6}, {4,6} };


// typedef double Scalar;
// typedef Eigen::Matrix<Scalar, 3, 1> Point;
// typedef Point SPoint;
// typedef Point TPoint;
// typedef std::pair<SPoint, TPoint> PointPair;
// typedef Scalar Weight;
// struct PointPairWithWeight
// {
// 	PointPair ppair;
// 	Weight w;
// };
// typedef std::vector<PointPairWithWeight, Eigen::aligned_allocator<PointPairWithWeight> > PointPairWithWeights;
// typedef unsigned int ScanIndex; 
// typedef std::pair<ScanIndex, ScanIndex> ScanIndexPair;
// typedef std::vector<ScanIndexPair> ScanIndexPairs;

// class SRoMCPS
// {
// public:

// 	SRoMCPS(ScanIndexPairs &_sipairs, std::vector<PointPairWithWeights> &_ppairwwss, int _M) : sipairs(_sipairs), ppairwwss(_ppairwwss), M(_M)
// 	{
// 		P = sipairs.size(); 

// 		createViewSelectionMatrices();

// 		createQ();

// 		solve_RT();
// 	}

// 	void createViewSelectionMatrices()
// 	{
// 		C_a = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>::Zero(3*M, 3*P);
// 		C_b = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>::Zero(3*M, 3*P);

// 		for (int u = 0; u < P; ++u)
// 		{
// 			C_a.block<3,3>( sipairs[u].first*3, u*3 ) = Eigen::Matrix<Scalar, 3, 3>::Identity();	
// 			C_b.block<3,3>( sipairs[u].second*3, u*3 ) = Eigen::Matrix<Scalar, 3, 3>::Identity();			
// 		}
// 	}

// 	void createQ()
// 	{
// 		createQ_R();
// 		createQ_RT();
// 		Q = Q_R + Q_RT;
// 	}

// 	void createQ_R()
// 	{
// 		Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Hxx = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>::Zero(3*P,3*P);
// 		Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Hyy = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>::Zero(3*P,3*P);
// 		Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Hxy = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>::Zero(3*P,3*P);
// 		Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Hyx = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>::Zero(3*P,3*P);

// 		for (int u = 0; u < P; ++u)
// 		{
// 			Eigen::Matrix<Scalar, 3, 3> Hxx_u = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>::Zero(3,3);
// 			Eigen::Matrix<Scalar, 3, 3> Hyy_u = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>::Zero(3,3);
// 			Eigen::Matrix<Scalar, 3, 3> Hxy_u = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>::Zero(3,3);
// 			Eigen::Matrix<Scalar, 3, 3> Hyx_u = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>::Zero(3,3);

// 			int N_u = ppairwwss[u].size();
// 			for (int i = 0; i < N_u; ++i)
// 			{
// 				Hxx_u += ppairwwss[u][i].w * ppairwwss[u][i].ppair.first * ppairwwss[u][i].ppair.first.transpose();
// 				Hyy_u += ppairwwss[u][i].w * ppairwwss[u][i].ppair.second * ppairwwss[u][i].ppair.second.transpose();
// 				Hxy_u += ppairwwss[u][i].w * ppairwwss[u][i].ppair.first * ppairwwss[u][i].ppair.second.transpose();
// 				Hyx_u += ppairwwss[u][i].w * ppairwwss[u][i].ppair.second * ppairwwss[u][i].ppair.first.transpose();
// 			}

// 			Hxx.block<3,3>(3*u, 3*u) = Hxx_u;
// 			Hyy.block<3,3>(3*u, 3*u) = Hyy_u;
// 			Hxy.block<3,3>(3*u, 3*u) = Hxy_u;
// 			Hyx.block<3,3>(3*u, 3*u) = Hyx_u;				
// 		}

// 		Q_R = C_a * Hxx * C_a.transpose() + C_b * Hyy * C_b.transpose() - C_a * Hxy * C_b.transpose() - C_b * Hyx * C_a.transpose();

// 	}

// 	void createQ_RT()
// 	{
// 		W = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>::Zero(3*P,3*P);
// 		std::vector<Weight> ws(P);

// 		for (int u = 0; u < P; ++u)
// 		{
// 			int N_u = ppairwwss[u].size();
// 			Weight w_u = 0.0;
// 			for (int i = 0; i < N_u; ++i) w_u += ppairwwss[u][i].w;
// 			Eigen::Matrix<Scalar, 3, 3> W_u = Eigen::Matrix<Scalar, 3, 3>::Identity() * w_u;
// 			W.block<3,3>(3*u, 3*u) = W_u;
// 			ws[u] = w_u;
// 		}

// 		Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> g = W * (C_a - C_b).transpose() * pseudo_inverse( (C_a - C_b) * W * (C_a - C_b).transpose(), 1e-6 ) * (C_a - C_b) * W;

// 		x_mean.resize(P);
// 		y_mean.resize(P);
// 		for (int u = 0; u < P; ++u)
// 		{
// 			Eigen::Matrix<Scalar, 3, 1> x_mean_u = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>::Zero(3, 1);
// 			Eigen::Matrix<Scalar, 3, 1> y_mean_u = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>::Zero(3, 1);

// 			int N_u = ppairwwss[u].size();
// 			for (int i = 0; i < N_u; ++i)
// 			{
// 				x_mean_u += ppairwwss[u][i].ppair.first * ppairwwss[u][i].w;
// 				y_mean_u += ppairwwss[u][i].ppair.second * ppairwwss[u][i].w;
// 			}
// 			x_mean_u /= ws[u];
// 			y_mean_u /= ws[u];

// 			x_mean[u] = x_mean_u;
// 			y_mean[u] = y_mean_u;
// 		}

// 		Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Gxx(3*P,3*P), Gxy(3*P,3*P), Gyx(3*P,3*P), Gyy(3*P,3*P);

// 		for (int u = 0; u < P; ++u)
// 		{
// 			for (int h = 0; h < P; ++h)
// 			{
// 				Eigen::Matrix<Scalar, 3, 3> Gxx_uh = g.block<3,3>(3*u,3*h).diagonal().mean() * x_mean[u] * x_mean[h].transpose();
// 				Eigen::Matrix<Scalar, 3, 3> Gxy_uh = g.block<3,3>(3*u,3*h).diagonal().mean() * x_mean[u] * y_mean[h].transpose();
// 				Eigen::Matrix<Scalar, 3, 3> Gyx_uh = g.block<3,3>(3*u,3*h).diagonal().mean() * y_mean[u] * x_mean[h].transpose();
// 				Eigen::Matrix<Scalar, 3, 3> Gyy_uh = g.block<3,3>(3*u,3*h).diagonal().mean() * y_mean[u] * y_mean[h].transpose();

// 				Gxx.block<3,3>(3*u, 3*h) = Gxx_uh;
// 				Gxy.block<3,3>(3*u, 3*h) = Gxy_uh;
// 				Gyx.block<3,3>(3*u, 3*h) = Gyx_uh;
// 				Gyy.block<3,3>(3*u, 3*h) = Gyy_uh;				
// 			}
// 		}

// 		Q_RT = -C_a * Gxx * C_a.transpose() + C_a * Gxy * C_b.transpose() + C_b * Gyx * C_a.transpose() - C_b * Gyy * C_b.transpose();
// 	}

// 	Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> pseudo_inverse(Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> squareMatrix, const Scalar pinvtoler = Eigen::NumTraits<Scalar>::epsilon())
// 	{
// 		Eigen::JacobiSVD< Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> > svd(squareMatrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
// 		Eigen::JacobiSVD< Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> >::SingularValuesType singleValues_inv = svd.singularValues();	

// 		for (int i = 0; i < squareMatrix.cols(); ++i)
// 		{
// 			if (singleValues_inv(i) > pinvtoler) singleValues_inv(i) = 1.0/singleValues_inv(i);
// 			else singleValues_inv(i) = 0.0;
// 		}

// 		return svd.matrixV() * singleValues_inv.asDiagonal() * svd.matrixU().transpose(); 
// 	}

// 	void solve_RT()
// 	{
// 		solve_R();
// 		solve_T();
// 	}

// 	void solve_R() 
// 	{
// 		Eigen::Matrix<Scalar, 3, Eigen::Dynamic> R_initial;
// 		R_initial.resize(Eigen::NoChange, 3*M); 
// 		for (int j = 0; j < M; ++j) R_initial.block<3,3>(0, 3*j) = Eigen::Matrix<Scalar, 3, 3>::Identity(); // R_initial could be set by the uniform angle refine method

// 		Eigen::Matrix<Scalar, 3, Eigen::Dynamic> R_current = R_initial;
// 		const int max_iter = 1000;
// 		for (int iter = 0; iter < max_iter; ++iter)
// 		{
// 			for (int j = 0; j < M; ++j)
// 			{
// 				Eigen::Matrix<Scalar, 3, 3> Sj;
// 				if (j == 0)
// 				{
// 					//Sj = Q.block<3, 3*(M-1-j)>(j*3, (j+1)*3) * R_current.block<3, 3*(M-j-1)>(0, (j+1)*3).transpose();
// 					Sj = Q.block(j*3, (j+1)*3, 3, 3*(M-1-j)) * R_current.block(0, (j+1)*3, 3, 3*(M-j-1)).transpose();					
// 				}
// 				else if (j == M-1)
// 				{
// 					// Sj = Q.block<3, 3*j>(j*3, 0) * R_current.block<3, 3*j>(0, 0).transpose() + Q.block<3, 3*(M-1-j)>(j*3, (j+1)*3) * R_current.block<3, 3*(M-j-1)>(0, (j+1)*3).transpose();
// 					Sj = Q.block(j*3, 0, 3, 3*j) * R_current.block(0, 0, 3, 3*j).transpose() + Q.block(j*3, (j+1)*3, 3, 3*(M-1-j)) * R_current.block(0, (j+1)*3, 3, 3*(M-j-1)).transpose();
// 				}
// 				else
// 				{
// 					//Sj = Q.block<3, 3*j>(j*3, 0) * R_current.block<3, 3*j>(0, 0).transpose();
// 					Sj = Q.block(j*3, 0, 3, 3*j) * R_current.block(0, 0, 3, 3*j).transpose();
// 				}
// 				Eigen::JacobiSVD< Eigen::Matrix<Scalar, 3, 3> > svd(-Sj, Eigen::ComputeFullU | Eigen::ComputeFullV);
// 				Eigen::Matrix<Scalar, 3, 3> Rj = svd.matrixV() * svd.matrixU().transpose();
// 				R_current.block<3,3>(0, j*3) = Rj;
// 			}
// 		}
// 		R = R_current;
// 	}

// 	void solve_T()
// 	{
// 		Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> A = (C_a - C_b) * W * (C_a - C_b).transpose();

// 		Eigen::Matrix<Scalar, Eigen::Dynamic, 1> Z;
// 		Z.resize(3*P, 1);
// 		for (int u = 0; u < P; ++u)	
// 		{
// 			Z.block<3,1>(u*3,0) = R.block<3,3>(0, sipairs[u].first * 3) * x_mean[u] - R.block<3,3>(0, sipairs[u].second * 3) * y_mean[u];
// 		}
// 		Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> B = (C_a - C_b) * W * Z ;

// 		T = -pseudo_inverse(A, 1e-6) * B;
// 	}

// 	ScanIndexPairs sipairs;
// 	std::vector<PointPairWithWeights> ppairwwss;

// 	int M;
// 	Eigen::Matrix<Scalar, 3, Eigen::Dynamic> R;
// 	Eigen::Matrix<Scalar, Eigen::Dynamic, 1> T;

// 	int P;
// 	Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> C_a;
// 	Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> C_b;

// 	Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Q;
// 	Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Q_R;
// 	Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Q_RT;

// 	Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> W;

// 	std::vector< Eigen::Matrix<Scalar, 3, 1> > x_mean, y_mean;


// };