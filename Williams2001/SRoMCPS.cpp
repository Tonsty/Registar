#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/SVD>

#include "SRoMCPS.h"

namespace Williams2001
{ 
	SRoMCPS::SRoMCPS(ScanIndexPairs &_sipairs, std::vector<PointPairWithWeights> &_ppairwwss, int _M) : sipairs(_sipairs), ppairwwss(_ppairwwss), M(_M)
	{
		P = sipairs.size(); 
		createViewSelectionMatrices();

		// std::cout << "C_a = \n" << C_a << std::endl;
		// std::cout << "C_b = \n" << C_b << std::endl;

		create_W_ws();
		// std::cout << "W = \n" << W << std::endl;
		// for (int i = 0; i < ws.size(); ++i)
		// {
		// 	std::cout << "ws[" << i << "] = " << ws[i] << std::endl;
		// }
		create_xymean();
		// for (int i = 0; i < x_mean.size(); ++i)
		// {
		// 	std::cout << "x_mean[" << i << "] = \n" << x_mean[i] << std::endl;
		// }
		// for (int i = 0; i < y_mean.size(); ++i)
		// {
		// 	std::cout << "y_mean[" << i << "] = \n" << y_mean[i] << std::endl;
		// }

		createQ();
		//std::cout << "Q = \n" << Q_RT << std::endl;	
		solve_RT();
	}

	void SRoMCPS::create_xymean()
	{
		x_mean.resize(P);
		y_mean.resize(P);
		for (int u = 0; u < P; ++u)
		{
			Eigen::Matrix<Scalar, 3, 1> x_mean_u = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>::Zero(3, 1);
			Eigen::Matrix<Scalar, 3, 1> y_mean_u = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>::Zero(3, 1);

			int N_u = ppairwwss[u].size();
			for (int i = 0; i < N_u; ++i)
			{
				x_mean_u += ppairwwss[u][i].ppair.first * ppairwwss[u][i].w;
				y_mean_u += ppairwwss[u][i].ppair.second * ppairwwss[u][i].w;
			}
			x_mean_u /= ws[u];
			y_mean_u /= ws[u];

			x_mean[u] = x_mean_u;
			y_mean[u] = y_mean_u;
		}	
	}


	void SRoMCPS::create_W_ws()
	{
		W = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>::Zero(3*P,3*P);
		ws.resize(P);
		for (int u = 0; u < P; ++u)
		{
			int N_u = ppairwwss[u].size();
			Weight w_u = 0.0;
			for (int i = 0; i < N_u; ++i) w_u += ppairwwss[u][i].w;
			Eigen::Matrix<Scalar, 3, 3> W_u = Eigen::Matrix<Scalar, 3, 3>::Identity() * w_u;
			W.block<3,3>(3*u, 3*u) = W_u;
			ws[u] = w_u;
		}
	}

	void SRoMCPS::createViewSelectionMatrices()
	{
		C_a = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>::Zero(3*M, 3*P);
		C_b = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>::Zero(3*M, 3*P);

		for (int u = 0; u < P; ++u)
		{
			C_a.block<3,3>( sipairs[u].first*3, u*3 ) = Eigen::Matrix<Scalar, 3, 3>::Identity();	
			C_b.block<3,3>( sipairs[u].second*3, u*3 ) = Eigen::Matrix<Scalar, 3, 3>::Identity();			
		}
	}

	void SRoMCPS::createQ()
	{
		createQ_R();
		//std::cout << "Q_R = \n" << Q_R << std::endl;
		createQ_RT();
		//std::cout << "Q_RT = \n" << Q_RT << std::endl;	
		Q = Q_R + Q_RT;
		// std::cout << "Q = \n" << Q << std::endl;
	}

	void SRoMCPS::createQ_R()
	{
		Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Hxx = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>::Zero(3*P,3*P);
		Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Hyy = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>::Zero(3*P,3*P);
		Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Hxy = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>::Zero(3*P,3*P);
		Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Hyx = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>::Zero(3*P,3*P);

		for (int u = 0; u < P; ++u)
		{
			Eigen::Matrix<Scalar, 3, 3> Hxx_u = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>::Zero(3,3);
			Eigen::Matrix<Scalar, 3, 3> Hyy_u = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>::Zero(3,3);
			Eigen::Matrix<Scalar, 3, 3> Hxy_u = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>::Zero(3,3);
			Eigen::Matrix<Scalar, 3, 3> Hyx_u = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>::Zero(3,3);

			int N_u = ppairwwss[u].size();
			for (int i = 0; i < N_u; ++i)
			{
				Hxx_u += ppairwwss[u][i].w * ppairwwss[u][i].ppair.first * ppairwwss[u][i].ppair.first.transpose();
				Hyy_u += ppairwwss[u][i].w * ppairwwss[u][i].ppair.second * ppairwwss[u][i].ppair.second.transpose();
				Hxy_u += ppairwwss[u][i].w * ppairwwss[u][i].ppair.first * ppairwwss[u][i].ppair.second.transpose();
				Hyx_u += ppairwwss[u][i].w * ppairwwss[u][i].ppair.second * ppairwwss[u][i].ppair.first.transpose();
			}

			Hxx.block<3,3>(3*u, 3*u) = Hxx_u;
			Hyy.block<3,3>(3*u, 3*u) = Hyy_u;
			Hxy.block<3,3>(3*u, 3*u) = Hxy_u;
			Hyx.block<3,3>(3*u, 3*u) = Hyx_u;				
		}

		Q_R = C_a * Hxx * C_a.transpose() + C_b * Hyy * C_b.transpose() - C_a * Hxy * C_b.transpose() - C_b * Hyx * C_a.transpose();

	}

	void SRoMCPS::createQ_RT()
	{

		Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> g = W * (C_a - C_b).transpose() * pseudo_inverse( (C_a - C_b) * W * (C_a - C_b).transpose() ) * (C_a - C_b) * W;

		// std::cout << "gamma = \n" << g << std::endl;

		Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Gxx(3*P,3*P), Gxy(3*P,3*P), Gyx(3*P,3*P), Gyy(3*P,3*P);

		for (int u = 0; u < P; ++u)
		{
			for (int h = 0; h < P; ++h)
			{
				Eigen::Matrix<Scalar, 3, 3> Gxx_uh = g.block<3,3>(3*u,3*h).diagonal().mean() * x_mean[u] * x_mean[h].transpose();
				Eigen::Matrix<Scalar, 3, 3> Gxy_uh = g.block<3,3>(3*u,3*h).diagonal().mean() * x_mean[u] * y_mean[h].transpose();
				Eigen::Matrix<Scalar, 3, 3> Gyx_uh = g.block<3,3>(3*u,3*h).diagonal().mean() * y_mean[u] * x_mean[h].transpose();
				Eigen::Matrix<Scalar, 3, 3> Gyy_uh = g.block<3,3>(3*u,3*h).diagonal().mean() * y_mean[u] * y_mean[h].transpose();
				
				Gxx.block<3,3>(3*u, 3*h) = Gxx_uh;
				Gxy.block<3,3>(3*u, 3*h) = Gxy_uh;
				Gyx.block<3,3>(3*u, 3*h) = Gyx_uh;
				Gyy.block<3,3>(3*u, 3*h) = Gyy_uh;				
			}
		}

		Q_RT = -C_a * Gxx * C_a.transpose() + C_a * Gxy * C_b.transpose() + C_b * Gyx * C_a.transpose() - C_b * Gyy * C_b.transpose();
	}


	Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> SRoMCPS::pseudo_inverse(Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> squareMatrix, const Scalar pinvtoler)
	{
		Eigen::JacobiSVD< Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> > svd(squareMatrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
		Eigen::JacobiSVD< Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> >::SingularValuesType singleValues_inv = svd.singularValues();	

		// std::cout << singleValues_inv.asDiagonal().toDenseMatrix() << std::endl;

		for (int i = 0; i < squareMatrix.cols(); ++i)
		{
			if (singleValues_inv(i) > pinvtoler) singleValues_inv(i) = 1.0/singleValues_inv(i);
			else singleValues_inv(i) = 0.0;
		}

		return svd.matrixV() * singleValues_inv.asDiagonal() * svd.matrixU().transpose(); 
	}

	void SRoMCPS::solve_RT()
	{
		solve_R();
		solve_T();
	}

	void SRoMCPS::solve_R() 
	{
		Eigen::Matrix<Scalar, 3, Eigen::Dynamic> R_initial;
		R_initial.resize(Eigen::NoChange, 3*M); 
		for (int j = 0; j < M; ++j) R_initial.block<3,3>(0, 3*j) = Eigen::Matrix<Scalar, 3, 3>::Identity(); // R_initial could be set by the uniform angle refine method

		Eigen::Matrix<Scalar, 3, Eigen::Dynamic> R_current = R_initial;
		const int max_iter = 10;
		for (int iter = 0; iter < max_iter; ++iter)
		{
			for (int j = 0; j < M; ++j)
			{
				Eigen::Matrix<Scalar, 3, 3> Sj;
				Eigen::Matrix<Scalar, 3, Eigen::Dynamic> R_L, R_U;
				Eigen::Matrix<Scalar, 3, Eigen::Dynamic> Q_jL, Q_jU;

				if (j == 0)
				{
					//Sj = Q.block<3, 3*(M-1-j)>(j*3, (j+1)*3) * R_current.block<3, 3*(M-j-1)>(0, (j+1)*3).transpose();
					//Sj = Q.block(j*3, (j+1)*3, 3, 3*(M-1-j)) * R_current.block(0, (j+1)*3, 3, 3*(M-j-1)).transpose();
					Q_jU = Q.block(j*3, (j+1)*3, 3, 3*(M-1-j));
					R_U = R_current.block(0, (j+1)*3, 3, 3*(M-j-1));
					Sj = Q_jU * R_U.transpose();					
				}
				else if (j == M-1)
				{
					//Sj = Q.block<3, 3*j>(j*3, 0) * R_current.block<3, 3*j>(0, 0).transpose();
					//Sj = Q.block(j*3, 0, 3, 3*j) * R_current.block(0, 0, 3, 3*j).transpose();
					Q_jL = Q.block(j*3, 0, 3, 3*j);	
					R_L = R_current.block(0, 0, 3, 3*j);			
					Sj = Q_jL * R_L.transpose();
				}
				else
				{
					// Sj = Q.block<3, 3*j>(j*3, 0) * R_current.block<3, 3*j>(0, 0).transpose() + Q.block<3, 3*(M-1-j)>(j*3, (j+1)*3) * R_current.block<3, 3*(M-j-1)>(0, (j+1)*3).transpose();
					//Sj = Q.block(j*3, 0, 3, 3*j) * R_current.block(0, 0, 3, 3*j).transpose() + Q.block(j*3, (j+1)*3, 3, 3*(M-1-j)) * R_current.block(0, (j+1)*3, 3, 3*(M-j-1)).transpose();
					R_L = R_current.block(0, 0, 3, 3*j);
					R_U = R_current.block(0, (j+1)*3, 3, 3*(M-j-1));
					Q_jL = Q.block(j*3, 0, 3, 3*j);
					Q_jU = Q.block(j*3, (j+1)*3, 3, 3*(M-1-j));				
					Sj = Q_jL * R_L.transpose() + Q_jU * R_U.transpose();
				}
				Eigen::JacobiSVD< Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> > svd(-Sj, Eigen::ComputeThinU | Eigen::ComputeThinV);
				Eigen::Matrix<Scalar, 3, 3> Rj = svd.matrixV() * svd.matrixU().transpose();
				R_current.block<3,3>(0, j*3) = Rj;

				// std::cout << "iter = \t" << iter << "\t j = \t" << j << "\t trace(RQR_t) = " << ( R_current * Q * R_current.transpose() ).trace() << std::endl;	
				if (Rj.determinant() < 0)
				{
					std::cout << "iter = \t" << iter << "\t j = \t" << j << "\t Rj determinant = " << Rj.determinant() << std::endl;
					exit(1);	
				}
			}
		}
		R = R_current;

		//R = R_initial;

		// std::cout << "trace(RQR_t) = " << ( R * Q * R.transpose() ).trace() << std::endl;
	}

	void SRoMCPS::solve_T()
	{
		Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> A = (C_a - C_b) * W * (C_a - C_b).transpose();

		Eigen::Matrix<Scalar, Eigen::Dynamic, 1> Z;
		Z.resize(3*P, 1);
		for (int u = 0; u < P; ++u)	
		{
			Z.block<3,1>(u*3,0) = R.block<3,3>(0, sipairs[u].first * 3) * x_mean[u] - R.block<3,3>(0, sipairs[u].second * 3) * y_mean[u];
		}
		Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> B = (C_a - C_b) * W * Z ;

		T = -pseudo_inverse(A) * B;
	}
}