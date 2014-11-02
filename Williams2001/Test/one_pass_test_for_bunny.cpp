#include <QtCore/QFileInfo>

#include <pcl/io/ply_io.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_plotter.h>
//#include <math.h>

#include "../include/pclbase.h"
#include "../include/qtbase.h"
#include "../include/cloud.h"
#include "../include/cloudmanager.h"
#include "../include/cloudio.h"
#include "../include/pairwiseregistration.h"
#include "../include/globalregistration.h"
#include "../include/mathutilities.h"

#include "SRoMCPS.h"

int main(int argc, char **argv)
{
	CloudManager* cloudManager = new CloudManager();
	PairwiseRegistrationManager* pairwiseRegistrationManager = new PairwiseRegistrationManager();
	RegistrationDataManager* registrationDataManager = new RegistrationDataManager();
	//CycleRegistrationManager *cycleRegistrationManager = new CycleRegistrationManager();
	
  	std::vector<int> p_file_indices_ply = pcl::console::parse_file_extension_argument (argc, argv, ".ply");	
	for (int i = 0; i < p_file_indices_ply.size(); ++i)
	{
		CloudDataPtr cloudData(new CloudData);
		Eigen::Matrix4f transformation;
		BoundariesPtr boundaries(new Boundaries);

		QString fileName = QString(argv[p_file_indices_ply[i]]);
		CloudIO::importCloudData(fileName, cloudData);
		CloudIO::importTransformation(fileName, transformation);
		CloudIO::importBoundaries(fileName, boundaries);
		Cloud* cloud = cloudManager->addCloud(cloudData, Cloud::fromIO, fileName, transformation);
		cloud->setBoundaries(boundaries);
	}

	std::vector< std::vector<int> > overlapRelations( cloudManager->getAllClouds().size() );
	
	// overlapRelations[0].push_back(1);
	// overlapRelations[1].push_back(2);
	// overlapRelations[2].push_back(0);

	overlapRelations[0].push_back(1);
	overlapRelations[0].push_back(2);
	overlapRelations[0].push_back(5);
	overlapRelations[0].push_back(6);
	overlapRelations[0].push_back(9);

	overlapRelations[1].push_back(0);
	overlapRelations[1].push_back(2);	
	overlapRelations[1].push_back(5);
	overlapRelations[1].push_back(6);
	overlapRelations[1].push_back(8);	
	overlapRelations[1].push_back(9);

	overlapRelations[2].push_back(0);
	overlapRelations[2].push_back(1);	
	overlapRelations[2].push_back(3);
	overlapRelations[2].push_back(7);
	overlapRelations[2].push_back(8);	
	overlapRelations[2].push_back(9);

	overlapRelations[3].push_back(2);
	overlapRelations[3].push_back(4);	
	overlapRelations[3].push_back(7);
	overlapRelations[3].push_back(8);	
	overlapRelations[3].push_back(9);	

	overlapRelations[4].push_back(3);
	overlapRelations[4].push_back(5);	
	overlapRelations[4].push_back(6);

	overlapRelations[5].push_back(0);
	overlapRelations[5].push_back(1);	
	overlapRelations[5].push_back(4);
	overlapRelations[5].push_back(6);
	overlapRelations[5].push_back(9);	

	overlapRelations[6].push_back(0);
	overlapRelations[6].push_back(1);	
	overlapRelations[6].push_back(4);
	overlapRelations[6].push_back(5);

	overlapRelations[7].push_back(2);
	overlapRelations[7].push_back(3);	
	overlapRelations[7].push_back(8);
	overlapRelations[7].push_back(9);

	overlapRelations[8].push_back(1);
	overlapRelations[8].push_back(2);	
	overlapRelations[8].push_back(3);
	overlapRelations[8].push_back(7);
	overlapRelations[8].push_back(9);	

	overlapRelations[9].push_back(0);
	overlapRelations[9].push_back(1);	
	overlapRelations[9].push_back(2);
	overlapRelations[9].push_back(3);
	overlapRelations[9].push_back(5);	
	overlapRelations[9].push_back(7);
	overlapRelations[9].push_back(8);	

	for (int i = 0; i < overlapRelations.size(); ++i)
	{
		for (int j = 0; j < overlapRelations[i].size(); ++j)
		{
			QString cloudName_target = QString::number(i);
			QString cloudName_source = QString::number(overlapRelations[i][j]);

			QString prName = PairwiseRegistration::generateName(cloudName_target, cloudName_source);
			PairwiseRegistration *pairwiseRegistration = pairwiseRegistrationManager->getPairwiseRegistration(prName);
			QString reversePrName = PairwiseRegistration::generateName(cloudName_source, cloudName_target);
			PairwiseRegistration *reversePairwiseRegistration = pairwiseRegistrationManager->getPairwiseRegistration(reversePrName);

			std::cout << i << " " << overlapRelations[i][j] << std::endl;

			if (pairwiseRegistration == NULL && reversePairwiseRegistration == NULL)
			{
				RegistrationData *registrationData_target = registrationDataManager->getRegistrationData(cloudName_target);
				if ( registrationData_target == NULL)
				{
					Cloud *cloud_target = cloudManager->getCloud(cloudName_target);
					registrationData_target = registrationDataManager->addRegistrationData(cloud_target, cloudName_target);
				}
				RegistrationData *registrationData_source = registrationDataManager->getRegistrationData(cloudName_source);
				if ( registrationData_source == NULL)
				{
					Cloud *cloud_source = cloudManager->getCloud(cloudName_source);
					registrationData_source = registrationDataManager->addRegistrationData(cloud_source, cloudName_source);
				}
				pairwiseRegistration = pairwiseRegistrationManager->addPairwiseRegistration(registrationData_target, registrationData_source, prName);
			}		
		}
	}

	std::string output_directory = ".";
	pcl::console::parse_argument (argc, argv, "--directory", output_directory);

	ScanIndexPairs sipairs;
	std::vector<PointPairWithWeights> ppairwwss;
	int M = cloudManager->getAllClouds().size();
	Eigen::MatrixXd R(3, 3*M);
	Eigen::MatrixXd T(3*M, 1);

	QList<PairwiseRegistration*> pairwiseRegistrationList = pairwiseRegistrationManager->getAllPairwiseRegistrations();

	for (int iter = 0; iter < 1; ++iter)
	{
		sipairs.clear();
		ppairwwss.clear();

		for (int i = 0; i < pairwiseRegistrationList.size(); ++i)
		{
			CorrespondencesComputationParameters correspondencesComputationParameters;
			correspondencesComputationParameters.method = POINT_TO_PLANE;
			correspondencesComputationParameters.distanceThreshold = 0.1f;    // FOR bunny
			correspondencesComputationParameters.normalAngleThreshold = 45.0f;  // FOR bunny

			// correspondencesComputationParameters.distanceThreshold = 1.0f;       //FOR buste
			// correspondencesComputationParameters.normalAngleThreshold = 45.0f;     //FOR buste

			correspondencesComputationParameters.boundaryTest = true;
			correspondencesComputationParameters.biDirectional = true;

			CorrespondencesComputationData correspondencesComputationData;
			Correspondences correspondences;
			CorrespondenceIndices correspondenceIndices;
			int inverseStartIndex;
			PairwiseRegistration::preCorrespondences(pairwiseRegistrationList[i]->getTarget(), pairwiseRegistrationList[i]->getSource(), Eigen::Matrix4f::Identity(), 
				correspondencesComputationParameters, correspondences, correspondenceIndices, inverseStartIndex, correspondencesComputationData);

			float rmsError_total;
			std::vector<float> squareErrors_total;
			PairwiseRegistration::computeSquareErrors(correspondences, squareErrors_total, rmsError_total);

			int targetErrorCloudIndex = pairwiseRegistrationList[i]->getTarget()->objectName().toInt();
			int sourceErrorCloudIndex = pairwiseRegistrationList[i]->getSource()->objectName().toInt();

			sipairs.push_back(ScanIndexPair(targetErrorCloudIndex, sourceErrorCloudIndex));
			PointPairWithWeights ppairwws_ts;
			for (int j = 0; j < correspondences.size(); ++j)
			{
				PointPairWithWeight ppairww;
				Point targetPoint;
				targetPoint.x() = static_cast<Scalar>( correspondences[j].targetPoint.getVector3fMap().x() );
				targetPoint.y() = static_cast<Scalar>( correspondences[j].targetPoint.getVector3fMap().y() );
				targetPoint.z() = static_cast<Scalar>( correspondences[j].targetPoint.getVector3fMap().z() );
				Point sourcePoint;
				sourcePoint.x() = static_cast<Scalar>( correspondences[j].sourcePoint.getVector3fMap().x() );
				sourcePoint.y() = static_cast<Scalar>( correspondences[j].sourcePoint.getVector3fMap().y() );
				sourcePoint.z() = static_cast<Scalar>( correspondences[j].sourcePoint.getVector3fMap().z() );

				ppairww.ppair = PointPair(targetPoint, sourcePoint);
				ppairww.w = 1.0f;
				ppairwws_ts.push_back(ppairww);
			}
			ppairwwss.push_back(ppairwws_ts);		

			// QString targetFileName = pairwiseRegistrationList[i]->getTarget()->cloud->getFileName();
			// QString sourceFileName = pairwiseRegistrationList[i]->getSource()->cloud->getFileName();

			std::cout << targetErrorCloudIndex << "<-" << sourceErrorCloudIndex
				<< "\t" << rmsError_total << "\t" << squareErrors_total.size() << std::endl;
		}

		SRoMCPS sromcps(sipairs, ppairwwss, M);
		// R = sromcps.R;
		// T = sromcps.T;
		// for (int j = 0; j < M; ++j)
		// {
		// 	std::cout << sromcps.R.block<3,3>(0, 3*j) << std::endl;
		// }
		// for (int j = 0; j < M; ++j)
		// {
		// 	std::cout << sromcps.T.block<3,1>(3*j, 0) << std::endl;
		// }
		// Eigen::Matrix3d R0 = sromcps.R.block<3,3>(0, 0);
		// Eigen::Matrix3d R1 = sromcps.R.block<3,3>(0, 3);
		// Eigen::Matrix3d R2 = sromcps.R.block<3,3>(0, 6);
		// Eigen::Vector3d T0 = sromcps.T.block<3,1>(0, 0);
		// Eigen::Vector3d T1 = sromcps.T.block<3,1>(3, 0);
		// Eigen::Vector3d T2 = sromcps.T.block<3,1>(6, 0);

		// std::cout << R0.transpose() * R1 << std::endl;
		// std::cout << R0.transpose() * ( T1 - T0 ) << std::endl;

		// std::cout << R0.transpose() * R2 << std::endl;
		// std::cout << R0.transpose() * ( T2 - T0 ) << std::endl;


		Scalar sum_error_before = 0.0;
		for (int u = 0; u < ppairwwss.size(); ++u)
		{	
			for (int i = 0; i < ppairwwss[u].size(); ++i)
			{
				sum_error_before += ( ppairwwss[u][i].ppair.first - ppairwwss[u][i].ppair.second ).squaredNorm();
			}
		}
		std::cout << "sum_error_before: " << sum_error_before << std::endl;		

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

		R = sromcps.R;
		T = sromcps.T;						

		for (int i = 0; i < M; ++i)
		{
			QString cloudName = QString::number(i);
			Cloud* cloud = cloudManager->getCloud(cloudName);

			QString fileName = cloud->getFileName();
			Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
			transformation.block<3,3>(0, 0) = R.block<3,3>(0, 3*i);
			transformation.block<3,1>(0, 3) = T.block<3,1>(3*i, 0);
			Eigen::Matrix4f transformationf = transformation.cast<float>();	
			CloudIO::exportTransformation(fileName, transformationf);
		}	
	}
	return 0;
}

