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

void generateErrorRGB(float error, 
						unsigned int &red, unsigned int &green, unsigned int &blue,
						float minerror = 0.0, float maxerror = 1.0);
						//float blueerror = 0, float qingerror = 0.25, float greenerror = 0.5, float yellowerror = 0.75, float rederror = 1 );

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

	std::vector<CloudDataPtr> sumErrorCloud(registrationDataManager->getAllRegistrationDatas().size());
	std::vector<CloudDataPtr> maxErrorCloud(registrationDataManager->getAllRegistrationDatas().size());

	QList<RegistrationData*> registrationDataList = registrationDataManager->getAllRegistrationDatas();
	for (int i = 0; i < registrationDataList.size(); ++i)
	{
		CloudDataPtr temp1(new CloudData);
		CloudDataPtr temp2(new CloudData);

		pcl::copyPointCloud(*(registrationDataList[i]->cloudData), *temp1);
		pcl::copyPointCloud(*(registrationDataList[i]->cloudData), *temp2);	

		QString dataName = registrationDataList[i]->objectName();

		sumErrorCloud[dataName.toInt()] = temp1;
		maxErrorCloud[dataName.toInt()] = temp2;
	}

	for (int i = 0; i < sumErrorCloud.size(); ++i)
	{
		for (int j = 0; j < sumErrorCloud[i]->size(); ++j)
		{
			// (*sumErrorCloud[i])[j].intensity = 0.0f;
			(*sumErrorCloud[i])[j].curvature = 0.0f;		
		}
	}

	for (int i = 0; i < maxErrorCloud.size(); ++i)
	{
		for (int j = 0; j < maxErrorCloud[i]->size(); ++j)
		{
			// (*maxErrorCloud[i])[j].intensity = 0.0f;
			(*maxErrorCloud[i])[j].curvature = 0.0f;
		}
	}

	std::string output_directory = ".";
	pcl::console::parse_argument (argc, argv, "--directory", output_directory);

	pcl::visualization::PCLPlotter plot;
	plot.setWindowSize( 800, 600 );
	plot.setBackgroundColor(1.0f, 1.0f, 1.0f);
	plot.setXTitle( "point pair index" );
	plot.setYTitle( "distance" );
	float max_error = 0;
	float max_index = 0;

	pcl::visualization::PCLPlotter plot1;
	plot1.setWindowSize( 800, 600 );
	plot1.setBackgroundColor(1.0f, 1.0f, 1.0f);
	plot1.setXTitle( "distance" );
	plot1.setYTitle( "number of point" );

	pcl::visualization::PCLPlotter plot2;
	plot2.setWindowSize( 800, 600 );
	plot2.setBackgroundColor(1.0f, 1.0f, 1.0f);
	plot2.setXTitle( "distance" );
	plot2.setYTitle( "point pair index");

	pcl::visualization::PCLPlotter plot3;
	plot3.setWindowSize( 800, 600 );
	plot3.setBackgroundColor(1.0f, 1.0f, 1.0f);
	plot3.setXTitle( "distance" );
	plot3.setYTitle( "point pair index");

	std::vector<double> total_errors;	

	QList<PairwiseRegistration*> pairwiseRegistrationList = pairwiseRegistrationManager->getAllPairwiseRegistrations();
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
			correspondencesComputationParameters, correspondences, 
			correspondenceIndices, inverseStartIndex, correspondencesComputationData);

		float rmsError_total;
		std::vector<float> squareErrors_total;
		PairwiseRegistration::computeSquareErrors(correspondences, squareErrors_total, rmsError_total);

		int targetErrorCloudIndex = pairwiseRegistrationList[i]->getTarget()->objectName().toInt();
		int sourceErrorCloudIndex = pairwiseRegistrationList[i]->getSource()->objectName().toInt();

		// QString targetFileName = pairwiseRegistrationList[i]->getTarget()->cloud->getFileName();
		// QString sourceFileName = pairwiseRegistrationList[i]->getSource()->cloud->getFileName();

		std::vector<double> errors(squareErrors_total.size());
		for(int j = 0;j < errors.size(); j++) errors[j] = (double)sqrtf(squareErrors_total[j]);

		std::vector<double> sorted_errors = errors;
		std::sort( sorted_errors.begin(), sorted_errors.end());

		std::cout << targetErrorCloudIndex << "<-" << sourceErrorCloudIndex
			<< "\t" << rmsError_total << "\t" << squareErrors_total.size()
			<< "\t" << sorted_errors.front() << "\t" << sorted_errors.back() << std::endl;

		std::vector<double> point_pair_indices(errors.size());		
		for ( int j = 0; j < errors.size(); j++ ) point_pair_indices[j] = 1.0 * j;

		max_index = max_index > errors.size() ? max_index : errors.size();
		max_error = max_error > sorted_errors.back() ? max_error : sorted_errors.back();

		plot.addPlotData( point_pair_indices, sorted_errors, ( QString::number(targetErrorCloudIndex) + "<-" + QString::number(sourceErrorCloudIndex) ).toStdString().c_str() );
		int num_histogram = 10;
		if (sorted_errors.back() - sorted_errors.front() > 1e-6)
		{
			plot1.addHistogramData(errors, num_histogram, ( QString::number(targetErrorCloudIndex) + "<-" + QString::number(sourceErrorCloudIndex) ).toStdString().c_str() );
		}
		else
		{
			std::cerr << ( QString::number(targetErrorCloudIndex) + "<-" + QString::number(sourceErrorCloudIndex) ).toStdString().c_str() << " : max or min error is too close to split" << std::endl;	
		}

		total_errors.insert(total_errors.end(), sorted_errors.begin(), sorted_errors.end());

		for (int j = inverseStartIndex; j < correspondenceIndices.size(); ++j)
		{
			int index = correspondenceIndices[j].targetIndex;
			float error = errors[j];                                    //sqrtf(squareErrors_total[j]);
			(*sumErrorCloud[targetErrorCloudIndex])[index].curvature += error;
			(*maxErrorCloud[targetErrorCloudIndex])[index].curvature = error > (*maxErrorCloud[targetErrorCloudIndex])[index].curvature ? error : (*maxErrorCloud[targetErrorCloudIndex])[index].curvature;		

			if (pcl_isnan((*sumErrorCloud[targetErrorCloudIndex])[index].curvature))
			{
				std::cout << targetErrorCloudIndex << " <- " << sourceErrorCloudIndex << std::endl;
				std::cout << "error : " << error << std::endl;				
				std::cout << "SUM NAN ERROR: " << j << std::endl;
				return 0;
			}

			if (pcl_isnan((*maxErrorCloud[targetErrorCloudIndex])[index].curvature))
			{
				std::cout << targetErrorCloudIndex << " <- " << sourceErrorCloudIndex << std::endl;
				std::cout << "error : " << error << std::endl;
				std::cout << "MAX NAN ERROR: " << j << std::endl;
				return 0;
			}	
		}

		for (int j = 0; j < inverseStartIndex; ++j)
		{
	    	int index = correspondenceIndices[j].sourceIndex;
	    	float error = errors[j];                                   //sqrtf(squareErrors_total[j]);
			(*sumErrorCloud[sourceErrorCloudIndex])[index].curvature += error;
			(*maxErrorCloud[sourceErrorCloudIndex])[index].curvature = error > (*maxErrorCloud[sourceErrorCloudIndex])[index].curvature ? error : (*maxErrorCloud[sourceErrorCloudIndex])[index].curvature;	

			if (pcl_isnan((*sumErrorCloud[sourceErrorCloudIndex])[index].curvature))
			{
				std::cout << targetErrorCloudIndex << " <- " << sourceErrorCloudIndex << std::endl;
				std::cout << "error : " << error << std::endl;
				std::cout << "SUM NAN ERROR: " << j << std::endl;
				return 0;
			}

			if (pcl_isnan((*maxErrorCloud[sourceErrorCloudIndex])[index].curvature))
			{
				std::cout << targetErrorCloudIndex << " <- " << sourceErrorCloudIndex << std::endl;
				std::cout << "error : " << error << std::endl;
				std::cout << "MAX NAN ERROR: " << j << std::endl;
				return 0;
			}			
		}
	}

	plot.setXRange( 0, max_index );
	plot.setYRange( 0, max_error );
	plot.spin();
	plot1.spin();

	std::vector<double> point_pair_indices(total_errors.size());		
	for ( int i = 0; i < total_errors.size(); i++ ) point_pair_indices[i] = 1.0 * i;

	std::vector<double> sorted_total_errors = total_errors;
	std::sort( sorted_total_errors.begin(), sorted_total_errors.end());	

	plot2.addPlotData( point_pair_indices, sorted_total_errors, "total");
	plot3.addHistogramData(sorted_total_errors, 10, "total");

	plot2.setXRange( 0, total_errors.size());
	plot2.setYRange( 0, max_error );
	plot2.spin();
	plot3.spin();

	for (int i = 0; i < total_errors.size(); ++i)
	{
		if (pcl_isnan(total_errors[i]))
		{
			//std::cout << targetErrorCloudIndex << " <- " << sourceErrorCloudIndex << std::endl;
			std::cout << "NAN ERROR: " << i << std::endl;
			return 0;
		}
	}

	float min_sumerror = 0.0f;  //FOR bunny
	float max_sumerror = 0.001f;  //FOR bunny
	// float min_sumerror = 0.0f;  //FOR buste
	// float max_sumerror = 1.0f;  //FOR buste

	for (int i = 0; i < sumErrorCloud.size(); ++i)
	{
		QString cloudName = QString::number(i);
		Cloud* cloud = cloudManager->getCloud(cloudName);

		QString fileName =  cloud->getFileName();
		QFileInfo fileInfo(fileName);
		// QString newFileName = fileInfo.path() + "/" + fileInfo.baseName() + "_sumerror.ply";		
		QString newFileName = QString(output_directory.c_str()) + "/" + fileInfo.baseName() + "_sumerror.ply";	

		for (int j = 0; j < sumErrorCloud[i]->size(); ++j)
		{
			unsigned int r, g, b;
			generateErrorRGB((*sumErrorCloud[i])[j].curvature, r, g, b, min_sumerror, max_sumerror);
			(*sumErrorCloud[i])[j].r = r;
			(*sumErrorCloud[i])[j].g = g;
			(*sumErrorCloud[i])[j].b = b;		
		}

		pcl::PLYWriter writer;
		writer.write(newFileName.toStdString(), *sumErrorCloud[i]);		
	}

	float min_maxerror = 0.0f;   //FOR bunny
	float max_maxerror = 0.1f; //FOR bunny
	// float min_maxerror = 0.0f;   //FOR buste
	// float max_maxerror = 1.0f; //FOR buste

	for (int i = 0; i < maxErrorCloud.size(); ++i)
	{
		QString cloudName = QString::number(i);
		Cloud* cloud = cloudManager->getCloud(cloudName);

		QString fileName =  cloud->getFileName();
		QFileInfo fileInfo(fileName);
		// QString newFileName = fileInfo.path() + "/" + fileInfo.baseName() + "_maxerror.ply";	
		QString newFileName = QString(output_directory.c_str()) + "/" + fileInfo.baseName() + "_maxerror.ply";	

		for (int j = 0; j < maxErrorCloud[i]->size(); ++j)
		{
			unsigned int r, g, b;
			generateErrorRGB((*maxErrorCloud[i])[j].curvature, r, g, b, min_maxerror, max_maxerror);
			(*maxErrorCloud[i])[j].r = r;
			(*maxErrorCloud[i])[j].g = g;
			(*maxErrorCloud[i])[j].b = b;	
		}

		pcl::PLYWriter writer;
		writer.write(newFileName.toStdString(), *maxErrorCloud[i]);		
	}	

	return 0;
}


void generateErrorRGB(float error, 
						unsigned int &red, unsigned int &green, unsigned int &blue,
						float minerror, float maxerror) 
						//float blueerror = 0, float qingerror = 0.25, float greenerror = 0.5, float yellowerror = 0.75, float rederror = 1 )
{
	float fract = ( error - minerror ) / ( maxerror - minerror );
	//std::cout << fract << std::endl;

	if (fract <= 0.25)
	{
		red = 0;
		green = truncf( 4 * fract * 255 );
		blue = 255;
	}
	if (fract > 0.25 && fract <= 0.5)
	{
		red = 0;
		green = 255;
		blue = truncf( ( 1 - 4 * (fract - 0.25) ) * 255 );
	}
	if (fract > 0.5 && fract <= 0.75 )
	{
		red = truncf( 4 * (fract - 0.5) * 255);
		green = 255;
		blue = 0;
	}
	if (fract > 0.75)
	{
		red = 255;
		green = truncf( ( 1 - 4 * (fract - 0.75) ) * 255);
		blue = 0;
	}
}