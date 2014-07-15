#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/filter_indices.h>

#include <opencv2/opencv.hpp>

int main (int argc, char** argv)
{
	int camera_num = atoi( argv[1] );
	std::string  pre_registration_directory = argv[2];
	std::string  initial_registered_directory = argv[3];

	Eigen::Matrix3f *myR1s = new Eigen::Matrix3f[camera_num];
	Eigen::Vector3f *translation_vectors = new Eigen::Vector3f[camera_num];
	Eigen::Matrix3f *mybundleR0s = new Eigen::Matrix3f[camera_num];
	Eigen::Matrix3f *mybundleR1s = new Eigen::Matrix3f[camera_num];
	Eigen::Vector3f *myt0s = new Eigen::Vector3f[camera_num];
	Eigen::Vector3f *myt1s = new Eigen::Vector3f[camera_num];

	float mean_scale = 0;
	float *scales = new float[camera_num];
	int succeed_num = 0;
	for ( int k = 0; k < camera_num; k++ )
	{
		std::stringstream sstream_yml;
		sstream_yml << pre_registration_directory << "/stereo/stereo_" << k << ".yml";
		cv::FileStorage bino_extrinsic_file( sstream_yml.str(), cv::FileStorage::READ );

		if ( bino_extrinsic_file.isOpened() )
		{
			cv::Mat R1;
			bino_extrinsic_file["R1"] >> R1;

			Eigen::Matrix3f myR1;
			for ( int i = 0; i < 3; i++ )
			{
				for ( int j = 0; j < 3; j++ )
				{
					myR1(i,j) = R1.at<double>(i,j);
				}
			}

			cv::Mat translation_vector;
			bino_extrinsic_file["translation_vector"] >> translation_vector;

			Eigen::Vector3f mytranslation_vector;
			for ( int i = 0; i < 3; i++ )
			{
				mytranslation_vector(i) = translation_vector.at<double>(i,0);
			}

			std::stringstream sstream_out;
			sstream_out << pre_registration_directory << "/bundle/bundle_" << k <<".out";
			std::fstream extrinsic_file( sstream_out.str().c_str(), std::ios::in );

			float temp0;
			extrinsic_file >> temp0 >> temp0 >> temp0;
			Eigen::Matrix3f mybundleR0;
			Eigen::Vector3f myt0;
			for ( int i = 0; i < 3; i++ )
			{
				for ( int j = 0; j < 3; j++ )
				{
					extrinsic_file >> mybundleR0(i,j);
				}
			}
			for ( int i = 0; i < 3; i++ )
			{
				extrinsic_file >> myt0(i);
			}

			float temp1;
			Eigen::Matrix3f mybundleR1;
			Eigen::Vector3f myt1;
			extrinsic_file >> temp1 >> temp1 >> temp1;
			for ( int i = 0; i < 3; i++ )
			{
				for ( int j = 0; j < 3; j++ )
				{
					extrinsic_file >> mybundleR1(i,j);
				}
			}
			for ( int i = 0; i < 3; i++ )
			{
				extrinsic_file >> myt1[i];
			}
			extrinsic_file.close();

			//Eigen::Vector3f origin0 = mybundleR0.transpose() * (-myt0);
			//Eigen::Vector3f origin1 = mybundleR1.transpose() * (-myt1);
			float scale = ( ( mybundleR0.transpose() * (-myt0) ) - ( mybundleR1.transpose() * (-myt1) ) ).norm() / mytranslation_vector.norm();
			std::cout << k << " " << scale << std::endl;
			mean_scale += scale;
			scales[k] = scale;

			myR1s[k] = myR1;
			translation_vectors[k] = mytranslation_vector;
			mybundleR0s[k] = mybundleR0;
			mybundleR1s[k] = mybundleR1;
			myt0s[k] = myt0;
			myt1s[k] = myt1;

			succeed_num++;
		}
	}
	mean_scale /= succeed_num;
	std::cout << succeed_num << " succeed" << std::endl;
	std::cout << "mean " << mean_scale << std::endl;

	for (int i = 0; i < camera_num; ++i)
	{
		float scale;
		Eigen::Matrix3f rotation_matrix;
		Eigen::Vector3f translation_vector;

		scale = scales[i];

		Eigen::Matrix3f flipYZAxis;
		flipYZAxis = Eigen::Matrix3f::Identity();
		flipYZAxis(1,1) = -1;
		flipYZAxis(2,2) = -1;

		rotation_matrix = mybundleR0s[i].transpose() * flipYZAxis * myR1s[i].transpose();

		translation_vector = mybundleR0s[i].transpose() * -myt0s[i];

		std::stringstream sstream_initial_tf;
		sstream_initial_tf << initial_registered_directory << "/" << i <<"_initial.tf";

		std::fstream tf_file( sstream_initial_tf.str().c_str(), std::ios::out );

		tf_file << "scale : " << scale << std::endl;
		tf_file << "rotation_matrix : " << std::endl;
		for (int i = 0; i < 3; ++i)
		{
			for (int j = 0; j < 3; ++j)
			{
				tf_file << rotation_matrix(i, j) << " ";
			}
			tf_file <<  std::endl;
		}
		tf_file << "translation_vector : " << std::endl;
		for (int i = 0; i < 3; ++i)
		{
			tf_file << translation_vector(i) << " ";
		}
		tf_file <<  std::endl;
	}

	std::stringstream sstream_plyfile_list;
	sstream_plyfile_list << pre_registration_directory << "/data/plyfile.list";
      	std::fstream plyfile_list( sstream_plyfile_list.str().c_str(), std::ios::in );

	for ( int i = 0; i < camera_num; i++ )
	{	
	  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGBNormal>);


		pcl::PLYReader reader;

		std::string plyfile_name;
                //plyfile_list >> plyfile_name;
		getline( plyfile_list, plyfile_name );
		reader.read (pre_registration_directory + "/data/" +  plyfile_name, *cloud); 

		if ( cloud->size() == 0 ) continue;

		std::cerr << "Cloud before filtering: " << std::endl;
		std::cerr << *cloud << std::endl;

		*cloud_filtered = *cloud;
		for ( int j = 0; j < cloud->points.size(); j++ )
		{
			Eigen::Vector3f vertex, normal;
			vertex[0] = cloud->points[j].x;
			vertex[1] = cloud->points[j].y;
			vertex[2] = cloud->points[j].z;
			normal[0] = cloud->points[j].normal_x;
			normal[1] = cloud->points[j].normal_y;
			normal[2] = cloud->points[j].normal_z;
			vertex = myR1s[i].transpose() * vertex;
			normal = myR1s[i].transpose() * normal;

			vertex[0]= vertex[0] * scales[i];
			vertex[1]=-vertex[1] * scales[i];
			vertex[2]=-vertex[2] * scales[i];

			normal[1]=-normal[1];
			normal[2]=-normal[2];

			vertex = mybundleR0s[i].transpose() * ( vertex - myt0s[i] );
			normal = mybundleR0s[i].transpose() * normal;
			cloud_filtered->points[j].x = vertex[0];
			cloud_filtered->points[j].y = vertex[1];
			cloud_filtered->points[j].z = vertex[2];
			cloud_filtered->points[j].normal_x = normal[0];
			cloud_filtered->points[j].normal_y = normal[1];
			cloud_filtered->points[j].normal_z = normal[2];
		}

		std::cerr << "Cloud after filtering: " << std::endl;
		std::cerr << *cloud_filtered << std::endl;

		std::stringstream sstream_initial_ply;
		sstream_initial_ply << initial_registered_directory << "/" << i <<"_initial.ply";
		
		// pcl::PLYWriter writer;
		// writer.write ( sstream_initial_ply.str(), *cloud_filtered );
		std::vector<int> mapping_index;
		pcl::removeNaNFromPointCloud( *cloud_filtered, *cloud_filtered, mapping_index );
		
		// pcl::PLYWriter writer;
		// writer.write(sstream_initial_ply.str(), *cloud_filtered);
		pcl::io::savePLYFileASCII( sstream_initial_ply.str(), *cloud_filtered );
	}

	return (0);
}
