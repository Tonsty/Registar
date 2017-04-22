#include <QtCore/QDebug>
#include <QtCore/QFileInfo>

#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
//#include <pcl/io/png_io.h>
#include <pcl/filters/filter.h>

#include "../include/cloudio.h"

using namespace registar;

CloudIO::CloudIO(){}

CloudIO::~CloudIO(){}

bool CloudIO::importPLYCloudData(const QString &fileName, CloudDataPtr cloudData)
{
	pcl::PLYReader reader;
	if (reader.read(fileName.toStdString(), *cloudData) == 0)
	{
		//qDebug() << "sensor_origin (x,y,z,w):" << cloudData->sensor_origin_.x() << ", " << cloudData->sensor_origin_.y()
		//		<< ", " << cloudData->sensor_origin_.z() << ", " << cloudData->sensor_origin_.w();
		//qDebug() << "sensor_orientation (w,x,y,z): " << cloudData->sensor_orientation_.w() << ", " << cloudData->sensor_orientation_.x() 
		//		<< ", " << cloudData->sensor_orientation_.y() << ", " << cloudData->sensor_orientation_.z();
		cloudData->sensor_origin_ = Eigen::Vector4f(0, 0, 0, 0);
		cloudData->sensor_orientation_ = Eigen::Quaternionf(1, 0, 0, 0);

		// QFileInfo fileInfo(fileName);
		// QString pngFileName = fileInfo.path() + "/" + fileInfo.baseName() + ".png";
		// pcl::io::savePNGFile( pngFileName.toStdString(), *cloudData);

		// std::vector<int> nanIndicesVector;
		// pcl::removeNaNFromPointCloud( *cloudData, *cloudData, nanIndicesVector );
	}
	else return false;

	return true;
}

bool CloudIO::importPLYPolygonMesh(const QString &fileName, PolygonMeshPtr polygonMesh)
{
	pcl::PLYReader reader;
	if (reader.read(fileName.toStdString(), *polygonMesh) == 0)
	{
		std::cout << "height: " << polygonMesh->cloud.height << " , " << "width: " <<  polygonMesh->cloud.width << std::endl;
		std::cout << "polygons: " << polygonMesh->polygons.size() << std::endl;
	}
	else return false;

	return true;
}

bool CloudIO::exportPLYPolygonMesh(const QString &fileName, PolygonMeshConstPtr polygonMesh)
{
	pcl::io::savePLYFile(fileName.toStdString(), *polygonMesh);
	return true;
}

bool CloudIO::importTransformation(const QString &fileName, Eigen::Matrix4f &transformation)
{
	QFileInfo fileInfo(fileName);
	QString tfFileName = fileInfo.path() + "/" + fileInfo.completeBaseName() + ".tf";

	qDebug() << tfFileName;

	QFile file(tfFileName);
	if (!file.open(QIODevice::ReadOnly))
	{
		qDebug() << "Cannot import transformation!";
		transformation = Eigen::Matrix4f::Identity();
		return false;
	}

	QTextStream in(&file);
	for (int i = 0; i < 4; ++i)
	{
		for (int j = 0; j < 4; ++j)
		{
			in >> transformation(i, j);
		}
	}
	
	return true;
}

bool CloudIO::importBoundaries(const QString &fileName, BoundariesPtr boundaries)
{
	QFileInfo fileInfo(fileName);
	QString bdFileName = fileInfo.path() + "/" + fileInfo.completeBaseName() + ".bd";

	qDebug() << bdFileName;

	QFile file(bdFileName);
	if (!file.open(QIODevice::ReadOnly))
	{
		qDebug() << "Cannot import boundaries!";
		return false;
	}

	pcl::io::loadPCDFile(bdFileName.toStdString(), *boundaries);

	return true;
}

bool CloudIO::exportPLYCloudData(const QString &fileName, CloudDataConstPtr cloudData)
{
	pcl::PLYWriter writer;
	if (cloudData->isOrganized())
	{
		pcl::PCLPointCloud2 msg;
		pcl::toPCLPointCloud2( *cloudData, msg );
		writer.writeASCII( fileName.toStdString(), msg,  Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), 8, false );		
	}
	else
	{
		std::vector<int> nanIndicesVector;
		CloudDataPtr temp(new CloudData);
		pcl::removeNaNFromPointCloud( *cloudData, *temp, nanIndicesVector );
		writer.write(fileName.toStdString(), *temp);
	}

	return true;
}

bool CloudIO::exportTransformation(const QString &fileName, const Eigen::Matrix4f &transformation)
{
	QFileInfo fileInfo(fileName);
	QString tfFileName = fileInfo.path() + "/" + fileInfo.completeBaseName() + ".tf";

	qDebug() << tfFileName;

	QFile file(tfFileName);
	if (!file.open(QIODevice::WriteOnly))
	{
		qDebug() << "Cannot export transformation!";
		return false;
	}

	QTextStream out(&file);
	for (int i = 0; i < 4; ++i)
	{
		for (int j = 0; j < 4; ++j)
		{
			out << transformation(i, j) << "\t";
		}
		out << endl;
	}

	return true;
}

bool CloudIO::exportBoundaries(const QString &fileName, BoundariesConstPtr boundaries)
{
	QFileInfo fileInfo(fileName);
	QString bdFileName = fileInfo.path() + "/" + fileInfo.completeBaseName() + ".bd";

	qDebug() << bdFileName;

	if(boundaries != NULL && boundaries->size() > 0) pcl::io::savePCDFile(bdFileName.toStdString(), *boundaries);

	return true;
}

bool CloudIO::importVTKCloudData(const QString &fileName, CloudDataPtr cloudData)
{
	return true;
}

bool CloudIO::exportVTKCloudData(const QString &fileName, CloudDataConstPtr cloudData)
{
	pcl::PCLPointCloud2Ptr cloud(new pcl::PCLPointCloud2);
	pcl::toPCLPointCloud2(*cloudData, *cloud);
	if (cloud->data.empty ())
	{
		PCL_ERROR ("[pcl::io::saveVTKFile] Input point cloud has no data!\n");
		return (-1);
	}

	// Open file
	std::ofstream fs;
	fs.precision (5);
	fs.open (fileName.toStdString().c_str ());

	unsigned int nr_points  = cloud->width * cloud->height;
	unsigned int point_size = static_cast<unsigned int> (cloud->data.size () / nr_points);

	// Write the header information
	fs << "# vtk DataFile Version 3.0\nvtk output\nASCII\nDATASET POLYDATA\nPOINTS " << nr_points << " float" << std::endl;

	// Iterate through the points
	for (unsigned int i = 0; i < nr_points; ++i)
	{
		int xyz = 0;
		for (size_t d = 0; d < cloud->fields.size (); ++d)
		{
			int count = cloud->fields[d].count;
			if (count == 0)
				count = 1;          // we simply cannot tolerate 0 counts (coming from older converter code)
			int c = 0;
			if ((cloud->fields[d].datatype == pcl::PCLPointField::FLOAT32) && (
				cloud->fields[d].name == "x" || 
				cloud->fields[d].name == "y" || 
				cloud->fields[d].name == "z"))
			{
				float value;
				memcpy (&value, &cloud->data[i * point_size + cloud->fields[d].offset + c * sizeof (float)], sizeof (float));
				fs << value;
				if (++xyz == 3)
					break;
			}
			fs << " ";
		}
		if (xyz != 3)
		{
			PCL_ERROR ("[pcl::io::saveVTKFile] Input point cloud has no XYZ data!\n");
			return (-2);
		}
		fs << std::endl;
	}

	// Write vertices
	fs << "\nVERTICES " << nr_points << " " << 2*nr_points << std::endl;
	for (unsigned int i = 0; i < nr_points; ++i)
		fs << "1 " << i << std::endl;

	// Write RGB values
	int field_index = getFieldIndex (*cloud, "rgb");
	if (field_index != -1)
	{
		fs << "\nPOINT_DATA " << nr_points << "\nCOLOR_SCALARS RGB 3\n";
		for (unsigned int i = 0; i < nr_points; ++i)
		{
			int count = cloud->fields[field_index].count;
			if (count == 0)
				count = 1;          // we simply cannot tolerate 0 counts (coming from older converter code)
			int c = 0;
			if (cloud->fields[field_index].datatype == pcl::PCLPointField::FLOAT32)
			{
				pcl::RGB color;
				memcpy (&color, &cloud->data[i * point_size + cloud->fields[field_index].offset + c * sizeof (float)], sizeof (pcl::RGB));
				int r = color.r;
				int g = color.g;
				int b = color.b;
				fs << static_cast<float> (r) / 255.0f << " " << static_cast<float> (g) / 255.0f << " " << static_cast<float> (b) / 255.0f;
			}
			fs << std::endl;
		}
	}

	// Write normal values
	field_index = getFieldIndex (*cloud, "normal_x");
	if (field_index != -1)
	{
		fs << "\nNORMALS normal float\n";
		for (unsigned int i = 0; i < nr_points; ++i)
		{
			int xyz = 0;
			for (size_t d = 0; d < cloud->fields.size (); ++d)
			{
				int count = cloud->fields[d].count;
				if (count == 0)
					count = 1;          // we simply cannot tolerate 0 counts (coming from older converter code)
				int c = 0;
				if ((cloud->fields[d].datatype == pcl::PCLPointField::FLOAT32) && (
					cloud->fields[d].name == "normal_x" || 
					cloud->fields[d].name == "normal_y" || 
					cloud->fields[d].name == "normal_z"))
				{
					float value;
					memcpy (&value, &cloud->data[i * point_size + cloud->fields[d].offset + c * sizeof (float)], sizeof (float));
					fs << value;
					if (++xyz == 3)
						break;
				}
				fs << " ";
			}
			if (xyz != 3)
			{
				PCL_ERROR ("[pcl::io::saveVTKFile] Input point cloud has no NORMAL_XYZ data!\n");
				return (-2);
			}
			fs << std::endl;
		}
	}

	// Write curvature values
	field_index = getFieldIndex (*cloud, "curvature");
	if (field_index != -1)
	{
		fs << "\nSCALARS curvature float\nLOOKUP_TABLE default\n";
		for (unsigned int i = 0; i < nr_points; ++i)
		{
			int count = cloud->fields[field_index].count;
			if (count == 0)
				count = 1;          // we simply cannot tolerate 0 counts (coming from older converter code)
			int c = 0;
			if (cloud->fields[field_index].datatype == pcl::PCLPointField::FLOAT32)
			{
				float curvature;
				memcpy (&curvature, &cloud->data[i * point_size + cloud->fields[field_index].offset + c * sizeof (float)], sizeof (float));
				fs << curvature;
			}
			fs << std::endl;
		}
	}

	// Close file
	fs.close ();
	return true;
}

bool CloudIO::importVTKPolygonMesh(const QString &fileName, PolygonMeshPtr polygonMesh)
{
	return true;
}

bool CloudIO::exportVTKPolygonMesh(const QString &fileName, PolygonMeshConstPtr polygonMesh )
{
	if (polygonMesh->cloud.data.empty ())
	{
		PCL_ERROR ("[pcl::io::saveVTKFile] Input point cloud has no data!\n");
		return (-1);
	}                                       

	// Open file
	std::ofstream fs;
	fs.precision (5);
	fs.open (fileName.toStdString().c_str ());

	unsigned int nr_points  = polygonMesh->cloud.width * polygonMesh->cloud.height;
	unsigned int point_size = static_cast<unsigned int> (polygonMesh->cloud.data.size () / nr_points);

	// Write the header information
	fs << "# vtk DataFile Version 3.0\nvtk output\nASCII\nDATASET POLYDATA\nPOINTS " << nr_points << " float" << std::endl;

	// Iterate through the points
	for (unsigned int i = 0; i < nr_points; ++i)
	{
		int xyz = 0;
		for (size_t d = 0; d < polygonMesh->cloud.fields.size (); ++d)
		{
			int count = polygonMesh->cloud.fields[d].count;
			if (count == 0)
				count = 1;          // we simply cannot tolerate 0 counts (coming from older converter code)
			int c = 0;
			if ((polygonMesh->cloud.fields[d].datatype == pcl::PCLPointField::FLOAT32) && (
				polygonMesh->cloud.fields[d].name == "x" || 
				polygonMesh->cloud.fields[d].name == "y" || 
				polygonMesh->cloud.fields[d].name == "z"))
			{
				float value;
				memcpy (&value, &polygonMesh->cloud.data[i * point_size + polygonMesh->cloud.fields[d].offset + c * sizeof (float)], sizeof (float));
				fs << value;
				if (++xyz == 3)
					break;
			}
			fs << " ";
		}
		if (xyz != 3)
		{
			PCL_ERROR ("[pcl::io::saveVTKFile] Input point cloud has no XYZ data!\n");
			return (-2);
		}
		fs << std::endl;
	}

	//fs << "\nVERTICES " << nr_points << " " << 2*nr_points << "\n";
	//for (unsigned int i = 0; i < nr_points; ++i) fs << "1 " << i << "\n";

	// Write polygons
	// compute the correct number of values:
	size_t triangle_size = polygonMesh->polygons.size ();
	size_t correct_number = triangle_size;
	for (size_t i = 0; i < triangle_size; ++i)
		correct_number += polygonMesh->polygons[i].vertices.size ();
	fs << "\nPOLYGONS " << triangle_size << " " << correct_number << std::endl;
	for (size_t i = 0; i < triangle_size; ++i)
	{
		fs << polygonMesh->polygons[i].vertices.size () << " ";
		size_t j = 0;
		for (j = 0; j < polygonMesh->polygons[i].vertices.size () - 1; ++j)
			fs << polygonMesh->polygons[i].vertices[j] << " ";
		fs << polygonMesh->polygons[i].vertices[j] << std::endl;
	}

	int field_index;

	// Write RGB values
	field_index = getFieldIndex (polygonMesh->cloud, "rgb");
	if (field_index != -1)
	{
		fs << "\nPOINT_DATA " << nr_points << "\nCOLOR_SCALARS RGB 3\n";
		for (unsigned int i = 0; i < nr_points; ++i)
		{
			int count = polygonMesh->cloud.fields[field_index].count;
			if (count == 0)
				count = 1;          // we simply cannot tolerate 0 counts (coming from older converter code)
			int c = 0;
			if (polygonMesh->cloud.fields[field_index].datatype == pcl::PCLPointField::FLOAT32)
			{
				pcl::RGB color;
				memcpy (&color, &polygonMesh->cloud.data[i * point_size + polygonMesh->cloud.fields[field_index].offset + c * sizeof (float)], sizeof (pcl::RGB));
				int r = color.r;
				int g = color.g;
				int b = color.b;
				fs << static_cast<float> (r) / 255.0f << " " << static_cast<float> (g) / 255.0f << " " << static_cast<float> (b) / 255.0f;
			}
			fs << std::endl;
		}
	}

	// Write normal values
	field_index = getFieldIndex (polygonMesh->cloud, "normal_x");
	if (field_index != -1)
	{
		fs << "\nNORMALS normal float\n";
		for (unsigned int i = 0; i < nr_points; ++i)
		{
			int xyz = 0;
			for (size_t d = 0; d < polygonMesh->cloud.fields.size (); ++d)
			{
				int count = polygonMesh->cloud.fields[d].count;
				if (count == 0)
					count = 1;          // we simply cannot tolerate 0 counts (coming from older converter code)
				int c = 0;
				if ((polygonMesh->cloud.fields[d].datatype == pcl::PCLPointField::FLOAT32) && (
					polygonMesh->cloud.fields[d].name == "normal_x" || 
					polygonMesh->cloud.fields[d].name == "normal_y" || 
					polygonMesh->cloud.fields[d].name == "normal_z"))
				{
					float value;
					memcpy (&value, &polygonMesh->cloud.data[i * point_size + polygonMesh->cloud.fields[d].offset + c * sizeof (float)], sizeof (float));
					fs << value;
					if (++xyz == 3)
						break;
				}
				fs << " ";
			}
			if (xyz != 3)
			{
				PCL_ERROR ("[pcl::io::saveVTKFile] Input point cloud has no NORMAL_XYZ data!\n");
				return (-2);
			}
			fs << std::endl;
		}
	}


	//// Write RGB values
	//field_index = getFieldIndex (polygonMesh->cloud, "rgb");
	//if (field_index != -1)
	//{
	//	fs << "\nPOINT_DATA " << nr_points << "\nCOLOR_SCALARS RGB 3\n";
	//	for (unsigned int i = 0; i < nr_points; ++i)
	//	{
	//		int count = polygonMesh->cloud.fields[field_index].count;
	//		if (count == 0)
	//			count = 1;          // we simply cannot tolerate 0 counts (coming from older converter code)
	//		int c = 0;
	//		if (polygonMesh->cloud.fields[field_index].datatype == pcl::PCLPointField::FLOAT32)
	//		{
	//			pcl::RGB color;
	//			memcpy (&color, &polygonMesh->cloud.data[i * point_size + polygonMesh->cloud.fields[field_index].offset + c * sizeof (float)], sizeof (pcl::RGB));
	//			int r = color.r;
	//			int g = color.g;
	//			int b = color.b;
	//			fs << static_cast<float> (r) / 255.0f << " " << static_cast<float> (g) / 255.0f << " " << static_cast<float> (b) / 255.0f;
	//		}
	//		fs << std::endl;
	//	}
	//}

	//// Write normal values
	//field_index = getFieldIndex (polygonMesh->cloud, "normal_x");
	//if (field_index != -1)
	//{
	//	fs << "\nNORMALS normal float\n";
	//	for (unsigned int i = 0; i < nr_points; ++i)
	//	{
	//		int xyz = 0;
	//		for (size_t d = 0; d < polygonMesh->cloud.fields.size (); ++d)
	//		{
	//			int count = polygonMesh->cloud.fields[d].count;
	//			if (count == 0)
	//				count = 1;          // we simply cannot tolerate 0 counts (coming from older converter code)
	//			int c = 0;
	//			if ((polygonMesh->cloud.fields[d].datatype == pcl::PCLPointField::FLOAT32) && (
	//				polygonMesh->cloud.fields[d].name == "normal_x" || 
	//				polygonMesh->cloud.fields[d].name == "normal_y" || 
	//				polygonMesh->cloud.fields[d].name == "normal_z"))
	//			{
	//				float value;
	//				memcpy (&value, &polygonMesh->cloud.data[i * point_size + polygonMesh->cloud.fields[d].offset + c * sizeof (float)], sizeof (float));
	//				fs << value;
	//				if (++xyz == 3)
	//					break;
	//			}
	//			fs << " ";
	//		}
	//		if (xyz != 3)
	//		{
	//			PCL_ERROR ("[pcl::io::saveVTKFile] Input point cloud has no NORMAL_XYZ data!\n");
	//			return (-2);
	//		}
	//		fs << std::endl;
	//	}
	//}

	// Write curvature values
	field_index = getFieldIndex (polygonMesh->cloud, "curvature");
	if (field_index != -1)
	{
		fs << "\nSCALARS curvature float\nLOOKUP_TABLE default\n";
		for (unsigned int i = 0; i < nr_points; ++i)
		{
			int count = polygonMesh->cloud.fields[field_index].count;
			if (count == 0)
				count = 1;          // we simply cannot tolerate 0 counts (coming from older converter code)
			int c = 0;
			if (polygonMesh->cloud.fields[field_index].datatype == pcl::PCLPointField::FLOAT32)
			{
				float curvature;
				memcpy (&curvature, &polygonMesh->cloud.data[i * point_size + polygonMesh->cloud.fields[field_index].offset + c * sizeof (float)], sizeof (float));
				fs << curvature;
			}
			fs << std::endl;
		}
	}

	// Close file
	fs.close ();
	return true;
}

