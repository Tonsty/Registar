#include <vtkDoubleArray.h>
#include <vtkPointData.h>
#include <vtkCellArray.h>
#include <vtkIdTypeArray.h>
#include <vtkCellLocator.h>

#include "../include/utilities.h"

namespace registar
{
	vtkSmartPointer<vtkPolyData> generateVTKPolyData(CloudDataConstPtr cloudData, const Polygons& polygons)
	{
		vtkSmartPointer<vtkPolyData> vtk_polydata = vtkSmartPointer<vtkPolyData>::New();
		vtkSmartPointer<vtkPoints> vtk_points = vtkSmartPointer<vtkPoints>::New();
		vtk_points->SetNumberOfPoints(cloudData->size());
		vtk_points->Modified();
		for (int i = 0; i < cloudData->size(); i++)
		{
			PointType point = (*cloudData)[i];
			vtk_points->SetPoint(i, point.x, point.y, point.z);
		}
		vtk_polydata->SetPoints(vtk_points);
		vtkSmartPointer<vtkUnsignedCharArray> vtk_colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
		vtk_colors->SetNumberOfComponents(3);
		vtk_colors->SetName("Colors");
		vtk_colors->SetNumberOfTuples(cloudData->size());
		vtk_colors->Modified();
		for (int i = 0; i < cloudData->size(); i++)
		{
			PointType point = (*cloudData)[i];
			unsigned char color[3];
			color[0] = point.r;
			color[1] = point.g;
			color[2] = point.b;
			vtk_colors->SetTupleValue(i, color);
		}
		vtk_polydata->GetPointData()->SetScalars(vtk_colors);
		vtkSmartPointer<vtkDoubleArray> vtk_normals = vtkSmartPointer<vtkDoubleArray>::New();
		vtk_normals->SetNumberOfComponents(3);
		vtk_normals->SetNumberOfTuples(vtk_polydata->GetNumberOfPoints());
		vtk_normals->Modified();
		for (int i = 0; i < cloudData->size(); i++)
		{
			PointType point = (*cloudData)[i];
			double normal[3];
			normal[0] = point.normal_x;
			normal[1] = point.normal_y;
			normal[2] = point.normal_z;
			vtk_normals->SetTuple(i, normal);
		}
		vtk_polydata->GetPointData()->SetNormals(vtk_normals);
		vtkSmartPointer<vtkCellArray> vtk_polygons = vtkSmartPointer<vtkCellArray>::New();
		vtkSmartPointer<vtkIdTypeArray> cells = vtkSmartPointer<vtkIdTypeArray>::New();
		cells->SetNumberOfComponents(4);
		cells->SetNumberOfTuples(polygons.size());
		cells->Modified();
		for (int i = 0; i < polygons.size(); i++)
		{
			Polygon polygon = polygons[i];
			cells->SetComponent(i, 0, 3);
			cells->SetComponent(i, 1, polygon.vertices[0]);
			cells->SetComponent(i, 2, polygon.vertices[1]);
			cells->SetComponent(i, 3, polygon.vertices[2]);
		}
		vtk_polygons->SetCells(polygons.size(), cells);
		vtk_polydata->SetPolys(vtk_polygons);

		return vtk_polydata;
	}

	vtkSmartPointer<vtkPolyData> generateVTKPolyData(PolygonMeshConstPtr polygonMesh )
	{
		CloudDataPtr cloudData(new CloudData);
		pcl::fromPCLPointCloud2(polygonMesh->cloud, *cloudData);
		return generateVTKPolyData(cloudData, polygonMesh->polygons);
	}

	void hausdorffDistance(const CloudData &cloud_in, vtkSmartPointer<vtkPolyData> polyData_target, CloudData &cloud_out)
	{
		pcl::copyPointCloud( cloud_in, cloud_out);

		vtkSmartPointer<vtkCellLocator> cellLocator = vtkSmartPointer<vtkCellLocator>::New();
		cellLocator->SetDataSet(polyData_target);
		cellLocator->BuildLocator();

		for (int i = 0; i < cloud_in.size(); i++)
		{
			double testPoint[3];
			testPoint[0] = cloud_in[i].x;
			testPoint[1] = cloud_in[i].y;
			testPoint[2] = cloud_in[i].z;
			double closestPoint[3];
			double closestPointDist2;
			vtkIdType cellId;
			int subId;
			cellLocator->FindClosestPoint(testPoint, closestPoint, cellId, subId, closestPointDist2);
			cloud_out[i].curvature = sqrt(closestPointDist2);
		}
	}
}