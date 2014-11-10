#include <vtkVersion.h>
#include <vtkSmartPointer.h>
//#include <vtkXMLPolyDataReader.h>
#include <vtkPointData.h>
#include <vtkPLYReader.h>
#include <vtkOBJReader.h>
#include <vtkSphereSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkPlane.h>
#include <vtkCutter.h>
#include <vtkProperty.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleRubberBandPick.h>
#include <vtkMath.h>
#include <vtkProbeFilter.h>
#include <vtkStripper.h>

#include <pcl/console/parse.h>

#include <Eigen/Dense>

#include "../set_color/set_color.h"

int main(int argc, char *argv[])
{ 
	std::vector< vtkSmartPointer<vtkPolyData> > inputPolyDatas;
	std::vector< vtkSmartPointer<vtkPlane> > planes;
	if(argc > 1)
    {
		std::vector<int> p_file_indices_ply = pcl::console::parse_file_extension_argument (argc, argv, ".ply");	
		for (int i = 0; i < p_file_indices_ply.size(); ++i)
		{
			vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
			vtkSmartPointer<vtkPolyData> inputPolyData;
			//std::cout << argv[p_file_indices_ply[i]] << std::endl;
			reader->SetFileName(argv[p_file_indices_ply[i]]);
			reader->Update();
			inputPolyData = reader->GetOutput();
			inputPolyDatas.push_back(inputPolyData);
			//std::cout << inputPolyData->GetNumberOfVerts() << std::endl;
			//std::cout << inputPolyData->GetNumberOfPolys() << std::endl;
		}
		std::vector<int> p_file_indices_obj = pcl::console::parse_file_extension_argument (argc, argv, ".obj");
		for (int i = 0; i < p_file_indices_obj.size(); ++i)
		{
			vtkSmartPointer<vtkOBJReader> reader = vtkSmartPointer<vtkOBJReader>::New();
			vtkSmartPointer<vtkPolyData> planePolyData;
			//std::cout << argv[p_file_indices_obj[i]] << std::endl;
			reader->SetFileName(argv[p_file_indices_obj[i]]);
			reader->Update();
			planePolyData = reader->GetOutput();

			Eigen::Vector3d origin;
			Eigen::Vector3d normal;
			Eigen::Vector3d points[4];
			for (int j = 0; j < 4; j++)
			{
				double point[3];
				planePolyData->GetPoints()->GetPoint(j, point);
				points[j].x() = point[0];
				points[j].y() = point[1];
				points[j].z() = point[2];

				std::cout << points[j].x() << "," << points[j].y() << "," << points[j].z() << std::endl;
			}

			origin = points[0];
			normal = (points[1] - points[0] ).cross(points[2] - points[0]).normalized();

			//std::cout << origin << std::endl;
			//std::cout << normal << std::endl;

			vtkSmartPointer<vtkPlane> plane = vtkSmartPointer<vtkPlane>::New();
			plane->SetOrigin(origin.x(), origin.y(), origin.z());
			plane->SetNormal(normal.x(), normal.y(), normal.z());
			planes.push_back(plane);
		}
    }
	else
    {
		vtkSmartPointer<vtkPolyData> inputPolyData;
		vtkSmartPointer<vtkSphereSource> sphereSource = vtkSmartPointer<vtkSphereSource>::New();
		sphereSource->SetThetaResolution(30);
		sphereSource->SetPhiResolution(15);
		sphereSource->Update();
		inputPolyData = sphereSource->GetOutput();

		inputPolyDatas.push_back(inputPolyData);
    }

	std::vector<RGB_> rgbs = generateUniformColors(inputPolyDatas.size(), 0, 359.99);	

	// Create renderers and add actors of plane and cube
	vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
	//renderer->SetBackground(.1, .2, .3);
	renderer->SetBackground(1, 1, 1);

	for (int i = 0; i < inputPolyDatas.size(); i++)
	{
		vtkSmartPointer<vtkPolyData> inputPolyData = inputPolyDatas[i];

		vtkSmartPointer<vtkPolyDataMapper> inputMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
#if VTK_MAJOR_VERSION <= 5
		inputMapper->SetInput(inputPolyData);
#else
		inputMapper->SetInputData(inputPolyData);
#endif
		// Create input actor
		vtkSmartPointer<vtkActor> inputActor = vtkSmartPointer<vtkActor>::New();
		//inputActor->GetProperty()->SetColor(1.0, 0.8941, 0.7686); // bisque
		inputActor->SetMapper(inputMapper);
		renderer->AddActor(inputActor); //display the cube

		for (int j = 0; j < planes.size(); j++)
		{
			// Create a plane to cut
			vtkSmartPointer<vtkPlane> plane = planes[j];

			// Create cutter
			vtkSmartPointer<vtkCutter> cutter = vtkSmartPointer<vtkCutter>::New();
			cutter->SetCutFunction(plane);
#if VTK_MAJOR_VERSION <= 5
			cutter->SetInput(inputPolyData);
#else
			cutter->SetInputData(inputPolyData);
#endif
			//cutter->GenerateValues(20, -distanceMin, distanceMax);
			cutter->GenerateValues(1, 0, 0);

			//vtkSmartPointer<vtkStripper> stripper = vtkSmartPointer<vtkStripper>::New();
			//stripper->SetInputConnection(cutter->GetOutputPort());

			vtkSmartPointer<vtkPolyDataMapper> cutterMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
			cutterMapper->SetInputConnection( cutter->GetOutputPort());
			//cutterMapper->SetInputConnection( stripper->GetOutputPort());
			cutterMapper->ScalarVisibilityOff();

			// Create plane actor
			vtkSmartPointer<vtkActor> planeActor = vtkSmartPointer<vtkActor>::New();
			planeActor->GetProperty()->SetColor(1.0,1,0);
			//planeActor->GetProperty()->SetColor(0,0,0);
			//float r, g, b;
			//r = rgbs[i].r/255.0;
			//g = rgbs[i].g/255.0;
			//b = rgbs[i].b/255.0;
			//std::cout << r << " " << g << " " << b << std::endl;
			//planeActor->GetProperty()->SetColor( r, g, b );
			planeActor->GetProperty()->SetLineWidth(1);
			planeActor->SetMapper(cutterMapper);

			renderer->AddActor(planeActor); //display the rectangle resulting from the cut
		}
	}
  
	//Add renderer to renderwindow and render
	vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
	renderWindow->AddRenderer(renderer);
	renderWindow->SetSize(600, 600);
  
	vtkSmartPointer<vtkRenderWindowInteractor> interactor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
	interactor->SetRenderWindow(renderWindow);

	renderWindow->Render();
	
	vtkSmartPointer<vtkInteractorStyleRubberBandPick> interactorStyle = vtkSmartPointer<vtkInteractorStyleRubberBandPick>::New();
	interactor->SetInteractorStyle(interactorStyle);
	interactor->Start();
  
	return EXIT_SUCCESS;
}

//for (int i = 0; i < inputPolyDatas.size(); i++)
//{
//	vtkSmartPointer<vtkPolyData> inputPolyData = inputPolyDatas[i];
//
//	double minBound[3];
//	minBound[0] = inputPolyData->GetBounds()[0];
//	minBound[1] = inputPolyData->GetBounds()[2];
//	minBound[2] = inputPolyData->GetBounds()[4];
//
//	double maxBound[3];
//	maxBound[0] = inputPolyData->GetBounds()[1];
//	maxBound[1] = inputPolyData->GetBounds()[3];
//	maxBound[2] = inputPolyData->GetBounds()[5];
//
//	double center[3];
//	center[0] = inputPolyData->GetCenter()[0];
//	center[1] = inputPolyData->GetCenter()[1];
//	center[2] = inputPolyData->GetCenter()[2];
//
//	double distanceMin = sqrt(vtkMath::Distance2BetweenPoints(minBound, center));
//	double distanceMax = sqrt(vtkMath::Distance2BetweenPoints(maxBound, center));
//}

