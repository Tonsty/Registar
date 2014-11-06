#include <vtkVersion.h>
#include <vtkSmartPointer.h>
//#include <vtkXMLPolyDataReader.h>
#include <vtkPointData.h>
#include <vtkPLYReader.h>
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

#include "../set_color/set_color.h"

int main(int argc, char *argv[])
{ 
	std::vector< vtkSmartPointer<vtkPolyData> > inputPolyDatas;
	if(argc > 1)
    {
		std::vector<int> p_file_indices_ply = pcl::console::parse_file_extension_argument (argc, argv, ".ply");	
		for (int i = 0; i < p_file_indices_ply.size(); ++i)
		{
			vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
			vtkSmartPointer<vtkPolyData> inputPolyData;
			std::cout << argv[p_file_indices_ply[i]] << std::endl;
			reader->SetFileName(argv[p_file_indices_ply[i]]);
			reader->Update();
			inputPolyData = reader->GetOutput();
			inputPolyDatas.push_back(inputPolyData);
			//std::cout << inputPolyData->GetNumberOfVerts() << std::endl;
			//std::cout << inputPolyData->GetNumberOfPolys() << std::endl;
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

	std::vector<RGB> rgbs = generateUniformColors(inputPolyDatas.size(), 0, 359.99);	

	// Create renderers and add actors of plane and cube
	vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
	//renderer->SetBackground(.1, .2, .3);
	renderer->SetBackground(1, 1, 1);

	for (int i = 0; i < inputPolyDatas.size(); i++)
	{
		vtkSmartPointer<vtkPolyData> inputPolyData = inputPolyDatas[i];

		double minBound[3];
		minBound[0] = inputPolyData->GetBounds()[0];
		minBound[1] = inputPolyData->GetBounds()[2];
		minBound[2] = inputPolyData->GetBounds()[4];

		double maxBound[3];
		maxBound[0] = inputPolyData->GetBounds()[1];
		maxBound[1] = inputPolyData->GetBounds()[3];
		maxBound[2] = inputPolyData->GetBounds()[5];

		double center[3];
		center[0] = inputPolyData->GetCenter()[0];
		center[1] = inputPolyData->GetCenter()[1];
		center[2] = inputPolyData->GetCenter()[2];

		double distanceMin = sqrt(vtkMath::Distance2BetweenPoints(minBound, center));
		double distanceMax = sqrt(vtkMath::Distance2BetweenPoints(maxBound, center));
	}

	for (int i = 0; i < inputPolyDatas.size(); i++)
	{
		vtkSmartPointer<vtkPolyData> inputPolyData = inputPolyDatas[i];

		vtkSmartPointer<vtkPolyDataMapper> inputMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
#if VTK_MAJOR_VERSION <= 5
		inputMapper->SetInput(inputPolyData);
#else
		inputMapper->SetInputData(inputPolyData);
#endif

		// Create a plane to cut
		vtkSmartPointer<vtkPlane> plane = vtkSmartPointer<vtkPlane>::New();
		plane->SetOrigin(0, 0, 0);
		//plane->SetOrigin(inputPolyData->GetCenter());
		//plane->SetNormal(1,1,1);
		plane->SetNormal(0,1,0);
		//plane->SetNormal(1,0,0);

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
		planeActor->GetProperty()->SetLineWidth(3);
		planeActor->SetMapper(cutterMapper);

		// Create input actor
		vtkSmartPointer<vtkActor> inputActor = vtkSmartPointer<vtkActor>::New();
		//inputActor->GetProperty()->SetColor(1.0, 0.8941, 0.7686); // bisque
		inputActor->SetMapper(inputMapper);

		renderer->AddActor(planeActor); //display the rectangle resulting from the cut
		renderer->AddActor(inputActor); //display the cube
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
