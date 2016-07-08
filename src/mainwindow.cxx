#include "mainwindow.h"
#include "ui_mainwindow.h"

// Qt headers
#include <QDebug>
#include <QFileDialog>
#include <QMessageBox>
#include <QProgressBar>
#include <QLabel>
#include <QtConcurrent>
#include <QProgressDialog>
#include <QColorDialog>

// ITK headers
#include <itkImageFileReader.h>
#include <itkLineIterator.h>
// #include <itkCropImageFilter.h>
#include <itkExtractImageFilter.h>
#include <itkFlipImageFilter.h>
#include <itkCastImageFilter.h>
#include <itkConnectedComponentImageFilter.h>
#include <itkBinaryThresholdImageFilter.h>

// Vesselness includes
#include <itkHessianToObjectnessMeasureImageFilter.h>
#include <itkMultiScaleHessianBasedMeasureImageFilter.h>
#include <itkRescaleIntensityImageFilter.h>
#include <itkRGBToLuminanceImageFilter.h>

// Geodesic Active Contours includes
#include <itkGeodesicActiveContourLevelSetImageFilter.h>
#include <itkCurvatureAnisotropicDiffusionImageFilter.h>
#include <itkGradientMagnitudeRecursiveGaussianImageFilter.h>
#include <itkSigmoidImageFilter.h>
#include <itkFastMarchingImageFilter.h>
#include <itkBinaryThresholdImageFilter.h>
#include <itkImageRegionIteratorWithIndex.h>


//minimal path includes
#include <itkImageFileWriter.h>
#include <itkPolyLineParametricPath.h>
#include <itkNearestNeighborInterpolateImageFunction.h>
#include <itkLinearInterpolateImageFunction.h>
#include <itkArrivalFunctionToPathFilter.h>
#include <itkSpeedFunctionToPathFilter.h>
#include <itkPathIterator.h>
#include <itkGradientDescentOptimizer.h>
#include <itkRegularStepGradientDescentOptimizer.h>
#include <itkIterateNeighborhoodOptimizer.h>
#include <itkTubeSpatialObject.h>
#include <itkTubeSpatialObjectPoint.h>
#include <itkSpatialObjectPoint.h>
#include <itkSpatialObjectWriter.h>

// VTK headers
#include <vtkImageMapper3D.h>
#include <vtkCamera.h>
#include <vtkRenderWindow.h>
#include <vtkCoordinate.h>
#include <vtkBorderRepresentation.h>
#include <vtkProperty2D.h>
#include <vtkPolyData.h>

// BSpline code
#include <Eigen/Dense>
#include "bspline.h"
#include "segmentpoints.h"


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    setupITK();
    setupVTK();
    setupStatusbar();

    ui->eraserRadioBox->click();

    ui->modeGroupBox->setEnabled(false);
}

MainWindow::~MainWindow() {
    delete ui;
}

/**
 * @brief sets up ITK (backend) part of the class
 */
void MainWindow::setupITK() {
    originalImage = InputImageType::New();

    outputImage = OutputImageType::New();
    connectedContourImage = OutputImageType::New();

    itk2vtkInputImageTypeConnector = itk2vtkInputImageConnectorType::New();
    itk2vtkOutputImageTypeConnector = itk2vtkOutputImageConnectorType::New();

    vesselnessProgressDialog = new QProgressDialog(this);
    gacProgressDialog = new QProgressDialog(this);
    segmentPointsProgressDialog = new QProgressDialog(this);

    connect(&vesselnessWatcher, SIGNAL(finished()), vesselnessProgressDialog, SLOT(cancel()));
    connect(&gacWatcher, SIGNAL(finished()), gacProgressDialog, SLOT(cancel()));
    connect(&segmentPointsWatcher, SIGNAL(finished()), segmentPointsProgressDialog, SLOT(cancel()));
}

/**
 * @brief sets up VTK (visualization) part of the class
 */
void MainWindow::setupVTK() {
    // Create an actor
    actor = vtkSmartPointer<vtkImageActor>::New();

    // Setup renderer
    renderer = vtkSmartPointer<vtkRenderer>::New();

    // Setup render window
    ui->qvtkWidget->GetRenderWindow()->AddRenderer(renderer);

    // Setup 2D interaction style
    rectangleStyle = vtkSmartPointer<vtkCustomInteractorStyleRubberBand2D>::New();
    pointStyle = vtkSmartPointer<vtkCustomInteractorStylePointPicker>::New();
    scribbleStyle = vtkSmartPointer<vtkInteractorStyleScribble>::New();

    // Get camera
    vtkCamera *cam = renderer->GetActiveCamera();

    // Make Y axis point downwards to align VTK and ITK coordinates
    cam->SetPosition(0, 0, -1);
    cam->SetFocalPoint(0, 0, 0);
    cam->SetViewUp(0, -1, 0);

    // To view the full bounds of your scene
    renderer->ResetCamera();

    /* ---------------------
     * VTK to Qt connections
     * ---------------------
     */

    // set up VTK to Qt connections
    vtk2QtConnections = vtkSmartPointer<vtkEventQtSlotConnect>::New();

    // update coords as we move through the window
    vtk2QtConnections->Connect(ui->qvtkWidget->GetRenderWindow()->GetInteractor(),
                               vtkCommand::MouseMoveEvent,
                               this,
                               SLOT(updateCoords(vtkObject*)));


    vtk2QtConnections->Connect(rectangleStyle,
                               vtkCommand::SelectionChangedEvent,
                               this,
                               SLOT(addRectangle(vtkObject *, unsigned long,
                                       void *, void *,
                                       vtkCommand *)));

    vtk2QtConnections->Connect(rectangleStyle,
                               vtkCommand::EndInteractionEvent,
                               this,
                               SLOT(redrawRectangles(vtkObject *, unsigned long,
                                       void *, void *,
                                       vtkCommand *)));

    vtk2QtConnections->Connect(scribbleStyle,
                               scribbleStyle->ScribbleEvent,
                               this,
                               SLOT(drawScribble(vtkObject *, unsigned long,
                                       void *, void *,
                                       vtkCommand *)));

    vtk2QtConnections->Connect(pointStyle,
                               vtkCommand::SelectionChangedEvent,
                               this,
                               SLOT(addPoint(vtkObject *, unsigned long,
                                       void *, void *,
                                       vtkCommand *)));
}

/**
 * @brief sets up status bar widgets
 *
 * @return void
 */
void MainWindow::setupStatusbar() {
    progressbar = new QProgressBar(this);
    progressbar->setVisible(false);

    coordLabel = new QLabel(this);

    ui->statusbar->addPermanentWidget(coordLabel);
    ui->statusbar->addPermanentWidget(progressbar);

    ui->statusbar->showMessage("Hello world!");
}


/**
 * @brief changes VTK interaction style and shows appropriate options
 * in the stacked widget
 */
void MainWindow::on_arrowRadioBox_clicked() {
    ui->stackedWidget->setCurrentIndex(1);
    ui->stackedWidget->setVisible(true);

    points.clear();

    // Start the computation.
    QFuture<void> future = QtConcurrent::run(this, &MainWindow::generateVesselness);
    this->vesselnessWatcher.setFuture(future);

    this->vesselnessProgressDialog->setMinimum(0);
    this->vesselnessProgressDialog->setMaximum(0);
    this->vesselnessProgressDialog->setWindowModality(Qt::WindowModal);
    this->vesselnessProgressDialog->exec();

    ui->qvtkWidget->GetInteractor()->SetInteractorStyle(pointStyle);

    scribbleStyle->SetCurrentRenderer(vtkSmartPointer<vtkRenderer>::New());
    scribbleStyle->InitializeTracer(vtkSmartPointer<vtkImageActor>::New());
}

/**
 * @brief changes VTK interaction style and shows appropriate options
 * in the stacked widget
 */
void MainWindow::on_regionRadioBox_clicked() {
    ui->qvtkWidget->GetInteractor()->SetInteractorStyle(rectangleStyle);

    scribbleStyle->SetCurrentRenderer(vtkSmartPointer<vtkRenderer>::New());
    scribbleStyle->InitializeTracer(vtkSmartPointer<vtkImageActor>::New());

    ui->stackedWidget->setCurrentIndex(0);
    ui->stackedWidget->setVisible(true);
}

/**
 * @brief changes VTK interaction style and shows appropriate options
 * in the stacked widget
 */
void MainWindow::on_pencilRadioBox_clicked() {
    ui->stackedWidget->setVisible(false);

    ui->qvtkWidget->GetInteractor()->SetInteractorStyle(scribbleStyle);
    scribbleStyle->SetCurrentRenderer(renderer);
    scribbleStyle->InitializeTracer(actor);
}

void MainWindow::on_eraserRadioBox_clicked() {
    ui->qvtkWidget->GetInteractor()->SetInteractorStyle(pointStyle);

    scribbleStyle->SetCurrentRenderer(vtkSmartPointer<vtkRenderer>::New());
    scribbleStyle->InitializeTracer(vtkSmartPointer<vtkImageActor>::New());

    ui->stackedWidget->setVisible(false);
}

/**
 * @brief starts the image processing
 */
void MainWindow::on_actionStart_triggered() {
    QFuture<void> future = QtConcurrent::run(this, &MainWindow::evolveContours);
    this->gacWatcher.setFuture(future);

    this->gacProgressDialog->setMinimum(0);
    this->gacProgressDialog->setMaximum(0);
    this->gacProgressDialog->setWindowModality(Qt::WindowModal);
    this->gacProgressDialog->exec();
}

void MainWindow::evolveContours() {



    typedef   itk::CurvatureAnisotropicDiffusionImageFilter<InputImageType, InputImageType>
    SmoothingFilterType;

    typedef   itk::GradientMagnitudeRecursiveGaussianImageFilter<InputImageType, InputImageType>
    GradientFilterType;

    typedef   itk::SigmoidImageFilter<InputImageType, InputImageType>
    SigmoidFilterType;

    typedef  itk::FastMarchingImageFilter<InputImageType, InputImageType>
    FastMarchingFilterType;

    typedef  itk::GeodesicActiveContourLevelSetImageFilter<InputImageType, InputImageType>
    GeodesicActiveContourFilterType;

    typedef itk::BinaryThresholdImageFilter<InputImageType, BinaryImageType> ThresholdingFilterType;

    typedef itk::ImageRegionIteratorWithIndex <OutputImageType> OutputIteratorType;

    typedef FastMarchingFilterType::NodeContainer  NodeContainer;
    typedef FastMarchingFilterType::NodeType       NodeType;


    outputImage = OutputImageType::New();
    outputImage->SetRegions(originalImage->GetLargestPossibleRegion());
    outputImage->Allocate();
    outputImage->FillBuffer(itk::NumericTraits<OutputPixelType>::max());


    SmoothingFilterType::Pointer smoothing = SmoothingFilterType::New();
    smoothing->SetInput(originalImage);
    smoothing->SetTimeStep(0.125);
    smoothing->SetNumberOfIterations(5);
    smoothing->SetConductanceParameter(9.0);

    GradientFilterType::Pointer  gradientMagnitude = GradientFilterType::New();
    gradientMagnitude->SetInput(smoothing->GetOutput());

    SigmoidFilterType::Pointer sigmoid = SigmoidFilterType::New();
    sigmoid->SetOutputMinimum(0.0);
    sigmoid->SetOutputMaximum(1.0);
    sigmoid->SetInput(gradientMagnitude->GetOutput());

    for (int i = 0; i < rectSeeds.size(); i++) {
        ThresholdingFilterType::Pointer thresholder = ThresholdingFilterType::New();

        thresholder->SetLowerThreshold(-1000.0);
        thresholder->SetUpperThreshold(0.0);

        thresholder->SetInsideValue(true);
        thresholder->SetOutsideValue(false);

        FastMarchingFilterType::Pointer  fastMarching = FastMarchingFilterType::New();

        GeodesicActiveContourFilterType::Pointer geodesicActiveContour =
            GeodesicActiveContourFilterType::New();
        geodesicActiveContour->SetPropagationScaling(rectSeeds[i].propagationScaling);
        geodesicActiveContour->SetCurvatureScaling(1.0);
        geodesicActiveContour->SetAdvectionScaling(1.0);

        geodesicActiveContour->SetMaximumRMSError(rectSeeds[i].maxRMSError);
        geodesicActiveContour->SetNumberOfIterations(rectSeeds[i].maxIterations);










        geodesicActiveContour->SetInput(fastMarching->GetOutput());
        geodesicActiveContour->SetFeatureImage(sigmoid->GetOutput());

        thresholder->SetInput(geodesicActiveContour->GetOutput());

        gradientMagnitude->SetSigma(1);

        sigmoid->SetAlpha(ui->sigmoidAlphaSpinBox->value());
        sigmoid->SetBeta(ui->sigmoidBetaSpinBox->value());


        NodeContainer::Pointer seed = NodeContainer::New();
        seed->Initialize();

        int seedNumber = 0;
        for (int y = 0; y < rectSeeds[i].area.height(); y++) {
            for (int x = 0; x < rectSeeds[i].area.width(); x++) {
                InputImageType::IndexType seedPosition;

                seedPosition[0] = rectSeeds[i].area.left() + x;
                seedPosition[1] = rectSeeds[i].area.top() + y;

                NodeType node;
                node.SetValue(-1);
                node.SetIndex(seedPosition);

                seed->InsertElement(seedNumber, node);
                ++seedNumber;
            }
        }

        fastMarching->SetTrialPoints(seed);
        fastMarching->SetSpeedConstant(1.0);
        fastMarching->SetOutputSize(originalImage->GetBufferedRegion().GetSize());

        qDebug() << "Processing " << i+1 << " of " << rectSeeds.size();
        try {
            thresholder->Update();
        } catch(itk::ExceptionObject & excep) {
            std::cerr << "Exception caught !" << std::endl;
            std::cerr << excep << std::endl;
        }

        std::cout << std::endl;
        std::cout << "Max. no. iterations: " << geodesicActiveContour->GetNumberOfIterations() << std::endl;
        std::cout << "Max. RMS error: " << geodesicActiveContour->GetMaximumRMSError() << std::endl;
        std::cout << std::endl;
        std::cout << "No. elpased iterations: " << geodesicActiveContour->GetElapsedIterations() << std::endl;
        std::cout << "RMS change: " << geodesicActiveContour->GetRMSChange() << std::endl;

        BinaryImageType::Pointer temp = thresholder->GetOutput();






        // Dilate the regions
        typedef itk::ImageRegionIteratorWithIndex <InputImageType> InputIteratorType;
        typedef itk::ImageRegionIteratorWithIndex <BinaryImageType> BinaryIteratorType;

        FastMarchingFilterType::Pointer fastMarching2 = FastMarchingFilterType::New();

        unsigned long seedNumber2 = 0;

        NodeContainer::Pointer seed2 = NodeContainer::New();
        seed2->Initialize();

        BinaryIteratorType iIt2(temp, temp->GetLargestPossibleRegion());

        iIt2.GoToBegin();
        while(!iIt2.IsAtEnd()) {
            if (iIt2.Get() == thresholder->GetInsideValue()) {
                NodeType node;
                //node.SetValue(-5);
                node.SetValue(-2.5);

                node.SetIndex(iIt2.GetIndex());

                seed2->InsertElement(seedNumber2++, node);
            }
            ++iIt2;
        }

        fastMarching2->SetTrialPoints(seed2);
        fastMarching2->SetSpeedConstant(1.0);
        fastMarching2->SetOutputSize(temp->GetBufferedRegion().GetSize());

        ThresholdingFilterType::Pointer thresholder2 = ThresholdingFilterType::New();

        thresholder2->SetLowerThreshold(-1000.0);
        thresholder2->SetUpperThreshold(0.0);

        thresholder2->SetInsideValue(true);
        thresholder2->SetOutsideValue(false);

        thresholder2->SetInput(fastMarching2->GetOutput());
        thresholder2->Update();

        temp = thresholder2->GetOutput();












        OutputPixelType regionPixelValue;
        regionPixelValue.SetRed(rectSeeds[i].color.red());
        regionPixelValue.SetGreen(rectSeeds[i].color.green());
        regionPixelValue.SetBlue(rectSeeds[i].color.blue());

        itk::ImageRegionIteratorWithIndex<BinaryImageType> iIt(temp, temp->GetLargestPossibleRegion());
        OutputIteratorType oIt(outputImage, outputImage->GetLargestPossibleRegion());

        iIt.GoToBegin();
        oIt.GoToBegin();

        while(!iIt.IsAtEnd() && !oIt.IsAtEnd()) {
            if (iIt.Get() == true) {
                oIt.Set(regionPixelValue);
            }
            ++iIt;
            ++oIt;
        }

        //rectSeeds[i].borderWidget->Off();

        updateImage(outputImage);

        qDebug() << "Done " << i+1 << " of " << rectSeeds.size();
    }











    OutputImageType::SizeType originalSize = outputImage->GetLargestPossibleRegion().GetSize();
    OutputImageType::IndexType cropStart;
    cropStart[0]  = axisOfSymmetryPoint.x();
    cropStart[1] = 0;

    OutputImageType::SizeType cropSize;
    cropSize[0] = originalSize[0] - axisOfSymmetryPoint.x();
    cropSize[1] = originalSize[1];

    OutputImageType::RegionType cropRegion(cropStart, cropSize);

    typedef itk::ExtractImageFilter< OutputImageType, OutputImageType > FilterType;
    FilterType::Pointer filter = FilterType::New();
    filter->SetExtractionRegion(cropRegion);
    filter->SetInput(outputImage);
    filter->SetDirectionCollapseToIdentity();

    try {
        filter->Update();
    } catch(itk::ExceptionObject & excep) {
        std::cerr << "Exception caught !" << std::endl;
        std::cerr << excep << std::endl;
    }











    OutputImageType::SizeType size = filter->GetOutput()->GetLargestPossibleRegion().GetSize();

    OutputImageType::IndexType origin;
    origin[0] = 0;
    origin[1] = 0;

    OutputImageType::SizeType newSize;
    newSize[0] = 2*size[0];
    newSize[1] = size[1];

    OutputImageType::RegionType region(origin, newSize);




    OutputImageType::Pointer finalImage = OutputImageType::New();
    finalImage->CopyInformation(filter->GetOutput());
    finalImage->SetRegions( region );
    finalImage->Allocate( );
    finalImage->FillBuffer( itk::NumericTraits<OutputPixelType>::max() );











    itk::FixedArray<bool, 2> flipAxes;
    flipAxes[0] = true;
    flipAxes[1] = false;

    typedef itk::FlipImageFilter <OutputImageType> FlipImageFilterType;
    FlipImageFilterType::Pointer flipFilter = FlipImageFilterType::New ();
    flipFilter->SetInput(filter->GetOutput());
    flipFilter->SetFlipAxes(flipAxes);
    try {
        flipFilter->Update();
    } catch(itk::ExceptionObject & excep) {
        std::cerr << "Exception caught !" << std::endl;
        std::cerr << excep << std::endl;
    }






    // mirrored part
    {
        OutputImageType::IndexType index;
        index.Fill(0);

        OutputImageType::SizeType size = flipFilter->GetOutput()->GetLargestPossibleRegion().GetSize();

        OutputImageType::RegionType region(index, size);

        //itk::ImageRegionIteratorWithIndex<OutputImageType> oIt(finalImage, flipFilter->GetOutput()->GetLargestPossibleRegion());
        itk::ImageRegionIteratorWithIndex<OutputImageType> oIt(finalImage, region);
        itk::ImageRegionIteratorWithIndex<OutputImageType> iIt(flipFilter->GetOutput(), flipFilter->GetOutput()->GetLargestPossibleRegion());

        iIt.GoToBegin();
        oIt.GoToBegin();

        while(!iIt.IsAtEnd() && !oIt.IsAtEnd()) {
            oIt.Set(iIt.Get());
            ++iIt;
            ++oIt;
        }

    }



    // not mirrored part
    {

        OutputImageType::SizeType croppedSize = filter->GetOutput()->GetLargestPossibleRegion().GetSize();

        OutputImageType::IndexType origin;
        origin[0] = croppedSize[0];
        origin[1] = 0;

        OutputImageType::SizeType size;
        size[0] = croppedSize[0];
        size[1] = croppedSize[1];

        OutputImageType::RegionType region(origin, size);


        itk::ImageRegionIteratorWithIndex<OutputImageType> oIt(finalImage, region);
        itk::ImageRegionIteratorWithIndex<OutputImageType> iIt(filter->GetOutput(), filter->GetOutput()->GetLargestPossibleRegion());


        iIt.GoToBegin();
        oIt.GoToBegin();

        while(!iIt.IsAtEnd() && !oIt.IsAtEnd()) {
            if (iIt.Get() != itk::NumericTraits<OutputPixelType>::max()) {
                oIt.Set(iIt.Get());
            }
            ++iIt;
            ++oIt;
        }

    }










    typedef itk::ImageFileWriter<OutputImageType> WriterType;
    WriterType::Pointer writer = WriterType::New();

    //writer->SetInput(outputImage);
    writer->SetInput(finalImage);
    writer->SetFileName("out.png");

    try {
        writer->Update();
    } catch(itk::ExceptionObject & excep) {
        std::cerr << "Exception caught !" << std::endl;
        std::cerr << excep << std::endl;
    }

    qDebug() << "Done!";


}

/**
 * @brief opens the image to be processed
 */
void MainWindow::on_actionOpen_triggered() {
    // get file path using a file dialog
    const QString path = QFileDialog::getOpenFileName(this);

    // if user canceled the dialog, path will be null
    if (path.isNull()) {
        qDebug() << tr("No file selected");
    }

    // else, user has selected a file
    else {
        qDebug() << tr("Opening file %1").arg(path);

        typedef itk::ImageFileReader<InputImageType> ReaderType;

        ReaderType::Pointer reader = ReaderType::New();
        reader->SetFileName(path.toStdString());

        // try to read the file and see if it is a valid image
        try {
            reader->Update();
        }

        // if the file is not a valid image, show an error message
        // and allow the user to choose an image again
        catch(itk::ExceptionObject & e) {
            const QMessageBox::StandardButton choice = QMessageBox::critical(
                        this,
                        tr("Error"),
                        tr("Invalid image file. Do you want to select another file?"),
                        QMessageBox::Yes | QMessageBox::No);

            // if user selects "Yes," call this function recursively to ask
            // the user again; after that, exit the function because the
            // recursively called function should have visualized image already
            if (choice == QMessageBox::Yes) {
                on_actionOpen_triggered();
                return;
            }
        }

        originalImage = reader->GetOutput();

        // show the image in the VTKWidget
        updateImage(originalImage);

        // enable the mode mode group box
        ui->modeGroupBox->setEnabled(true);

        // set mode to eraser
        ui->eraserRadioBox->click();
    }
}


/**
 * @brief shows an image in the qvtkWidget
 *
 * @param image ITK image pointer to be visualized
 */
void MainWindow::updateImage(InputImageType::Pointer image) {
    // convert ITK image to VTK image
    itk2vtkInputImageTypeConnector = itk2vtkInputImageConnectorType::New();
    itk2vtkInputImageTypeConnector->SetInput(image);

    try {
        itk2vtkInputImageTypeConnector->Update();
    }

    catch(itk::ExceptionObject & e) {
        qDebug() << "Could not convert ITK image to VTK image.";

        qDebug() << "ExceptionObject caught !";
        qDebug() << e.what();

        return;
    }

    // connect the converted image to the actor
    actor->GetMapper()->SetInputData(itk2vtkInputImageTypeConnector->GetOutput());

    // if there is no actor in the renderer, add the actor to the renderer
    if (renderer->GetActors()->GetNumberOfItems() == 0) {
        renderer->AddActor(actor);
    }

    // reset the camera to view the entire image and update the widget
    renderer->ResetCamera();
    ui->qvtkWidget->update();
}

void MainWindow::updateImage(OutputImageType::Pointer image) {
    itk2vtkOutputImageTypeConnector = itk2vtkOutputImageConnectorType::New();

    // convert ITK image to VTK image
    itk2vtkOutputImageTypeConnector->SetInput(image);

    try {
        itk2vtkOutputImageTypeConnector->Update();
    }

    catch(itk::ExceptionObject & e) {
        qDebug() << "Could not convert ITK image to VTK image.";

        qDebug() << "ExceptionObject caught !";
        qDebug() << e.what();

        return;
    }

    // connect the converted image to the actor
    actor->GetMapper()->SetInputData(itk2vtkOutputImageTypeConnector->GetOutput());

    // if there is no actor in the renderer, add the actor to the renderer
    if (renderer->GetActors()->GetNumberOfItems() == 0) {
        renderer->AddActor(actor);
    }

    // reset the camera to view the entire image and update the widget
    renderer->ResetCamera();
    ui->qvtkWidget->update();
}


void MainWindow::updateCoords(vtkObject* obj) {
    // get interactor
    vtkRenderWindowInteractor* iren = vtkRenderWindowInteractor::SafeDownCast(obj);

    // get event position
    int event_pos[2];
    iren->GetEventPosition(event_pos);

    QPointF world_pos = displayToWorld(event_pos[0], event_pos[1]);

    // update label
    coordLabel->setText( tr("Coordinates: x=%1, y=%2")
                         .arg((long) world_pos.x())
                         .arg((long) world_pos.y()) );
}

void MainWindow::redrawRectangles(vtkObject* caller, long unsigned int, void* clientData, void* callData, vtkCommand* command) {
    qDebug() << tr("Redrawing rectangles");

    for (int i = 0; i < rectSeeds.size(); i++) {
        rectSeeds[i].borderWidget = vtkSmartPointer<vtkBorderWidget>::New();

        vtkBorderWidget *borderWidget = rectSeeds[i].borderWidget;

        borderWidget->SetInteractor(ui->qvtkWidget->GetInteractor());
        borderWidget->CreateDefaultRepresentation();
        static_cast<vtkBorderRepresentation*>(borderWidget->GetRepresentation())->GetBorderProperty()->SetColor(0,0,1);

        const QRectF r = rectSeeds[i].area;
        const QRectF p = QRectF( worldToNormalizedDisplay(r.left(), r.top()),
                                 worldToNormalizedDisplay(r.right(), r.bottom()) ).normalized();

        vtkCoordinate* lowerLeftCoordinate = static_cast<vtkBorderRepresentation*>(
                borderWidget->GetRepresentation())->GetPositionCoordinate();
        lowerLeftCoordinate->SetValue(p.left(), p.top());

        vtkCoordinate* upperRightCoordinate = static_cast<vtkBorderRepresentation*>(
                borderWidget->GetRepresentation())->GetPosition2Coordinate();
        upperRightCoordinate->SetValue(p.width(), p.height());

        borderWidget->On();

        vtk2QtConnections->Connect(borderWidget,
                                   vtkCommand::StartInteractionEvent,
                                   //vtkCommand::EndInteractionEvent,
                                   this,
                                   SLOT(selectRectangularRegions(vtkObject *, unsigned long,
                                           void *, void *,
                                           vtkCommand *)));

        ui->qvtkWidget->GetInteractor()->Render();
        ui->qvtkWidget->update();
    }
}

void MainWindow::addRectangle(vtkObject* caller, long unsigned int, void* clientData, void* callData, vtkCommand* command) {
    unsigned int* rect = reinterpret_cast<unsigned int*> ( callData );
    unsigned int pos1X = rect[0];
    unsigned int pos1Y = rect[1];
    unsigned int pos2X = rect[2];
    unsigned int pos2Y = rect[3];

    if (pos1X == pos2X || pos1Y == pos2Y) {
        qDebug() << "Not adding a rectangle of zero height or width";
    } else {
        static unsigned int id = 0;
        rectSeed tempSeed(id++);
        tempSeed.area = QRectF( displayToWorld(pos1X, pos1Y),
                                displayToWorld(pos2X, pos2Y));
        rectSeeds.push_back(tempSeed);

        redrawRectangles(caller, vtkCommand::RenderEvent, NULL, NULL, NULL);
    }
}

void MainWindow::drawScribble(vtkObject* caller, long unsigned int, void* clientData, void* callData, vtkCommand* command) {
    vtkPoints *points = scribbleStyle->GetSelection();

    if (points->GetNumberOfPoints() > 1) {
        double prevPos[3];
        points->GetPoint(0, prevPos);

        for (int i = 1; i < points->GetNumberOfPoints(); i++) {
            double pos[3];
            points->GetPoint(i, pos);

            InputImageType::IndexType index;
            index[0] = pos[0];
            index[1] = pos[1];

            InputImageType::IndexType prevIndex;
            prevIndex[0] = prevPos[0];
            prevIndex[1] = prevPos[1];

            itk::LineIterator<InputImageType> it(originalImage, index, prevIndex);
            it.GoToBegin();

            while (!it.IsAtEnd()) {
                it.Set(itk::NumericTraits<InputPixelType>::Zero);

                ++it;
            }

            for (int n = 0; n < 3; n++) {
                prevPos[n] = pos[n];
            }

            qDebug() << "Finished point " << i;
        }
    }

    std::cout << "There are " << points->GetNumberOfPoints() << " points in the selection." << std::endl;

    updateImage(originalImage);

}


void MainWindow::selectRectangularRegions(vtkObject * caller, unsigned long, void * clientData, void *callData, vtkCommand * command) {
    vtkBorderWidget* borderWidget = reinterpret_cast<vtkBorderWidget*> ( caller );

    const bool erase = ui->eraserRadioBox->isChecked();

    for (QVector<rectSeed>::iterator it = rectSeeds.begin(); it != rectSeeds.end(); it++) {
        if (it->borderWidget == borderWidget) {
            if (erase) {
                qDebug() << tr("Removing seed");
                it = rectSeeds.erase(it);
                selectedRectSeed = NULL;
            } else {
                selectedRectSeed = it;
                qDebug() << tr("Selecting seed");
            }
            break;
        }
    }

    showSelectedRectSeedInfo();

    ui->qvtkWidget->update();
}

void MainWindow::showSelectedRectSeedInfo() {
    if (selectedRectSeed != NULL) {
        ui->regionLabelLineEdit->setText(selectedRectSeed->label);
        ui->gacPropScalingSpinBox->setValue(selectedRectSeed->propagationScaling);
        ui->gacMaxRMSErrorSpinBox->setValue(selectedRectSeed->maxRMSError);
        ui->gacMaxIterationSpinBox->setValue(selectedRectSeed->maxIterations);

        QPalette pal = ui->colorPushButton->palette();
        pal.setColor(ui->colorPushButton->backgroundRole(), selectedRectSeed->color);
        ui->colorPushButton->setPalette(pal);
    }
}

void MainWindow::addPoint(vtkObject* caller, long unsigned int, void* clientData, void* callData, vtkCommand* command) {
    unsigned int* point = reinterpret_cast<unsigned int*> ( callData );


    if (ui->arrowRadioBox->isChecked() || ui->brokenContourRadioBox->isChecked()) {
        points.push_back(displayToWorld(point[0], point[1]));
        qDebug() << tr("Point added");
    }
    else if (ui->symmetryRadioButton->isChecked()) {
        qDebug() << tr("Axis of symmetry");
        axisOfSymmetryPoint = displayToWorld(point[0], point[1]);

        auto DrawLine = [] (InputImageType::Pointer image, const unsigned int &x1, const unsigned int &y1, const unsigned int &x2,const unsigned int &y2, const InputPixelType &value) {
            InputImageType::IndexType corner1;
            corner1[0] = x1;
            corner1[1] = y1;

            InputImageType::IndexType corner2;
            corner2[0] = x2;
            corner2[1] = y2;

            itk::LineIterator<InputImageType> it1(image, corner1, corner2);
            it1.GoToBegin();
            while (!it1.IsAtEnd()) {
                it1.Set(value);
                ++it1;
            }
        };

        InputImageType::SizeType size = originalImage->GetLargestPossibleRegion().GetSize();
        DrawLine(originalImage, (unsigned int) axisOfSymmetryPoint.x(), 0, (unsigned int) axisOfSymmetryPoint.x(), size[1]-1, 0.0);
    }
    updateImage(originalImage);
}

QPointF MainWindow::displayToWorld(double x, double y) {
    vtkSmartPointer<vtkCoordinate> point = vtkSmartPointer<vtkCoordinate>::New();
    point->SetCoordinateSystemToDisplay();
    point->SetValue(x, y);

    double *world = point->GetComputedWorldValue(renderer);

    return QPointF(world[0], world[1]);
}

QPointF MainWindow::worldToDisplay(double x, double y) {
    vtkSmartPointer<vtkCoordinate> point = vtkSmartPointer<vtkCoordinate>::New();
    point->SetCoordinateSystemToWorld();
    point->SetValue(x, y);

    double *display = point->GetComputedDoubleDisplayValue(renderer);

    return QPointF(display[0], display[1]);
}


QPointF MainWindow::worldToNormalizedDisplay(double x, double y) {
    QPointF world = worldToDisplay(x, y);

    double point[2] = {world.x(), world.y()};
    renderer->DisplayToNormalizedDisplay(point[0], point[1]);

    return QPointF(point[0], point[1]);
}

void MainWindow::generateVesselness() {
    typedef itk::SymmetricSecondRankTensor< InputPixelType, 2 > HessianPixelType;
    typedef itk::Image< HessianPixelType, 2 >           HessianImageType;
    typedef itk::HessianToObjectnessMeasureImageFilter< HessianImageType, InputImageType >
    ObjectnessFilterType;
    ObjectnessFilterType::Pointer objectnessFilter = ObjectnessFilterType::New();
    objectnessFilter->SetBrightObject( false );
    objectnessFilter->SetScaleObjectnessMeasure( false );
    objectnessFilter->SetAlpha( 0.5 );
    objectnessFilter->SetBeta( 1.0 );
    objectnessFilter->SetGamma( 5.0 );

    typedef itk::MultiScaleHessianBasedMeasureImageFilter< InputImageType, HessianImageType, InputImageType >
    MultiScaleEnhancementFilterType;
    MultiScaleEnhancementFilterType::Pointer multiScaleEnhancementFilter =
        MultiScaleEnhancementFilterType::New();

    multiScaleEnhancementFilter->SetInput( originalImage );
    multiScaleEnhancementFilter->SetHessianToMeasureFilter( objectnessFilter );
    multiScaleEnhancementFilter->SetSigmaStepMethodToLogarithmic();
    multiScaleEnhancementFilter->SetSigmaMinimum( ui->vesselnessSigmaMinSpinBox->value() );
    multiScaleEnhancementFilter->SetSigmaMaximum( ui->vesselnessSigmaMaxSpinBox->value() );
    multiScaleEnhancementFilter->SetNumberOfSigmaSteps( ui->vesselnessStepsSpinBox->value() );

    typedef itk::RescaleIntensityImageFilter< InputImageType, InputImageType >
    RescaleFilterType;
    RescaleFilterType::Pointer rescaleFilter = RescaleFilterType::New();
    rescaleFilter->SetInput( multiScaleEnhancementFilter->GetOutput() );
    rescaleFilter->SetOutputMinimum(0);
    rescaleFilter->SetOutputMaximum(255);

    try {
        rescaleFilter->Update();
    } catch( itk::ExceptionObject & error ) {
        qDebug() << "Error: " << error.what();
    }

    vesselnessImage = rescaleFilter->GetOutput();

    updateImage(vesselnessImage);

    typedef  itk::ImageFileWriter<  InputImageType  > WriterType3;
    WriterType3::Pointer writer3 = WriterType3::New();

    writer3->SetInput(vesselnessImage);
    writer3->SetFileName("vesselness.vtk");

    try {
        writer3->Update();
    } catch( itk::ExceptionObject & excep ) {
        std::cerr << "Exception caught !" << std::endl;
        std::cerr << excep << std::endl;
    }
}

void MainWindow::on_detectArrowButton_clicked() {
    typedef itk::PolyLineParametricPath< 2 > PathType;
    typedef itk::SpeedFunctionToPathFilter< InputImageType, PathType > PathFilterType;
    typedef typename PathFilterType::CostFunctionType::CoordRepType CoordRepType;
    typedef itk::PathIterator< InputImageType, PathType > PathIteratorType;

    try {
        const float TerminationValue = ui->mpTerminationValueSpinBox->value();
        unsigned int NumberOfIterations = 100000;
        float StepLengthFactor = ui->mpStepLengthFactorSpinBox->value();
        float StepLengthRelax = ui->mpStepLengthRelaxSpinBox->value();

        // Compute the minimum spacing
        typename InputImageType::SpacingType spacing = vesselnessImage->GetSpacing();
        double minspacing = spacing[0];
        for (unsigned int dim=0; dim<2; dim++) {
            if (spacing[dim] < minspacing) minspacing = spacing[dim];
        }

        // Create Interpolator
        typedef itk::LinearInterpolateImageFunction<InputImageType, CoordRepType>
        InterpolatorType;
        typename InterpolatorType::Pointer interp = InterpolatorType::New();

        // Create Cost Function
        typename PathFilterType::CostFunctionType::Pointer cost =
            PathFilterType::CostFunctionType::New();
        cost->SetInterpolator( interp );

        // Create RegularStepGradientDescentOptimizer
        typedef itk::RegularStepGradientDescentOptimizer OptimizerType;
        typename OptimizerType::Pointer optimizer = OptimizerType::New();
        optimizer->SetNumberOfIterations( NumberOfIterations );
        optimizer->SetMaximumStepLength( 1.0*StepLengthFactor*minspacing );
        optimizer->SetMinimumStepLength( 0.5*StepLengthFactor*minspacing );
        optimizer->SetRelaxationFactor( StepLengthRelax );

        // Create path filter
        typename PathFilterType::Pointer pathFilter = PathFilterType::New();
        pathFilter->SetInput( vesselnessImage );
        pathFilter->SetCostFunction( cost );
        pathFilter->SetOptimizer( optimizer );
        pathFilter->SetTerminationValue( TerminationValue );

        typename PathFilterType::PathInfo info;
        typename PathFilterType::PathInfo info_reversed;

        for (int i = 0; i < points.size(); i++) {
            PathFilterType::PointType point;

            point[0] = points[i].x();
            point[1] = points[i].y();

            if (i == 0) {
                info.SetStartPoint(point);
                info_reversed.SetEndPoint(point);
                qDebug() << tr("Added x=%1, y=%2 as start point").arg(point[0]).arg(point[1]);
            } else if (i != points.size()-1) {
                info.AddWayPoint(point);
                info_reversed.AddWayPoint(point);
                qDebug() << tr("Added x=%1, y=%2 as mid point").arg(point[0]).arg(point[1]);
            } else {
                info.SetEndPoint(point);
                info_reversed.SetStartPoint(point);
                qDebug() << tr("Added x=%1, y=%2 as end point").arg(point[0]).arg(point[1]);
            }
        }

        pathFilter->AddPathInfo(info);
        pathFilter->AddPathInfo(info_reversed);

        points.clear();

        // Compute the path
        qDebug() << "Computing path...";
        pathFilter->Update( );

        // Allocate output image
        typename InputImageType::Pointer output = InputImageType::New();
        output->SetRegions( vesselnessImage->GetLargestPossibleRegion() );
        output->SetSpacing( vesselnessImage->GetSpacing() );
        output->SetOrigin( vesselnessImage->GetOrigin() );
        output->Allocate( );
        output->FillBuffer( itk::NumericTraits<InputPixelType>::Zero );

        // Rasterize path
        for (unsigned int i=0; i<pathFilter->GetNumberOfOutputs(); i++) {
            // Get the path
            typename PathType::Pointer path = pathFilter->GetOutput( i );

            // Check path is valid
            if ( path->GetVertexList()->Size() == 0 ) {
                qDebug() << "WARNING: Path " << (i+1) << " contains no points!";
                continue;
            }

            // Iterate path and convert to image
            qDebug() << "Rasterizing path...";
            PathIteratorType it( output, path );
            for (it.GoToBegin(); !it.IsAtEnd(); ++it) {
                it.Set( itk::NumericTraits<InputPixelType>::max() );
            }
        }

        pathImage = output;

    }

    catch (itk::ExceptionObject & err) {
        qDebug() << "ExceptionObject caught !";
        qDebug() << err.what();

        const QMessageBox::StandardButton choice = QMessageBox::critical(
                    this,
                    tr("Error"),
                    tr(err.what()),
                    QMessageBox::Yes | QMessageBox::No);

        return;
    }

    qDebug() << "Done minimalpath";



    // remove the arrows
    typedef itk::ImageRegionIteratorWithIndex <InputImageType> InputIteratorType;

    typedef  itk::FastMarchingImageFilter<InputImageType, InputImageType> FastMarchingFilterType;
    typedef FastMarchingFilterType::NodeContainer  NodeContainer;
    typedef FastMarchingFilterType::NodeType       NodeType;

    FastMarchingFilterType::Pointer fastMarching = FastMarchingFilterType::New();

    unsigned long seedNumber = 0;

    NodeContainer::Pointer seed = NodeContainer::New();
    seed->Initialize();

    InputIteratorType iIt(pathImage, pathImage->GetLargestPossibleRegion());

    iIt.GoToBegin();
    while(!iIt.IsAtEnd()) {
        if (iIt.Get() == itk::NumericTraits<InputPixelType>::max()) {
            NodeType node;
            node.SetValue(-1);
            //node.SetValue(1);

            node.SetIndex(iIt.GetIndex());

            seed->InsertElement(seedNumber++, node);
        }
        ++iIt;
    }

    fastMarching->SetTrialPoints(seed);
    fastMarching->SetSpeedConstant(1.0);
    fastMarching->SetOutputSize(originalImage->GetBufferedRegion().GetSize());


    typedef itk::BinaryThresholdImageFilter<
    InputImageType,
    BinaryImageType    >       ThresholdingFilterType;

    ThresholdingFilterType::Pointer thresholder = ThresholdingFilterType::New();

    thresholder->SetLowerThreshold( -1000.0 );
    thresholder->SetUpperThreshold(     0.0 );

    thresholder->SetOutsideValue(  false  );
    thresholder->SetInsideValue(  true );



    typedef itk::RescaleIntensityImageFilter<
    InputImageType,
    OutputImageType >   CastFilterType;


    typedef   itk::CurvatureAnisotropicDiffusionImageFilter<
    InputImageType,
    InputImageType >  SmoothingFilterType;

    SmoothingFilterType::Pointer smoothing = SmoothingFilterType::New();


    typedef   itk::GradientMagnitudeRecursiveGaussianImageFilter<
    InputImageType,
    InputImageType >  GradientFilterType;
    typedef   itk::SigmoidImageFilter<
    InputImageType,
    InputImageType >  SigmoidFilterType;

    GradientFilterType::Pointer  gradientMagnitude = GradientFilterType::New();

    SigmoidFilterType::Pointer sigmoid = SigmoidFilterType::New();

    sigmoid->SetOutputMinimum(  0.0  );
    sigmoid->SetOutputMaximum(  1.0  );

    sigmoid->SetAlpha(-0.5);
    sigmoid->SetBeta(3.00);

    typedef  itk::GeodesicActiveContourLevelSetImageFilter< InputImageType,
             InputImageType >    GeodesicActiveContourFilterType;
    GeodesicActiveContourFilterType::Pointer geodesicActiveContour =
        GeodesicActiveContourFilterType::New();

    geodesicActiveContour->SetPropagationScaling( 2.0 );
    geodesicActiveContour->SetCurvatureScaling( 1.0 );
    geodesicActiveContour->SetAdvectionScaling( 1.0 );

    geodesicActiveContour->SetMaximumRMSError( 0.05 );
    geodesicActiveContour->SetNumberOfIterations( 100 );

    smoothing->SetInput( originalImage );
    gradientMagnitude->SetInput( smoothing->GetOutput() );
    sigmoid->SetInput( gradientMagnitude->GetOutput() );

    geodesicActiveContour->SetInput(  fastMarching->GetOutput() );
    geodesicActiveContour->SetFeatureImage( vesselnessImage );

    thresholder->SetInput( geodesicActiveContour->GetOutput() );

    try {
        thresholder->Update();
    } catch( itk::ExceptionObject & excep ) {
        std::cerr << "Exception caught !" << std::endl;
        std::cerr << excep << std::endl;
    }




    typedef itk::BinaryThresholdImageFilter<
    InputImageType,
    itk::Image<unsigned char, 2>    >       ThresholdingFilterType2;

    ThresholdingFilterType2::Pointer thresholder2 = ThresholdingFilterType2::New();

    thresholder2->SetLowerThreshold( -1000.0 );
    thresholder2->SetUpperThreshold(     0.0 );

    thresholder2->SetOutsideValue(  0  );
    thresholder2->SetInsideValue(  255 );

    thresholder2->SetInput(geodesicActiveContour->GetOutput());


    typedef  itk::ImageFileWriter<  itk::Image<unsigned char, 2>  > WriterType2;
    WriterType2::Pointer writer2 = WriterType2::New();

    writer2->SetInput(thresholder2->GetOutput());
    writer2->SetFileName("line.png");

    try {
        writer2->Update();
    } catch( itk::ExceptionObject & excep ) {
        std::cerr << "Exception caught !" << std::endl;
        std::cerr << excep << std::endl;
    }


    typedef  itk::ImageFileWriter<  InputImageType  > WriterType3;
    WriterType3::Pointer writer3 = WriterType3::New();

    writer3->SetInput(fastMarching->GetOutput());
    writer3->SetFileName("line.vtk");

    try {
        writer3->Update();
    } catch( itk::ExceptionObject & excep ) {
        std::cerr << "Exception caught !" << std::endl;
        std::cerr << excep << std::endl;
    }

    std::cout << std::endl;
    std::cout << "Max. no. iterations: " << geodesicActiveContour->GetNumberOfIterations() << std::endl;
    std::cout << "Max. RMS error: " << geodesicActiveContour->GetMaximumRMSError() << std::endl;
    std::cout << std::endl;
    std::cout << "No. elpased iterations: " << geodesicActiveContour->GetElapsedIterations() << std::endl;
    std::cout << "RMS change: " << geodesicActiveContour->GetRMSChange() << std::endl;


    WriterType3::Pointer writer3_2 = WriterType3::New();

    writer3_2->SetInput(pathImage);
    writer3_2->SetFileName("path.vtk");

    try {
        writer3_2->Update();
    } catch( itk::ExceptionObject & excep ) {
        std::cerr << "Exception caught !" << std::endl;
        std::cerr << excep << std::endl;
    }





















    // ADDITIONAL LOGIC

    // radius of the window
    BinaryImageType::Pointer temp_line = thresholder->GetOutput();

    OutputImageType::SizeType radius;
    radius[0] = 15;
    radius[1] = 15;
    itk::NeighborhoodIterator<InputImageType> iterator(radius, originalImage, originalImage->GetLargestPossibleRegion());
    itk::NeighborhoodIterator<BinaryImageType> iterator_line(radius, temp_line, temp_line->GetLargestPossibleRegion());

    while(!iterator.IsAtEnd() && !iterator_line.IsAtEnd()) {
        if (iterator_line.GetCenterPixel() == thresholder->GetInsideValue()) {
            std::vector<int> neighbors;

            for (int i = 0; i < (radius[0]*2+1)*(radius[1]*2+1); i++) {
                if (iterator_line.GetPixel(i) == thresholder->GetOutsideValue()) {
                    neighbors.push_back(iterator.GetPixel(i));
                }
            }

            std::sort(neighbors.begin(), neighbors.end());

            size_t size = neighbors.size();
            if(size != 0) {
                int median;

                if (size  % 2 == 0) {
                    median = (neighbors[size / 2 - 1] + neighbors[size / 2]) / 2;
                } else {
                    median = neighbors[size / 2];
                }
                iterator.SetCenterPixel(median);
            }
        }
        ++iterator; ++iterator_line;
    }

    updateImage(originalImage);


    typedef itk::Image<unsigned char, 2> PNGImageType;
    typedef itk::CastImageFilter< InputImageType, PNGImageType > FilterType;
    FilterType::Pointer filter = FilterType::New();
    filter->SetInput( originalImage );

    //typedef itk::ImageFileWriter<InputImageType> WriterType;
    typedef itk::ImageFileWriter<PNGImageType> WriterType;
    WriterType::Pointer writer = WriterType::New();

    //writer->SetInput(originalImage);
    writer->SetInput(filter->GetOutput());
    writer->SetFileName("cleaned.png");

    try {
        writer->Update();
    } catch(itk::ExceptionObject & excep) {
        std::cerr << "Exception caught !" << std::endl;
        std::cerr << excep << std::endl;
    }
}

void MainWindow::on_applyLSButton_clicked() {
    selectedRectSeed->label = ui->regionLabelLineEdit->text();
    selectedRectSeed->propagationScaling = ui->gacPropScalingSpinBox->value();
    selectedRectSeed->maxRMSError = ui->gacMaxRMSErrorSpinBox->value();
    selectedRectSeed->maxIterations = ui->gacMaxIterationSpinBox->value();

    selectedRectSeed->color = ui->colorPushButton->palette().color(ui->colorPushButton->backgroundRole());
}

void MainWindow::on_colorPushButton_clicked() {
    static QColor lastColor(Qt::red);
    QColor currentColor = QColorDialog::getColor(lastColor);

    if (currentColor.isValid()) {
        QPalette pal = ui->colorPushButton->palette();
        pal.setColor(ui->colorPushButton->backgroundRole(), currentColor);
        ui->colorPushButton->setPalette(pal);

        lastColor = currentColor;
    }
}

void MainWindow::on_symmetryRadioButton_clicked() {
    ui->qvtkWidget->GetInteractor()->SetInteractorStyle(pointStyle);

    scribbleStyle->SetCurrentRenderer(vtkSmartPointer<vtkRenderer>::New());
    scribbleStyle->InitializeTracer(vtkSmartPointer<vtkImageActor>::New());
}

void MainWindow::findSegmentEndPoints() {
    generateVesselness();





    typedef itk::BinaryThresholdImageFilter <InputImageType, InputImageType> BinaryThresholdImageFilterType;
    BinaryThresholdImageFilterType::Pointer thresholdFilter = BinaryThresholdImageFilterType::New();
    thresholdFilter->SetInput(vesselnessImage);
    thresholdFilter->SetLowerThreshold(1);
    thresholdFilter->SetUpperThreshold(255);
    thresholdFilter->SetInsideValue(255);
    thresholdFilter->SetOutsideValue(0);








    typedef itk::Image<size_t, 2> LabelImageType;
    typedef itk::ConnectedComponentImageFilter <InputImageType, LabelImageType > ConnectedComponentImageFilterType;
    ConnectedComponentImageFilterType::Pointer connected = ConnectedComponentImageFilterType::New();
    //connected->SetInput(vesselnessImage);
    connected->SetInput(thresholdFilter->GetOutput());

    try {
        connected->Update();
    } catch(itk::ExceptionObject & excep) {
        std::cerr << "Exception caught !" << std::endl;
        std::cerr << excep << std::endl;
    }

    auto segments = getSegments(connected->GetOutput());
    //qDebug() << "Number of segments" << segments.size();

    brokenContourEndPoints = getSegmentEndPoints(segments);
    //qDebug() << "Number of end point pairs" << brokenContourEndPoints.size();

    brokenContourConnectivity = findConnections(brokenContourEndPoints, ui->bucketsizeSpinBox->value());
    //qDebug() << "Number of connectivity pairs" << brokenContourConnectivity.size();










    itk::RGBPixel<unsigned char> black;
    black.SetRed(0);
    black.SetGreen(0);
    black.SetBlue(0);

    itk::RGBPixel<unsigned char> white;
    white.SetRed(255);
    white.SetGreen(255);
    white.SetBlue(255);

    itk::RGBPixel<unsigned char> red;
    red.SetRed(255);
    red.SetGreen(0);
    red.SetBlue(0);

    // draw the image
    connectedContourImage->CopyInformation(originalImage);
    connectedContourImage->SetRegions( originalImage->GetLargestPossibleRegion() );
    connectedContourImage->Allocate();
    connectedContourImage->FillBuffer(black);



    auto DrawLine = [] (OutputImageType::Pointer image, const unsigned int &x1, const unsigned int &y1, const unsigned int &x2,const unsigned int &y2, itk::RGBPixel<unsigned char> &value) {
        OutputImageType::IndexType corner1;
        corner1[0] = x1;
        corner1[1] = y1;

        OutputImageType::IndexType corner2;
        corner2[0] = x2;
        corner2[1] = y2;

        itk::LineIterator<OutputImageType> it1(image, corner1, corner2);
        it1.GoToBegin();
        while (!it1.IsAtEnd()) {
            it1.Set(value);
            ++it1;
        }
    };

    // draw segments in white
    for (int i = 0; i < brokenContourEndPoints.size(); i++) {
        //qDebug() << "Segment " << i << ": " << distance(finalSegments[i].first, finalSegments[i].second);
        const int x1 = brokenContourEndPoints[i].first.x(),
                  y1 = brokenContourEndPoints[i].first.y(),
                  x2 = brokenContourEndPoints[i].second.x(),
                  y2 = brokenContourEndPoints[i].second.y();

        QRect rect;
        rect.setCoords(x1, y1, x2, y2);

        DrawLine(connectedContourImage, rect.left(), rect.top(), rect.right(), rect.bottom(), white);
    }

    // draw connections in red
    for (int i = 0; i < brokenContourConnectivity.size(); i++) {
        //qDebug() << "Segment " << i << ": " << distance(finalSegments[i].first, finalSegments[i].second);
        const int x1 = brokenContourConnectivity[i].first.x(),
                  y1 = brokenContourConnectivity[i].first.y(),
                  x2 = brokenContourConnectivity[i].second.x(),
                  y2 = brokenContourConnectivity[i].second.y();

        QRect rect;
        rect.setCoords(x1, y1, x2, y2);

        DrawLine(connectedContourImage, rect.left(), rect.top(), rect.right(), rect.bottom(), red);
    }

    updateImage(connectedContourImage);
}

void MainWindow::on_brokenContourRadioBox_clicked() {
    ui->stackedWidget->setCurrentIndex(2);
    ui->stackedWidget->setVisible(true);

    points.clear();

    // Start the computation.
    QFuture<void> future = QtConcurrent::run(this, &MainWindow::findSegmentEndPoints);
    this->segmentPointsWatcher.setFuture(future);

    this->segmentPointsProgressDialog->setMinimum(0);
    this->segmentPointsProgressDialog->setMaximum(0);
    this->segmentPointsProgressDialog->setWindowModality(Qt::WindowModal);
    this->segmentPointsProgressDialog->exec();

    ui->qvtkWidget->GetInteractor()->SetInteractorStyle(pointStyle);

    scribbleStyle->SetCurrentRenderer(vtkSmartPointer<vtkRenderer>::New());
    scribbleStyle->InitializeTracer(vtkSmartPointer<vtkImageActor>::New());

}

void MainWindow::on_connectContourButton_clicked() {
    qDebug() << "Total number of segments: " << brokenContourEndPoints.size();

    QPoint start, end;

    size_t dist_start = ui->bucketsizeSpinBox->value();
    size_t dist_end = ui->bucketsizeSpinBox->value();

    bool found_start = false;
    bool found_end = false;

    for (size_t i = 0; i < brokenContourEndPoints.size(); i++) {
        for (size_t j = 0; j < 2; j++) {
            QPoint p1;

            if (j == 0) {
                p1 = brokenContourEndPoints[i].first;
            } else {
                p1 = brokenContourEndPoints[i].second;
            }
            const size_t curr_dist_start = distance(points[0].toPoint(), p1);
            if (curr_dist_start <= dist_start) {
                start = p1;
                dist_start = curr_dist_start;
                found_start = true;
            }

            const size_t curr_dist_end = distance(points[1].toPoint(), p1);
            if (curr_dist_end <= dist_end) {
                end = p1;
                dist_end = curr_dist_end;
                found_end = true;
            }
        }
    }
    if (found_start && found_end) {
        qDebug() << "Found both start and end";
        qDebug() << "Start: " << start;
        qDebug() << "End: " << end;

        try {
            brokenContourPathPoints = getContourPoints(start, end, brokenContourEndPoints, brokenContourConnectivity);

            Eigen::RowVectorXd x = Eigen::RowVectorXd::Zero(brokenContourPathPoints.size());
            Eigen::RowVectorXd y = Eigen::RowVectorXd::Zero(brokenContourPathPoints.size());
            Eigen::RowVectorXd z = Eigen::RowVectorXd::Zero(brokenContourPathPoints.size());

            for (size_t i = 0; i < brokenContourPathPoints.size(); i++) {
                x[i] = brokenContourPathPoints[i].x();
                y[i] = brokenContourPathPoints[i].y();
            }

            Eigen::MatrixXd P;
            Eigen::RowVectorXd U;

            const size_t p = 2;

            std::tie(P, U) = GlobalCurveInterp(brokenContourPathPoints.size(), p, x, y, z);

            Eigen::VectorXd u = Eigen::VectorXd::Zero(10001);
            size_t index = 0;
            for (float t = 0; t <= 1; t += 0.0001) {
                u[index++] = t;
            }

            const Eigen::MatrixXd points = CurvePoint(p, P, U, u);

            for (size_t i = 0; i < 10001; i++) {
                InputImageType::IndexType pixelIndex;
                pixelIndex[0] = points(0, i);
                pixelIndex[1] = points(1, i);

                originalImage->SetPixel(pixelIndex, 0);

                updateImage(originalImage);
            }
        } catch (std::domain_error &e) {
            QMessageBox::critical(
                this,
                tr("Error"),
                e.what(),
                QMessageBox::Ok);
            brokenContourEndPoints.clear();
            brokenContourConnectivity.clear();
        }
    } else {
        qDebug() << "Could not find either start or end";
    }

    points.clear();
}
