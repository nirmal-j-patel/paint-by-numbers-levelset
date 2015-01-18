#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "structs.h"

// Qt headers
#include <QMainWindow>
#include <QFutureWatcher>

// ITK headers
#include <itkImage.h>
#include <itkImageToVTKImageFilter.h>
#include <itkRGBPixel.h>

// VTK headers
#include <vtkSmartPointer.h>
#include <vtkImageActor.h>
#include <vtkRenderer.h>
#include <vtkInteractorStyleImage.h>
#include <vtkEventQtSlotConnect.h>
#include "vtkCustomInteractorStyleRubberBand2D.h"
#include "vtkCustomInteractorStylePointPicker.h"

#include "vtkInteractorStyleScribble.h"

namespace Ui {
class MainWindow;
}

class QProgressBar;
class QLabel;
class QProgressDialog;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    // Qt slots
    void on_arrowRadioBox_clicked();

    void on_regionRadioBox_clicked();

    void on_pencilRadioBox_clicked();

    void on_eraserRadioBox_clicked();

    void on_actionOpen_triggered();

    void on_actionStart_triggered();
    
    void on_detectArrowButton_clicked();
    
    /* ---------------------
     * VTK to Qt connections
     * ---------------------
     */
    void updateCoords(vtkObject*);
    
    void addRectangle(vtkObject * caller, unsigned long,
               void * clientData, void *callData,
               vtkCommand * command);
    
    void drawScribble(vtkObject * caller, unsigned long,
               void * clientData, void *callData,
               vtkCommand * command);

    void redrawRectangles(vtkObject * caller, unsigned long,
               void * clientData, void *callData,
               vtkCommand * command);
    
    void selectRectangularRegions(vtkObject * caller, unsigned long,
               void * clientData, void *callData,
               vtkCommand * command);
    
    void addPoint(vtkObject * caller, unsigned long,
               void * clientData, void *callData,
               vtkCommand * command);


    void on_applyLSButton_clicked();

    void on_colorPushButton_clicked();

    void on_symmetryRadioButton_clicked();

    void on_brokenContourRadioBox_clicked();

    void on_connectContourButton_clicked();

private:
    Ui::MainWindow *ui;
    
    /* ------------
     *  Qt section
     * ------------
     */
    void setupStatusbar();

    QProgressBar *progressbar;
    QLabel *coordLabel;


    /* -------------
     *  ITK section
     * -------------
     */

    void setupITK();
    void generateVesselness();
    void evolveContours();
    void findSegmentEndPoints();

    void showSelectedRectSeedInfo();
    void updateSelectedRectSeedInfo();

    //typedefs for readability
    //typedef itk::RGBPixel<float> InputPixelType;
    //typedef itk::RGBPixel<unsigned char> InputPixelType;
    typedef float InputPixelType;
    typedef itk::Image < InputPixelType, 2 >    InputImageType;

    InputImageType::Pointer originalImage;
    InputImageType::Pointer smoothenedImage;

    typedef bool BinaryPixelType;
    typedef itk::Image < BinaryPixelType, 2 >    BinaryImageType;
    
    InputImageType::Pointer originalGrayscaleImage;
    InputImageType::Pointer vesselnessImage;
    InputImageType::Pointer pathImage;


    //typedef itk::RGBPixel<float> OutputPixelType;
    typedef itk::RGBPixel<unsigned char> OutputPixelType;
    typedef itk::Image < OutputPixelType, 2 >   OutputImageType;
    
    OutputImageType::Pointer outputImage;
    OutputImageType::Pointer connectedContourImage;

    typedef itk::ImageToVTKImageFilter< InputImageType >   itk2vtkInputImageConnectorType;
    itk2vtkInputImageConnectorType::Pointer itk2vtkInputImageTypeConnector;
    
    typedef itk::ImageToVTKImageFilter< BinaryImageType >   itk2vtkBinaryImageConnectorType;
    itk2vtkBinaryImageConnectorType::Pointer itk2vtkBinaryImageTypeConnector;
    
    typedef itk::ImageToVTKImageFilter< OutputImageType >   itk2vtkOutputImageConnectorType;
    itk2vtkOutputImageConnectorType::Pointer itk2vtkOutputImageTypeConnector;
    
    QVector<rectSeed> rectSeeds;
    rectSeed *selectedRectSeed;

    QVector<QPointF> points;
    QPointF axisOfSymmetryPoint;
    QVector<QPoint> brokenContourPathPoints;
    QVector<QPair<QPoint, QPoint>> brokenContourEndPoints;
    QVector<QPair<QPoint, QPoint>> brokenContourConnectivity;
    
    QFutureWatcher<void> vesselnessWatcher;
    QProgressDialog *vesselnessProgressDialog;

    QFutureWatcher<void> gacWatcher;
    QProgressDialog *gacProgressDialog;

    QFutureWatcher<void> segmentPointsWatcher;
    QProgressDialog *segmentPointsProgressDialog;


    /* -------------
     *  VTK section
     * -------------
     */

    void setupVTK();

    void updateImage(InputImageType::Pointer image);
    
    void updateImage(BinaryImageType::Pointer image);
    
    void updateImage(OutputImageType::Pointer image);
    
    QPointF displayToWorld(double x, double y);
    
    QPointF worldToNormalizedDisplay(double x, double y);
    
    QPointF worldToDisplay(double x, double y);

    // image actor
    vtkSmartPointer<vtkImageActor> actor;

    // renderer
    vtkSmartPointer<vtkRenderer> renderer;

    // interaction styles
    vtkSmartPointer<vtkCustomInteractorStyleRubberBand2D> rectangleStyle;
    vtkSmartPointer<vtkCustomInteractorStylePointPicker> pointStyle;
    vtkSmartPointer<vtkInteractorStyleScribble> scribbleStyle;

    // VTK to Qt connections
    vtkSmartPointer<vtkEventQtSlotConnect> vtk2QtConnections;
};

#endif // MAINWINDOW_H
