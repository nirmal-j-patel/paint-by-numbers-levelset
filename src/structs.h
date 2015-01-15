#ifndef __STRUCTS_H__
#define __STRUCTS_H__

// Qt headers
#include <QColor>
#include <QLabel>

// VTK headers
#include <vtkBorderWidget.h>
#include <vtkSmartPointer.h>

/**
 * @brief struct containing information about rectangular initial contour
 */
struct rectSeed {
    unsigned long id;
    QRectF area;
    vtkSmartPointer<vtkBorderWidget> borderWidget;

    float propagationScaling;
    float maxRMSError;
    unsigned int maxIterations;

    QColor color;
    QString label;

    rectSeed(unsigned int ID) {
        id = ID;
        propagationScaling = 2;

        maxRMSError = 0.02;
        maxIterations = 10000;

        color = Qt::red;
        label = "";
    }

    rectSeed() {
        rectSeed(0);
    }
};

#endif
