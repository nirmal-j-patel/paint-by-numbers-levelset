/*
Copyright (C) 2010 David Doria, daviddoria@gmail.com

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/*
 * This class is responsible for the user interaction with the input image.
 * A vtkImageTracerWidget does most of the work, but this class appends and maintains
 * the selections.
 * Connect to this interactor's event with something like:
 *   this->InteractorStyle->AddObserver(this->InteractorStyle->ScribbleEvent,
                                         this, &LidarSegmentationWidget::ScribbleEventHandler);
 * Note the ScribbleEventHandler function must be a VTK-style slot, e.g.:
 * void LidarSegmentationWidget::ScribbleEventHandler(vtkObject* caller, long unsigned int eventId, void* callData)
*/

#ifndef vtkInteractorStyleScribble_H
#define vtkInteractorStyleScribble_H

#include <vtkImageTracerWidget.h>
#include <vtkInteractorStyleImage.h> // superclass
#include <vtkSmartPointer.h>

class vtkImageSlice;
class vtkImageData;
class vtkPolyData;

class vtkInteractorStyleScribble : public vtkInteractorStyleImage
{
public:
  static vtkInteractorStyleScribble* New();
  vtkTypeMacro(vtkInteractorStyleScribble, vtkInteractorStyleImage);

  vtkInteractorStyleScribble();

  void OnLeftButtonDown();
  void OnLeftButtonUp();

  void SetColorToGreen();
  void SetColorToRed();

  vtkPoints* GetSelection();

  /** Empty both the foreground and background selection */
  void ClearSelections();

  /** Empty the foreground selection */
  void ClearForegroundSelections();

  /** Empty the background selection */
  void ClearBackgroundSelections();

  /** Connect the tracer to the interactor, etc. */
  void InitializeTracer(vtkImageSlice* imageSlice);

  unsigned int ScribbleEvent;

private:
  void Refresh();

  /** Update the selection when the EndInteraction event is fired. */
  void CatchWidgetEvent(vtkObject* caller, long unsigned int eventId, void* callData);

  /** The state (foreground or background) of the selection. */
  int SelectionType;

  /** The widget which does most of the work. */
  vtkSmartPointer<vtkImageTracerWidget> Tracer;

  /** Keep track of the pixels the user selected. */
  vtkSmartPointer<vtkPoints> Selection;

};

#endif
