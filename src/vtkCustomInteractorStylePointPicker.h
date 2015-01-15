#ifndef __vtkCustomInteractorStylePointPicker_h
#define __vtkCustomInteractorStylePointPicker_h

#include "vtkInteractionStyleModule.h" // For export macro
// #include "vtkInteractorStyle.h"
#include "vtkInteractorStyleTrackballCamera.h"

class vtkUnsignedCharArray;

class VTKINTERACTIONSTYLE_EXPORT vtkCustomInteractorStylePointPicker : public vtkInteractorStyleTrackballCamera
{
public:
  static vtkCustomInteractorStylePointPicker *New();
  vtkTypeMacro(vtkCustomInteractorStylePointPicker, vtkInteractorStyleTrackballCamera);
  void PrintSelf(ostream& os, vtkIndent indent);

  virtual void OnLeftButtonDown();
  
protected:
  vtkCustomInteractorStylePointPicker();
  ~vtkCustomInteractorStylePointPicker();

private:
  vtkCustomInteractorStylePointPicker(const vtkCustomInteractorStylePointPicker&); // Not implemented
  void operator=(const vtkCustomInteractorStylePointPicker&); // Not implemented
};

#endif
