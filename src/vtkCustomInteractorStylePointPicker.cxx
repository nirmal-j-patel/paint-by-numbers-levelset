/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkInteractorStyleRubberBand2D.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
/*-------------------------------------------------------------------------
  Copyright 2008 Sandia Corporation.
  Under the terms of Contract DE-AC04-94AL85000 with Sandia Corporation,
  the U.S. Government retains certain rights in this software.
-------------------------------------------------------------------------*/

#include "vtkCustomInteractorStylePointPicker.h"

#include "vtkActor.h"
#include "vtkCamera.h"
#include "vtkCommand.h"
#include "vtkObjectFactory.h"
#include "vtkRenderer.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkUnsignedCharArray.h"

vtkStandardNewMacro(vtkCustomInteractorStylePointPicker);

//--------------------------------------------------------------------------
vtkCustomInteractorStylePointPicker::vtkCustomInteractorStylePointPicker() {
}

//--------------------------------------------------------------------------
vtkCustomInteractorStylePointPicker::~vtkCustomInteractorStylePointPicker() {
}

//--------------------------------------------------------------------------
void vtkCustomInteractorStylePointPicker::OnLeftButtonDown() {
    this->InvokeEvent(vtkCommand::SelectionChangedEvent, reinterpret_cast<void*>(this->Interactor->GetEventPosition()));
}

//--------------------------------------------------------------------------
void vtkCustomInteractorStylePointPicker::PrintSelf(ostream& os, vtkIndent indent) {
}
