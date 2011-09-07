/*
ROBOOP -- A robotics object oriented package in C++
Copyright (C) 1996-2004  Richard Gourdeau

This library is free software; you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as
published by the Free Software Foundation; either version 2.1 of the
License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA


Report problems and direct all questions to:

Richard Gourdeau
Professeur Agrege
Departement de genie electrique
Ecole Polytechnique de Montreal
C.P. 6079, Succ. Centre-Ville
Montreal, Quebec, H3C 3A7

email: richard.gourdeau@polymtl.ca

-------------------------------------------------------------------------------
Revision_history:

2004/07/01: Etienne Lachance
   -Added doxygen documentation.

2004/07/01: Ethan Tira-Thompson
    -Added support for newmat's use_namespace #define, using ROBOOP namespace

2004/09/18: Etienne Lachance
   -Added deg2rad rad2deg

2005/06/10: Carmine Lia
   -Added pinv

2008/08/01: Ulrich Klank, Lorenz Mosenlechner
   - Removed large code parts, created single include

Cite author (ref:  http://www.robertnz.net/nm10.htm#problem):
   1.1 Conditions of use
        next - skip - up - start
       I place no restrictions on the use of newmat except that I take no liability for any problems that may arise from its use, distribution or other dealings with it.

      You can use it in your commercial projects.

      You can make and distribute modified or merged versions. You can include parts of it in your own software.

      If you distribute modified or merged versions, please make it clear which parts are mine and which parts are modified.

      For a substantially modified version, simply note that it is, in part, derived from my software. A comment in the code will be sufficient.

      The software is provided "as is", without warranty of any kind.

      Please understand that there may still be bugs and errors. Use at your own risk. I (Robert Davies) take no responsibility for any errors or omissions in this
      package or for any misfortune that may befall you or others as a result of your use, distribution or other dealings with it.
End Cite.

All contents in the Folder 'NewMat' and in the File NewMatExhaustive.cpp are created by R B Davies.
Reorganization of the content was performed by  Ulrich Klank and Lorenz Mosenlechner

-------------------------------------------------------------------------------
*/

#ifndef __cplusplus
#error Must use C++ for the type Robot
#endif

#ifndef NEWMAT_LIB
#define NEWMAT_LIB

#include "NewMat/utils.h"
#include "NewMat/include.h"
#include "NewMat/myexcept.h"
#include "NewMat/general.h"
#include "NewMat/BaseMatrix.h"
#include "NewMat/GeneralMatrix.h"
#include "NewMat/Matrix.h"
#include "NewMat/SquareMatrix.h"
#include "NewMat/nricMatrix.h"
#include "NewMat/SymmetricMatrix.h"
#include "NewMat/TriangularMatrix.h"
#include "NewMat/DiagonalMatrix.h"
#include "NewMat/Vector.h"
#include "NewMat/CroutMatrix.h"
#include "NewMat/BandMatrix.h"
#include "NewMat/IdentityMatrix.h"
#include "NewMat/GenericMatrix.h"
#include "NewMat/Temporary.h"
#include "NewMat/SubMatrix.h"
#include "NewMat/LinearEquationSolver.h"
#include "NewMat/Array.h"
#include "NewMat/MatrixExceptions.h"
#include "NewMat/MatrixInput.h"
#include "NewMat/Functions.h"
#include "NewMat/RectMatrix.h"

#endif

