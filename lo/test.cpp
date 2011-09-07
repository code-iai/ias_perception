// Copyright 2007 Alexis Maldonado
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include "lo/lo.h"
#include <iostream>
#include "lo/UT_Timer.h"

using namespace std;
using jlo::LocatedObject;

class ObjectLoader : public jlo::LazyLocatedObjectLoader
{
  LocatedObject* GetParent(const LocatedObject& child){return NULL;};
} test;

int main() {
  
  cout << "LocatedObject: Testing simple transformations\n";
  
  
  LocatedObject robot(&test, NULL,0,0,0,0,0,0);
  
  LocatedObject rightShoulder(&test, 123, robot.m_uniqueID, 0.085,-0.165,1.053,- M_PI/2, 0,M_PI*30.0/180.0);
  
  LocatedObject leftShoulder(&test, 124 ,robot.m_uniqueID,0.085,0.165,1.053,  -M_PI/2, 0,-M_PI*30.0/180.0);
  
}

