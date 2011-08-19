/*
 * Copyright (C) 2009 by Ulrich Friedrich Klank <klank@in.tum.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */



#ifndef ELEMTYPES_H
#define ELEMTYPES_H

typedef long ObjectID_t;
typedef long PerceptionPrimitiveID_t;
typedef unsigned long  LocatedObjectID_t;
typedef unsigned long  AlgorithmID_t;

enum ElemType_t
{
  ELEM,
  DESCRIPTOR_SHAPE,
  DESCRIPTOR_COLOR,
  DESCRIPTOR_FEATURE,
  DESCRIPTOR_TEXTURE,
  DESCRIPTOR_DEFORMSHAPE,
  DESCRIPTOR_CALTAB,
  DESCRIPTOR_BLOB,
  DESCRIPTOR_PLANE,
  DESCRIPTOR_COLORCLASS,
  DESCRIPTOR_SEGMPROTO,
  DESCRIPTOR_DETECTEDFACE,
  DESCRIPTOR_TRANSPARENTOBJECTCAND,
  DESCRIPTOR_TRANSPARENTOBJECT,
  DESCRIPTOR_CLOUDALGODATA,
  DESCRIPTOR_TABLEOBJ,
  DESCRIPTOR_SHAPETYPE,
  DESCRIPTOR_NAMEDCLASS,
  DESCRIPTOR_BARCODE,
  DESCRIPTOR_CIRCLE,
  DESCRIPTOR_SURFACE,
  DESCRIPTOR_MARKER,
  DESCRIPTOR_LINE3D,
  SIGNATURE,
  CLASS
};

enum SensorType_t
{
  SENSOR,
  SENSORTYPE_CAMERA,
  SENSORTYPE_SIMCAM,
  SENSORTYPE_SWISSRANGER
};

#endif /*ELEMTYPES_H*/
