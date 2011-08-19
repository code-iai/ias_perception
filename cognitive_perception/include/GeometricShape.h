#ifndef GEOMETRIC_SHAPES_H
#define GEOMETRIC_SHAPES_H
#include <vector>


namespace cop
{
typedef struct Point_
{
   typedef double _x_type;
   double x;
                                    
   typedef double _y_type;
   double y;
                                       
   typedef double _z_type;
   double z;
} PointShape;                                            

typedef struct Shape_
{

  unsigned char  type;

  std::vector<double>  dimensions;

  std::vector<int >  triangles;

  std::vector< PointShape >  vertices;

  enum { SPHERE = 0 };
  enum { BOX = 1 };
  enum { CYLINDER = 2 };
  enum { MESH = 3 };

} GeometricShape;
}
#endif // GEOMETRIC_SHAPES__H

