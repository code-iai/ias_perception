
#ifndef __WRAPPED_MATRIX_H__
#define __WRAPPED_MATRIX_H__

#include <string.h>

#include <iostream>
#include <sstream>

#include <NewMat/Matrix.h>
#include <NewMat/Temporary.h>

namespace jlo
{

    class Matrix
    {
        public:
            Matrix() : w(0), h(0), data(0), printed(0) {}
            Matrix(int _w, int _h)
                : w(_w), h(_h), data( (_w * _h > 0) ? new double[_w * _h] : 0 ), printed(0)
            { memset( data, 0, w * h * sizeof( *data ) ); }
            Matrix( const jlo::Matrix &src )
                : w(src.w), h(src.h), data( (w * h > 0) ? new double[ src.w * src.h ] : 0 ), printed(0)
            { memcpy( data, src.data, w*h ); }
            Matrix( const ::BaseMatrix &src )
                : h(0), w(0), data(0), printed(0)
            { init( src ); }

            ~Matrix() { delete[] data; delete[] printed; }
            int width() const { return w; }
            int height() const { return h; }
            double get( int x, int y ) const { return data[ y*w + x ]; }
            void set( int x, int y, double newval ) { data[ y*w + x ] = newval; }

            operator ::Matrix() const
            {
                ::Matrix result( toReturnMatrix() );
                return result;
            }
            operator ReturnMatrix() const
            {
                return toReturnMatrix();
            }
            Matrix &operator=( const Matrix &src )
            {
                w = src.w;
                h = src.h;
                delete[] data;
                data = w * h ? new double[w * h] : 0;
                memcpy( data, src.data, w * h * sizeof( *data ) );
                return *this;
            }
            const char *print()
            {
                std::stringstream str;

                str.str("");
                for( int y=0; y<height(); y++ )
                {
                    str << "| ";
                    for( int x=0; x<width(); x++ )
                        str << get(x, y) << " ";
                    str << "|" << std::endl;
                }
                delete[] printed;
                printed = new char[ str.str().size()+1 ];
                strcpy( printed, str.str().c_str() );
                return printed;
            }

        private:
            int w;
            int h;
            double *data;
            char *printed;

            void init( const ::Matrix &src )
            {
                if( w!=src.ncols() || h!=src.nrows() )
                {
                    delete[] data;
                    w = src.ncols();
                    h = src.nrows();
                    data = (w * h > 0) ? new double[ w * h ] : 0;
                }
                for( int y=0; y<h; y++ )
                {
                    for( int x=0; x<w; x++ )
                        set( x, y, src.element( y, x ) );
                }
            }
            ::ReturnMatrix toReturnMatrix() const
            {
                ::Matrix result( w, h );

                for( int y=0; y<h; y++ )
                {
                    for( int x=0; x<w; x++ )
                        result.element( y, x ) = get( x, y );
                }
                return ::ReturnMatrix( result );
            }
    };
}

#endif

