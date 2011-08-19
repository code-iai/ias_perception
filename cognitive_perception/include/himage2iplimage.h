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

 
#ifndef HIMAGE2IPLIMAGE_H
#define HIMAGE2IPLIMAGE_H
/**
*	
*
**/
#include <Halcon.h>



/*/ here are following private macros for the next function: HImage2IplImage
//Copy all channels in the composed pixels that are expected by OpenCv**/
template<class type> IplImage* CPYIMG2CV(Hlong** himg, CvSize size, INT depth, INT channels, INT width, INT height)					
{																				
	IplImage* iplimg;
    type* pointer[3];															
	int pixelItLines = 1;														
	int pixelItColumns = 1;														
	int channelIt = 0;															
	iplimg = cvCreateImage(size, depth, channels);				
	if(channels == 1 && (sizeof(type) * width * height) == iplimg->imageSize)	
	{																			
		memcpy(iplimg->imageData, (void*)himg[0], sizeof(type) * width * height);
	}																			
	else																		
	{																			
		for (channelIt = 0; channelIt < channels; channelIt++)					
			pointer[channelIt] = (type*)himg[channelIt];					
		for (pixelItLines = 0; pixelItLines < height ;pixelItLines++ )			
        {																		
			for (pixelItColumns = 0;pixelItColumns < width; pixelItColumns++)	
			{																	
				CvScalar csv;													
				for (channelIt = 0; channelIt < channels; channelIt++)			
				{																
					csv.val[(channels-1) - channelIt] = pointer[channelIt][pixelItLines * width +  pixelItColumns];
                }															
				cvSet2D(iplimg,pixelItLines,pixelItColumns,csv);				
			}																	
		}																		
	}
    return iplimg;
}																			

/*/Copy all channels in the composed pixels that are expected by OpenCv, special macro for the H(U)Int2Pixel Struct*/
#define CPYIMG_STRUCT2CV(depth, channels, width, height, type)			\
{																				\
	int pixelItLines = 1;														\
	int pixelItColumns = 1;														\
	int channelIt = 0;															\
	iplimg = cvCreateImage(size, depth, channels);				\
	for (pixelItLines = 0; pixelItLines < height ;pixelItLines++ )				\
	{																			\
		for (pixelItColumns = 0;pixelItColumns < width; pixelItColumns++)		\
		{																		\
			CvScalar csv;														\
			for(channelIt = 0; channelIt < channels; channelIt++)				\
				csv.val[(channels - 1) - channelIt] = himg[channelIt][pixelItLines * width +  pixelItColumns]; \
			cvSet2D(iplimg,pixelItLines,pixelItColumns,csv);					\
		}																		\
	}																			\
}

/*/Copy all channels in the composed pixels that are expected by OpenCv, special macro for the HComplexPixel Struct*/
#define CPYIMG_COMPLEX2CV(depth, channels, width, height, type)			\
{																				\
	int pixelItLines = 1;														\
	int pixelItColumns = 1;														\
	int channelIt = 0;															\
	iplimg = cvCreateImage(size, depth, channels);				\
	for (pixelItLines = 0; pixelItLines < height ;pixelItLines++ )				\
	{																			\
		for (pixelItColumns = 0;pixelItColumns < width; pixelItColumns++)		\
		{																		\
			CvScalar csv;														\
			for(channelIt = 0; channelIt < channels / 2; channelIt++)			\
			{																	\
				csv.val[((channels/2- 1) - channelIt) * 2    ] = himg[channelIt][pixelItLines * width +  pixelItColumns]; \
				csv.val[((channels/2- 1) - channelIt) * 2 + 1] = himg[channelIt][pixelItLines * width +  pixelItColumns]; \
			}																	\
			cvSet2D(iplimg,pixelItLines,pixelItColumns,csv);					\
		}																		\
	}																			\
}


/**
*	@author		Uli Klank
*	@date		08/03/2007
*	@brief		converts a HALCON image structure into an openCV Image
*	@return	a new IplImage that must be deleted if the function is succesful else NULL	
*	@remarks	not all images types are compatible, if the return value is NULL, an error occured.
*	@param	himg		The source image in an Halcon image format as an array of channels
*	@param	channels	Number of channels int the array given by himg, this information can not be extracted without proc_handle, so it has to be extracted before
**/
IplImage *HImage2IplImage(Hlong** himg, INT channels, INT width, INT height, INT type)
{
	IplImage* iplimg;
	CvSize size;
	size.width = width;
	size.height = height;
	if(channels > 3)
		channels = 3;
	
	switch(type)
	{
	case BYTE_IMAGE:
		/*/ - unsigend char*/
		iplimg = CPYIMG2CV<HBYTE>(himg,size, IPL_DEPTH_8U, channels, width, height);
		break;
	case INT4_IMAGE:
		/*/ - signed 32-bit integers*/
		iplimg = CPYIMG2CV<INT4>(himg,size, IPL_DEPTH_32S, channels, width, height);
		break;
	case FLOAT_IMAGE:
		/*/ - single precision floating-point numbers*/
		iplimg = CPYIMG2CV<float>(himg,size, IPL_DEPTH_32F, channels, width, height);
		/*/ TODO Check if there are used double in halcon instead of float?*/
		/*/depth = IPL_DEPTH_64F;// - double precision floating-point numbers*/
		break;
	case DIR_IMAGE: /*/TOCHECK: Byte? 0..180*/
		iplimg = CPYIMG2CV<HBYTE>(himg,size, IPL_DEPTH_8U, channels, width, height);
		break;
	case CYCLIC_IMAGE:/*/TOCHECK: nothing compareable in opencv*/
		/*/TOCHECK: 0..255*/
		iplimg = CPYIMG2CV<HBYTE>(himg,size, IPL_DEPTH_8U, channels, width, height);
		break;
	case INT1_IMAGE:
		/*/ - signed 8-bit integers*/
		iplimg = CPYIMG2CV<INT1>(himg,size, IPL_DEPTH_8S, channels, width, height);
		break;
	case COMPLEX_IMAGE: /*/TODO: not supported in open CV*/
		CPYIMG_COMPLEX2CV(IPL_DEPTH_32F, channels * 2, width, height, float*)
		break;
	case INT2_IMAGE:
		/*/ - signed 16-bit integers*/
		CPYIMG_STRUCT2CV(IPL_DEPTH_16S, channels, width, height, HInt2Pixel)
		break;
	case UINT2_IMAGE:
		/*/ - unsigned 16-bit integers*/
		CPYIMG_STRUCT2CV(IPL_DEPTH_16U, channels, width, height, HUInt2Pixel)
		break;
	}
	return iplimg;
}


#endif /*/HIMAGE2IPLIMAGE_H*/
