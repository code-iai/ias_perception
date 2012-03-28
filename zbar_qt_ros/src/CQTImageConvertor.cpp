/*
 * CQTImageConvertor.cpp
 *
 *  Created on: 23.07.2010
 *      Author: grigorescu
 */

#include "CQTImageConvertor.h"

CQTImageConvertor::CQTImageConvertor()
{
	// TODO Auto-generated constructor stub
}

CQTImageConvertor::~CQTImageConvertor()
{
	// TODO Auto-generated destructor stub
}


void CQTImageConvertor::showQTImage(QImage* image, QLabel* label)
{
    if(!image->isNull())
    {
		QPixmap pixmap = QPixmap::fromImage(*image);
    	pixmap = pixmap.scaled(label->width(), label->height(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
    	label->setPixmap(pixmap);
    }
}

void CQTImageConvertor::showQTImage(const IplImage* image, QLabel* label)
{

	QImage *qimg = new QImage(image->width, image->height, QImage::Format_RGB32);

	IplImage2QImage(image, qimg);

	showQTImage(qimg, label);

	delete qimg;

}

void CQTImageConvertor::showQTImage(const IplImage* image, QPixmap &pixmap)
{

	QImage *qimg = new QImage(image->width, image->height, QImage::Format_RGB32);
	std::cerr <<"part 1\n";
	IplImage2QImage(image, qimg);
	std::cerr <<"part 2\n";

	if(!qimg->isNull())
	{
		std::cerr << "before picmap\n";
		pixmap = QPixmap::fromImage(*qimg);
		std::cerr << "*pixmap = QPixmap::fromImage(*qimg);\n";
		pixmap.scaled(image->width, image->height, Qt::KeepAspectRatio, Qt::SmoothTransformation);
		std::cerr << "pixmap->scaled(image->width, image->height, Qt::KeepAspectRatio, Qt::SmoothTransformation);\n";
	}

	std::cerr <<"part 3\n";
	//delete qimg;

}

void CQTImageConvertor::showQTImage(cv::Mat image, QLabel* label)
{
	QImage *qimg = new QImage(image.rows, image.cols, QImage::Format_RGB32);

	IplImage2QImage(image, qimg);

	showQTImage(qimg, label);

	delete qimg;
}

void CQTImageConvertor::IplImage2QImage(const IplImage *iplImg, QImage* qimg)
{
	int h = iplImg->height;
	int w = iplImg->width;
	int channels = iplImg->nChannels;

	char *data = iplImg->imageData;

	for (int y = 0; y < h; y++, data += iplImg->widthStep)
	{
		for (int x = 0; x < w; x++)
		{
			char r=0, g=0, b=0, a=0;

			if (channels == 1)
			{
				r = data[x * channels];
				g = data[x * channels];
				b = data[x * channels];
			}
			else if (channels == 3 || channels == 4)
			{
				r = data[x * channels + 2];
				g = data[x * channels + 1];
				b = data[x * channels];
			}

			if (channels == 4)
			{
				a = data[x * channels + 3];
				qimg->setPixel(x, y, qRgba(r, g, b, a));
			}
			else
			{
				qimg->setPixel(x, y, qRgb(r, g, b));
			}
		}
	}
}

void CQTImageConvertor::IplImage2QImage(cv::Mat iplImg, QImage* qimg)
{
	int h = iplImg.cols;
	int w = iplImg.rows;
	int channels = iplImg.channels();

	char *data = (char*)iplImg.data;

	for (int y = 0; y < h; y++, data += iplImg.step)
	{
		for (int x = 0; x < w; x++)
		{
			char r=0, g=0, b=0, a=0;

			if (channels == 1)
			{
				r = data[x * channels];
				g = data[x * channels];
				b = data[x * channels];
			}
			else if (channels == 3 || channels == 4)
			{
				r = data[x * channels + 2];
				g = data[x * channels + 1];
				b = data[x * channels];
			}

			if (channels == 4)
			{
				a = data[x * channels + 3];
				qimg->setPixel(x, y, qRgba(r, g, b, a));
			}
			else
			{
				qimg->setPixel(x, y, qRgb(r, g, b));
			}
		}
	}
}
