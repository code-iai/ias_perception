/*
 * CQTImageConvertor.h
 *
 *  Created on: 23.07.2010
 *      Author: grigorescu
 */

#ifndef CQTIMAGECONVERTOR_H_
#define CQTIMAGECONVERTOR_H_

//#include "QT_TYPES.h"
//#include "../common_utils/ROVIS_TYPES.h"
#include <QLabel>
#include <QImage>
#include "cv_bridge/CvBridge.h"

class CQTImageConvertor
{
public:
	CQTImageConvertor();
	virtual ~CQTImageConvertor();

public:
	static void showQTImage(QImage* image, QLabel* label);

	static void showQTImage(const IplImage* image, QLabel* label);

	static void showQTImage(cv::Mat image, QLabel* label);

	static void IplImage2QImage(const IplImage *iplImg, QImage* qimg);

	static void IplImage2QImage(cv::Mat iplImg, QImage* qimg);

	static void showQTImage(const IplImage* image, QPixmap &pixmap);
};

#endif /* CQTIMAGECONVERTOR_H_ */
