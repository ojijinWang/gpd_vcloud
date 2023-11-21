// *** Standard
#include <stdio.h>
#include <iostream>
// *** OpenCV
#include <opencv2/opencv.hpp>

cv::Mat makeCanvas(const std::vector<cv::Mat>& vecMat, int windowHeight, int nRows);

cv::Mat makeCanvas(const std::vector<cv::Mat>& vecMat, int windowHeight, int nRows) 
{
	int N = vecMat.size();
	nRows  = nRows > N ? N : nRows; 
	int edgeThickness = 10;
	int imagesPerRow = ceil(double(N) / nRows);
	int resizeHeight = floor(2.0 * ((floor(double(windowHeight - edgeThickness) / nRows)) / 2.0)) - edgeThickness;
	int maxRowLength = 0;

	std::vector<int> resizeWidth;
	for (int i = 0; i < N;) 
	{
		int thisRowLen = 0;
		for (int k = 0; k < imagesPerRow; k++)
		{
			double aspectRatio = double(vecMat[i].cols) / vecMat[i].rows;
			int temp = int( ceil(resizeHeight * aspectRatio));
			resizeWidth.push_back(temp);
			thisRowLen += temp;
			if (++i == N) break;
		}
		if ((thisRowLen + edgeThickness * (imagesPerRow + 1)) > maxRowLength)
		{ maxRowLength = thisRowLen + edgeThickness * (imagesPerRow + 1); }
	}
	int windowWidth = maxRowLength;
	cv::Mat canvasImage(windowHeight, windowWidth, CV_8UC3, cv::Scalar(255, 255, 255));

	for (int k = 0, i = 0; i < nRows; i++) 
	{
		int y = i * resizeHeight + (i + 1) * edgeThickness;
		int x_end = edgeThickness;
		for (int j = 0; j < imagesPerRow && k < N; k++, j++) 
		{
 			int x = x_end;
			cv::Rect roi(x, y, resizeWidth[k], resizeHeight);
			cv::Size s = canvasImage(roi).size();
			// change the number of channels to three
			cv::Mat target_ROI(s, CV_8UC3);
			if (vecMat[k].channels() != canvasImage.channels()) 
			{
				if (vecMat[k].channels() == 1) 
				{ cv::cvtColor(vecMat[k], target_ROI, CV_GRAY2BGR); }
			} 
			else
			{ vecMat[k].copyTo(target_ROI); }
			cv::resize(target_ROI, target_ROI, s);
			if (target_ROI.type() != canvasImage.type()) 
			{ target_ROI.convertTo(target_ROI, canvasImage.type()); }
			target_ROI.copyTo(canvasImage(roi));
			x_end += resizeWidth[k] + edgeThickness;
		}
	}
	return canvasImage;
}
