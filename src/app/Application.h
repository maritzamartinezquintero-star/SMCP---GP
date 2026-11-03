/*
Copyright (c) 2014, Daniel Moreno and Gabriel Taubin
Copyright (c) 2024, José Luis Aguilera Luzania, Agustín Brau Ávila & Octavio Icasio Hernández
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
	* Redistributions of source code must retain the above copyright
	  notice, this list of conditions and the following disclaimer.
	* Redistributions in binary form must reproduce the above copyright
	  notice, this list of conditions and the following disclaimer in the
	  documentation and/or other materials provided with the distribution.
	* Neither the name of the Brown University nor the
	  names of its contributors may be used to endorse or promote products
	  derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL DANIEL MORENO AND GABRIEL TAUBIN BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#pragma once
#ifndef WINVER
#define WINVER 0x0500
#define _WIN32_WINNT 0x0500
#endif

#include <QApplication>
#include <QSettings>
#include <QList>
#include <QFileSystemModel>
#include <QMap>

#include <opencv2/core/core.hpp>

#include "ui/models/TreeModel.h"
#include "app/MainWindow.h"
#include "ui/ProcessingDialog.h"
#include "core/CalibrationData.h"
#include "core/Scan3d.h"
#include "core/GPCompensation.h"
#include "common/Settings.h"

namespace smcp
{

#if defined(_MSC_VER) && !defined(isnan)
#define isnan _isnan
#endif

enum Role {
	ImageFilenameRole = Qt::UserRole, GrayImageRole, ColorImageRole,
	ProjectorWidthRole, ProjectorHeightRole
};

#define DUMP_ROWS 1
#define DUMP_COLS 2

class Application : public QApplication
{
	Q_OBJECT
public:
	Application(int& argc, char** argv);
	~Application();

	QSettings& GetSettings();

	//data dir
	void SetRootDir(const QString& dirname);
	QString GetRootDir(void) const;
	bool ChangeRootDir(QWidget* parentWidget = NULL);

	void Clear(void);

	const cv::Mat GetImage(unsigned level, unsigned n, Role role = GrayImageRole) const;
	int GetCameraWidth(unsigned level = 0) const;
	int GetCameraHeight(unsigned level = 0) const;
	int GetProjectorWidth(unsigned level = 0) const;
	int GetProjectorHeight(unsigned level = 0) const;

	bool ExtractChessboardCorners(void);
	static void GetChessboardWorldCoords(std::vector<cv::Point3f>& worldCorners, cv::Size cornerCount, cv::Size cornerSize);

	bool ExtractChessboardCornersV2(void);
	static void GetChessboardWorldCoordsV2(std::vector<cv::Point3f>& worldCorners, cv::Size cornerCount, cv::Size2f cornerSize);
	void DecodeAll(void);
	void Decode(int level, QWidget* parentWidget = NULL);
	void Calibrate(void);

	bool DecodeGraySet(unsigned level, cv::Mat& patternImage, cv::Mat& minMaxImage, QWidget* parentWidget = NULL) const;
	bool DumpDecoded(const char* filename, int type, cv::Mat2f const& patternImage, cv::Mat2b const& minMaxImage, cv::Mat3b const& colorImage) const;
	bool LoadDump(const char* filename, int type, cv::Mat2f& patternImage, cv::Mat2b& minMaxImage, cv::Mat3b& colorImage) const;

	void LoadConfig(void);
	void ApplyTheme(void);

	//Detection/Decoding/Calibration processing
	inline void ProcessingSetCurrentMessage(const QString& text) const { processingDialog.SetCurrentMessage(text); processEvents(); }
	inline void ProcessingReset(void) { processingDialog.Reset(); processEvents(); }
	inline void ProcessingSetProgressTotal(unsigned value) { processingDialog.SetProgressTotal(value); processEvents(); }
	inline void ProcessingSetProgressValue(unsigned value) { processingDialog.SetProgressValue(value); processEvents(); }
	inline void ProcessingMessage(const QString& text) const { processingDialog.Message(text); processEvents(); }
	inline bool ProcessingCanceled(void) const { return processingDialog.Canceled(); }

	//calibration
	bool LoadCalibration(QWidget* parentWidget = NULL);
	bool SaveCalibration(QWidget* parentWidget = NULL);

	//reconstruction
	void ReconstructModel(int level, Scan3d::Pointcloud& pointcloud, QWidget* parentWidget = NULL);
	void ReconstructModelDump(cv::Mat2f const& patternImage, cv::Mat2b const& minMaxImage, cv::Mat3b const& colorImage, Scan3d::Pointcloud& pointcloud, QWidget* parentWidget = NULL);
	void ComputeNormals(Scan3d::Pointcloud& pointcloud);

	//GP compensation
	bool LoadGpCompensation(QWidget* parentWidget = NULL);
	bool ApplyGpCompensation(Scan3d::Pointcloud& pointcloud, QWidget* parentWidget = NULL);
	void CompensateXyzFile(QWidget* parentWidget = NULL);

	void MakePatternImages(int level, cv::Mat& colImage, cv::Mat& rowImage);
	cv::Mat GetProjectorView(int level, bool forceUpdate = false);

	//model
	void SelectNone(void);
	void SelectAll(void);

public slots:
	void deinit(void);

Q_SIGNALS:
	void root_dir_changed(const QString& dirname);

public:
	QSettings  config;
	TreeModel  model;

	CalibrationData calib;

	cv::Size2i cornerCount;
	cv::Size2f cornerSize;
	std::vector<std::vector<cv::Point3f> > cornersWorld;
	std::vector<std::vector<cv::Point2f> > cornersCamera;
	std::vector<std::vector<cv::Point2f> > cornersProjector;
	std::vector<cv::Mat> patternList;
	std::vector<cv::Mat> minMaxList;
	std::vector<cv::Mat> projectorViewList;
	Scan3d::Pointcloud pointcloud;
	GPCompensation gpCompensation;
	bool pointcloudGpCompensated = false;

	MainWindow mainWin;
	mutable ProcessingDialog processingDialog;
};

#define APP dynamic_cast<Application *>(Application::instance())

} // namespace smcp
