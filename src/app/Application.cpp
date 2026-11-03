/*
Copyright (c) 2014, Daniel Moreno and Gabriel Taubin
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

#include "app/Application.h"

#include <QDir>
#include <QFile>
#include <QFont>
#include <QProgressDialog>
#include <QMessageBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QStyleFactory>

#include <cmath>
#include <iostream>
#include <ctime>
#include <cstdlib>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "core/StructuredLight.h"
#include "core/BusyCursorGuard.h"

namespace smcp
{

Application::Application(int& argc, char** argv) :
	QApplication(argc, argv),
	// User scope, native format (on Windows the registry: HKCU\Software\CENAM\SMCP).
	// IniFormat is not used: QSettings writes the INI through QSaveFile and its rename fails
	// on profiles with an encrypted AppData (EFS), leaving the configuration unsaved.
	config(QSettings::NativeFormat, QSettings::UserScope, Settings::App::Organization, Settings::App::App_Name, this),
	model(this),
	calib(),
	cornerCount(11, 7),
	cornerSize(21.f, 21.f),
	cornersWorld(),
	cornersCamera(),
	cornersProjector(),
	patternList(),
	minMaxList(),
	projectorViewList(),
	pointcloud(),
	// LoadConfig() and ApplyTheme() must run before the main window is constructed.
	mainWin((QWidget*)(LoadConfig(), ApplyTheme(), NULL)),
	processingDialog(&mainWin, Qt::Window | Qt::CustomizeWindowHint | Qt::WindowTitleHint)
{
	connect(this, SIGNAL(aboutToQuit()), this, SLOT(deinit()));

	//setup the main window state
	mainWin.show();
	mainWin.restoreGeometry(config.value(Settings::MainWindow::Geometry).toByteArray());
	QVariant windowState = config.value(Settings::MainWindow::State);
	if (windowState.isValid())
	{
		mainWin.setWindowState(static_cast<Qt::WindowStates>(windowState.toUInt()));
	}

	//set model
	SetRootDir(config.value(Settings::App::Root_Directory, QDir::currentPath()).toString());
	QModelIndex index = model.index(0, 0);
	mainWin._on_image_tree_currentChanged(index, index);
}

Application::~Application()
{}

QSettings& Application::GetSettings()
{
	return config;
}

void Application::deinit(void)
{
	config.setValue(Settings::MainWindow::Geometry, mainWin.saveGeometry());
	config.setValue(Settings::MainWindow::State, static_cast<unsigned>(mainWin.windowState()));
}

void Application::Clear(void)
{
	//calib.Clear();
	cornersWorld.clear();
	cornersCamera.clear();
	cornersProjector.clear();
	patternList.clear();
	minMaxList.clear();
	projectorViewList.clear();
	pointcloud.Clear();
}

// Single place where the look of the application is defined: Fusion style, base font
// and the resources/theme/smcp.qss sheet (see docs/STYLE.md). The .ui files carry no
// styleSheet or font properties.
void Application::ApplyTheme(void)
{
	setStyle(QStyleFactory::create("Fusion"));
	setFont(QFont("Segoe UI", 9));

	QFile qss(":/theme/smcp.qss");
	if (qss.open(QIODevice::ReadOnly | QIODevice::Text))
	{
		setStyleSheet(QString::fromUtf8(qss.readAll()));
	}
	else
	{
		std::cerr << "[theme] could not load :/theme/smcp.qss" << std::endl;
	}
}

void Application::LoadConfig(void)
{
	//decode
	if (!config.value(Settings::Decode::Threshold).isValid())
	{
		config.setValue(Settings::Decode::Threshold, Settings::Decode::Threshold_Default_Value);
	}
	if (!config.value(Settings::Decode::B).isValid())
	{
		config.setValue(Settings::Decode::B, Settings::Decode::B_Default_Value);
	}
	if (!config.value(Settings::Decode::M).isValid())
	{
		config.setValue(Settings::Decode::M, Settings::Decode::M_Default_Value);
	}

	//checkerboard size
	if (!config.value(Settings::Chessboard::Columns).isValid())
	{
		config.setValue(Settings::Chessboard::Columns, Settings::Chessboard::Columns_Default_Value);
	}
	if (!config.value(Settings::Chessboard::Rows).isValid())
	{
		config.setValue(Settings::Chessboard::Rows, Settings::Chessboard::Rows_Default_Value);
	}
	if (!config.value(Settings::Chessboard::Width).isValid())
	{
		config.setValue(Settings::Chessboard::Width, Settings::Chessboard::Width_Default_Value);
	}
	if (!config.value(Settings::Chessboard::Height).isValid())
	{
		config.setValue(Settings::Chessboard::Height, Settings::Chessboard::Height_Default_Value);
	}

	//reconstruction
	if (!config.value(Settings::Reconstruction::Max_Dist).isValid())
	{
		config.setValue(Settings::Reconstruction::Max_Dist, Settings::Reconstruction::Max_Dist_Default_Value);
	}
	if (!config.value(Settings::Reconstruction::Save_Normals).isValid())
	{
		config.setValue(Settings::Reconstruction::Save_Normals, Settings::Reconstruction::Save_Normals_Default_Value);
	}
	if (!config.value(Settings::Reconstruction::Save_Colors).isValid())
	{
		config.setValue(Settings::Reconstruction::Save_Colors, Settings::Reconstruction::Save_Colors_Default_Value);
	}
	if (!config.value(Settings::Reconstruction::Save_Binary).isValid())
	{
		config.setValue(Settings::Reconstruction::Save_Binary, Settings::Reconstruction::Save_Binary_Default_Value);
	}
}

void Application::SetRootDir(const QString& dirname)
{
	QDir rootDir(dirname);

	//reset internal data
	model.Clear();
	Clear();

	QStringList dirlist = rootDir.entryList(QDir::Dirs | QDir::NoDotAndDotDot, QDir::Name);
	foreach(const QString & item, dirlist)
	{
		QDir dir(rootDir.filePath(item));

		QStringList filters;
		filters << "*.jpg" << "*.bmp" << "*.png";

		QStringList filelist = dir.entryList(filters, QDir::Files, QDir::Name);
		QString path = dir.path();

		//setup the model
		int filecount = filelist.count();

		if (filecount < 1)
		{   //no images, skip
			continue;
		}

		unsigned row = model.rowCount();
		if (!model.InsertRow(row))
		{
			std::cout << "Failed model insert " << item.toStdString() << "(" << row << ")" << std::endl;
			continue;
		}

		//add the childrens
		QModelIndex parent = model.index(row, 0);
		model.setData(parent, item, Qt::DisplayRole);
		model.setData(parent, item, Qt::ToolTipRole);
		model.setData(parent, Qt::Checked, Qt::CheckStateRole);

		//read projector info
		int projectorWidth = 1024, projectorHeight = 768; //defaults compatible with old software
		QString projectorFilename = dirname + "/" + item + "/projector_info.txt";
		FILE* fp = fopen(qPrintable(projectorFilename), "r");
		if (fp)
		{   //projector info file exists
			int width, height;
			if (fscanf(fp, "%u %u", &width, &height) == 2 && width > 0 && height)
			{   //ok
				projectorWidth = width;
				projectorHeight = height;
				std::cerr << "Projector info file loaded: " << projectorFilename.toStdString() << std::endl;
			}
			else
			{
				std::cerr << "Projector info file has invalid values" << std::endl;
			}
			fclose(fp);
		}
		else
		{
			std::cerr << "Projector info file failed to open: " << projectorFilename.toStdString() << std::endl;
		}
		std::cerr << "Projector info file: using width=" << projectorWidth << " height=" << projectorHeight << std::endl;
		model.setData(parent, projectorWidth, ProjectorWidthRole);
		model.setData(parent, projectorHeight, ProjectorHeightRole);

		for (int i = 0; i < filecount; i++)
		{
			const QString& filename = filelist.at(i);
			if (!model.InsertRow(i, parent))
			{
				std::cout << "Failed model insert " << filename.toStdString() << "(" << row << ")" << std::endl;
				break;
			}

			QModelIndex index = model.index(i, 0, parent);
			QString label = QString("#%1 %2").arg(i, 2, 10, QLatin1Char('0')).arg(filename);
			model.setData(index, label, Qt::DisplayRole);
			model.setData(index, label, Qt::ToolTipRole);

			//additional data
			model.setData(index, path + "/" + filename, ImageFilenameRole);
		}
	}

	config.setValue(Settings::App::Root_Directory, dirname);
	emit root_dir_changed(dirname);
}

QString Application::GetRootDir(void) const
{
	return config.value(Settings::App::Root_Directory).toString();
}

bool Application::ChangeRootDir(QWidget* parentWidget)
{
	QString dirname = QFileDialog::getExistingDirectory(parentWidget, "Select Image Directory", config.value(Settings::App::Root_Directory, QString()).toString());

	if (dirname.isEmpty())
	{   //nothing selected
		return false;
	}

	SetRootDir(dirname);
	return true;
}

const cv::Mat Application::GetImage(unsigned level, unsigned n, Role role) const
{
	if (role != GrayImageRole && role != ColorImageRole)
	{   //invalid args
		return cv::Mat();
	}

	//try to load
	if (model.rowCount() < static_cast<int>(level))
	{   //out of bounds
		return cv::Mat();
	}
	QModelIndex parent = model.index(level, 0);
	if (model.rowCount(parent) < static_cast<int>(n))
	{   //out of bounds
		return cv::Mat();
	}

	QModelIndex index = model.index(n, 0, parent);
	if (!index.isValid())
	{   //invalid index
		return cv::Mat();
	}

	QString filename = model.data(index, ImageFilenameRole).toString();
	std::cout << "[" << (role == GrayImageRole ? "gray" : "color") << "] Filename: " << filename.toStdString() << std::endl;

	//load image
	cv::Mat rgbImage = cv::imread(filename.toStdString());
	if (rgbImage.rows > 0 && rgbImage.cols > 0)
	{
		//color
		if (role == ColorImageRole)
		{
			return rgbImage;
		}

		//gray scale
		if (role == GrayImageRole)
		{
			cv::Mat grayImage;
			cvtColor(rgbImage, grayImage, CV_BGR2GRAY);
			return grayImage;
		}
	}

	return cv::Mat();
}

int Application::GetCameraWidth(unsigned level) const
{
	return GetImage(level, 0, ColorImageRole).cols;
}

int Application::GetCameraHeight(unsigned level) const
{
	return GetImage(level, 0, ColorImageRole).rows;
}

int Application::GetProjectorWidth(unsigned level) const
{
	if (static_cast<int>(level) < model.rowCount())
	{   //ok
		QModelIndex parent = model.index(level, 0);
		return model.data(parent, ProjectorWidthRole).toInt();
	}
	return 0;
}

int Application::GetProjectorHeight(unsigned level) const
{
	if (static_cast<int>(level) < model.rowCount())
	{   //ok
		QModelIndex parent = model.index(level, 0);
		return model.data(parent, ProjectorHeightRole).toInt();
	}
	return 0;
}

bool Application::ExtractChessboardCorners(void)
{
	cornerCount = cv::Size(config.value(Settings::Chessboard::Columns).toUInt(), config.value(Settings::Chessboard::Rows).toUInt()); //interior number of corners
	cornerSize = cv::Size2f(config.value(Settings::Chessboard::Width).toDouble(), config.value(Settings::Chessboard::Height).toDouble());

	unsigned count = static_cast<unsigned>(model.rowCount());

	ProcessingSetProgressTotal(count);
	ProcessingSetProgressValue(0);
	ProcessingSetCurrentMessage("Extracting corners...");

	cornersWorld.clear();
	cornersCamera.clear();
	cornersWorld.resize(count);
	cornersCamera.resize(count);

	cv::Size imageSize(0, 0);
	int imageScale = 1;

	bool allFound = true;
	for (unsigned i = 0; i < count; i++)
	{
		QModelIndex index = model.index(i, 0);
		QString setName = model.data(index, Qt::DisplayRole).toString();
		bool checked = (model.data(index, Qt::CheckStateRole).toInt() == Qt::Checked);
		if (!checked)
		{   //skip
			ProcessingMessage(QString(" * %1: skip (not selected)").arg(setName));
			ProcessingSetProgressValue(i + 1);
			continue;
		}
		ProcessingSetCurrentMessage(QString("Extracting corners... %1").arg(setName));

		cv::Mat grayImage = GetImage(i, 1, GrayImageRole);
		if (grayImage.rows < 1)
		{
			ProcessingSetProgressValue(i + 1);
			continue;
		}

		if (imageSize.width == 0)
		{   //init image size
			imageSize = grayImage.size();
			if (imageSize.width > 1024)
			{
				imageScale = cvRound(imageSize.width / 1024.0);
			}
		}
		else if (imageSize != grayImage.size())
		{   //error
			std::cout << "ERROR: image of different size: set " << i << std::endl;
			return false;
		}

		cv::Mat smallImg;

		if (imageScale > 1)
		{
			cv::resize(grayImage, smallImg, cv::Size(grayImage.cols / imageScale, grayImage.rows / imageScale));
		}
		else
		{
			grayImage.copyTo(smallImg);
		}

		if (ProcessingCanceled())
		{
			ProcessingSetCurrentMessage("Extract corners canceled");
			ProcessingMessage("Extract corners canceled");
			return false;
		}

		//this will be filled by the detected corners
		std::vector<cv::Point2f>& camCorners = cornersCamera[i];
		std::vector<cv::Point3f>& worldCorners = cornersWorld[i];
		if (cv::findChessboardCorners(smallImg, cornerCount, camCorners,
			cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE /*+ cv::CALIB_CB_FILTER_QUADS*/))
		{
			ProcessingMessage(QString(" * %1: found %2 corners").arg(setName).arg(camCorners.size()));
			std::cout << " - corners: " << camCorners.size() << std::endl;

			GetChessboardWorldCoords(worldCorners, cornerCount, cornerSize);
		}

		for (std::vector<cv::Point2f>::iterator iter = camCorners.begin(); iter != camCorners.end(); iter++)
		{
			*iter = imageScale * (*iter);
		}
		if (camCorners.size())
		{
			cv::cornerSubPix(grayImage, camCorners, cv::Size(11, 11), cv::Size(-1, -1),
				cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
		}

		ProcessingSetProgressValue(i + 1);
	}

	ProcessingSetCurrentMessage("Extract corners finished");
	ProcessingSetProgressValue(count);
	return allFound;
}

void Application::DecodeAll(void)
{
	unsigned count = static_cast<unsigned>(model.rowCount());
	cv::Size imageSize(0, 0);

	ProcessingSetProgressTotal(count);
	ProcessingSetProgressValue(0);
	ProcessingSetCurrentMessage("Decoding...");

	patternList.resize(count);
	minMaxList.resize(count);

	QString path = config.value(Settings::App::Root_Directory).toString();

	//decode gray patterns
	for (unsigned i = 0; i < count; i++)
	{
		QModelIndex index = model.index(i, 0);
		QString setName = model.data(index, Qt::DisplayRole).toString();
		bool checked = (model.data(index, Qt::CheckStateRole).toInt() == Qt::Checked);
		if (!checked)
		{   //skip
			ProcessingMessage(QString(" * %1: skipped [not selected]").arg(setName));
			ProcessingSetProgressValue(i + 1);
			continue;
		}

		ProcessingSetCurrentMessage(QString("Decoding... %1").arg(setName));

		cv::Mat& patternImage = patternList[i];
		cv::Mat& minMaxImage = minMaxList[i];
		if (!DecodeGraySet(i, patternImage, minMaxImage))
		{   //error
			std::cout << "ERROR: Decode image set " << i << " failed. " << std::endl;
			return;
		}

		if (ProcessingCanceled())
		{
			ProcessingSetCurrentMessage("Decode canceled");
			ProcessingMessage("Decode canceled");
			return;
		}

		if (imageSize.width == 0)
		{
			imageSize = patternImage.size();
		}
		else if (imageSize != patternImage.size())
		{
			ProcessingMessage(QString("ERROR: pattern image of different size: set %1").arg(setName));
			std::cout << "ERROR: pattern image of different size: set " << i << std::endl;
			return;
		}
		else if (GetProjectorWidth(0) != GetProjectorWidth(i) || GetProjectorHeight(0) != GetProjectorHeight(i))
		{
			QString warningMessage = QString("WARNING: projector resolution does not match: set %1 [expected %2x%3, got %4x%5]").arg(setName)
				.arg(GetProjectorWidth(0)).arg(GetProjectorHeight(0)).arg(GetProjectorWidth(i)).arg(GetProjectorHeight(i));
			ProcessingMessage(warningMessage);
			std::cout << warningMessage.toStdString() << std::endl;
		}

		//save pattern image as PGM for debugging
		//QString filename = path + "/" + setName;
		//IoUtil::WritePgm(patternImage, qPrintable(filename));

		ProcessingMessage(QString(" * %1: decoded").arg(setName));
		ProcessingSetProgressValue(i + 1);
	}

	ProcessingSetCurrentMessage("Decode finished");
	ProcessingSetProgressValue(count);
}

void Application::Decode(int level, QWidget* parentWidget)
{
	if (level < 0 || level >= model.rowCount())
	{   //invalid row
		return;
	}
	if (patternList.size() < model.rowCount<size_t>())
	{
		patternList.resize(model.rowCount());
	}
	if (minMaxList.size() < model.rowCount<size_t>())
	{
		minMaxList.resize(model.rowCount());
	}

	cv::Mat& patternImage = patternList[level];
	cv::Mat& minMaxImage = minMaxList[level];

	if (!DecodeGraySet(level, patternImage, minMaxImage, parentWidget))
	{   //error
		std::cout << "ERROR: Decode image set " << level << " failed. " << std::endl;
	}
}

bool Application::DumpDecoded(const char* filename, int type, cv::Mat2f const& patternImage, cv::Mat2b const& minMaxImage, cv::Mat3b const& colorImage) const
{
	if (!filename || !patternImage.data || !minMaxImage.data || !colorImage.data)
	{
		return false;
	}

	FILE* fp = fopen(filename, "wb");
	if (!fp)
	{
		return false;
	}

	fwrite(&type, sizeof(int), 1, fp);
	fwrite(&patternImage.cols, sizeof(int), 1, fp);
	fwrite(&patternImage.rows, sizeof(int), 1, fp);

	//int index = type-1;

	//dump cols
	for (int h = 0; h < patternImage.rows; ++h)
	{
		cv::Vec2f const* row = patternImage.ptr<cv::Vec2f>(h);
		for (int w = 0; w < patternImage.cols; ++w)
		{
			float value = row[w][0];
			fwrite(&value, sizeof(float), 1, fp);
		}
	}

	//dump rows
	for (int h = 0; h < patternImage.rows; ++h)
	{
		cv::Vec2f const* row = patternImage.ptr<cv::Vec2f>(h);
		for (int w = 0; w < patternImage.cols; ++w)
		{
			float value = row[w][1];
			fwrite(&value, sizeof(float), 1, fp);
		}
	}

	//dump min
	for (int h = 0; h < minMaxImage.rows; ++h)
	{
		cv::Vec2b const* row = minMaxImage.ptr<cv::Vec2b>(h);
		for (int w = 0; w < minMaxImage.cols; ++w)
		{
			unsigned char value = row[w][0];
			fwrite(&value, sizeof(unsigned char), 1, fp);
		}
	}

	//dump max
	for (int h = 0; h < minMaxImage.rows; ++h)
	{
		cv::Vec2b const* row = minMaxImage.ptr<cv::Vec2b>(h);
		for (int w = 0; w < minMaxImage.cols; ++w)
		{
			unsigned char value = row[w][1];
			fwrite(&value, sizeof(unsigned char), 1, fp);
		}
	}

	//dump rgb
	for (int h = 0; h < colorImage.rows; ++h)
	{
		cv::Vec3b const* row = colorImage.ptr<cv::Vec3b>(h);
		for (int w = 0; w < colorImage.cols; ++w)
		{
			fwrite(&(row[w]), sizeof(unsigned char), 3, fp);
		}
	}

	fclose(fp);

	printf("saved file %s\n", filename);

	return true;
}

bool Application::LoadDump(const char* filename, int type, cv::Mat2f& patternImage, cv::Mat2b& minMaxImage, cv::Mat3b& colorImage) const
{
	if (!filename)
	{
		return false;
	}

	FILE* fp = fopen(filename, "rb");
	if (!fp)
	{
		return false;
	}

	int rows, cols;
	fread(&type, sizeof(int), 1, fp);
	fread(&cols, sizeof(int), 1, fp);
	fread(&rows, sizeof(int), 1, fp);

	patternImage.create(rows, cols);
	minMaxImage.create(rows, cols);
	colorImage.create(rows, cols);

	//dump cols
	for (int h = 0; h < patternImage.rows; ++h)
	{
		cv::Vec2f* row = patternImage.ptr<cv::Vec2f>(h);
		for (int w = 0; w < patternImage.cols; ++w)
		{
			float value;
			fread(&value, sizeof(float), 1, fp);
			row[w][0] = value;
		}
	}

	//dump rows
	for (int h = 0; h < patternImage.rows; ++h)
	{
		cv::Vec2f* row = patternImage.ptr<cv::Vec2f>(h);
		for (int w = 0; w < patternImage.cols; ++w)
		{
			float value;
			fread(&value, sizeof(float), 1, fp);
			row[w][1] = value;
		}
	}

	//dump min
	for (int h = 0; h < minMaxImage.rows; ++h)
	{
		cv::Vec2b* row = minMaxImage.ptr<cv::Vec2b>(h);
		for (int w = 0; w < minMaxImage.cols; ++w)
		{
			unsigned char value;
			fread(&value, sizeof(unsigned char), 1, fp);
			row[w][0] = value;
		}
	}

	//dump max
	for (int h = 0; h < minMaxImage.rows; ++h)
	{
		cv::Vec2b* row = minMaxImage.ptr<cv::Vec2b>(h);
		for (int w = 0; w < minMaxImage.cols; ++w)
		{
			unsigned char value;
			fread(&value, sizeof(unsigned char), 1, fp);
			row[w][1] = value;
		}
	}

	//dump rgb
	for (int h = 0; h < colorImage.rows; ++h)
	{
		cv::Vec3b* row = colorImage.ptr<cv::Vec3b>(h);
		fread(row, sizeof(unsigned char), 3 * cols, fp);
	}

	fclose(fp);

	printf("loaded file %s\n", filename);

	return true;
}

void Application::Calibrate(void)
{   //try to calibrate the camera, projector, and stereo system
	unsigned count = static_cast<unsigned>(model.rowCount());
	const unsigned threshold = config.value(Settings::Calibration::Shadow_Threshold, Settings::Calibration::Shadow_Threshold_Default_Value).toUInt();

	calib.Clear();

	std::cout << " shadow_threshold = " << threshold << std::endl;

	cv::Size imageSize(0, 0);

	//detect corners ////////////////////////////////////
	ProcessingMessage("Extracting corners:");
	if (!ExtractChessboardCorners())
	{
		return;
	}
	ProcessingMessage("");

	//collect projector correspondences
	cornersProjector.resize(count);
	patternList.resize(count);
	minMaxList.resize(count);

	ProcessingSetProgressTotal(count);
	ProcessingSetProgressValue(0);
	ProcessingSetCurrentMessage("Decoding and computing homographies...");

	for (unsigned i = 0; i < count; i++)
	{
		std::vector<cv::Point2f> const& camCorners = cornersCamera[i];
		std::vector<cv::Point2f>& projCorners = cornersProjector[i];

		QModelIndex index = model.index(i, 0);
		QString setName = model.data(index, Qt::DisplayRole).toString();
		bool checked = (model.data(index, Qt::CheckStateRole).toInt() == Qt::Checked);
		if (!checked)
		{   //skip
			ProcessingMessage(QString(" * %1: skip (not selected)").arg(setName));
			ProcessingSetProgressValue(i + 1);
			continue;
		}

		//checked: use this set
		projCorners.clear(); //erase previous points

		ProcessingSetCurrentMessage(QString("Decoding... %1").arg(setName));

		cv::Mat& patternImage = patternList[i];
		cv::Mat& minMaxImage = minMaxList[i];
		if (!DecodeGraySet(i, patternImage, minMaxImage))
		{   //error
			std::cout << "ERROR: Decode image set " << i << " failed. " << std::endl;
			return;
		}

		if (imageSize.width == 0)
		{
			imageSize = patternImage.size();
		}
		else if (imageSize != patternImage.size())
		{
			std::cout << "ERROR: pattern image of different size: set " << i << std::endl;
			return;
		}

		//cv::Mat outPatternImage = StructuredLight::Pixel_Uncertain*cv::Mat::ones(patternImage.size(), patternImage.type());

		ProcessingSetCurrentMessage(QString("Computing homographies... %1").arg(setName));

		for (std::vector<cv::Point2f>::const_iterator iter = camCorners.cbegin(); iter != camCorners.cend(); iter++)
		{
			const cv::Point2f& p = *iter;
			cv::Point2f q;

			if (ProcessingCanceled())
			{
				ProcessingSetCurrentMessage("Calibration canceled");
				ProcessingMessage("Calibration canceled");
				return;
			}
			processEvents();

			//find an homography around p
			unsigned WINDOW_SIZE = config.value(Settings::Calibration::H_Win, Settings::Calibration::H_Win_Default_Value).toUInt() / 2;
			std::vector<cv::Point2f> imgPoints, projPoints;
			if (p.x > WINDOW_SIZE && p.y > WINDOW_SIZE && p.x + WINDOW_SIZE < patternImage.cols && p.y + WINDOW_SIZE < patternImage.rows)
			{
				for (unsigned h = p.y - WINDOW_SIZE; h < p.y + WINDOW_SIZE; h++)
				{
					const cv::Vec2f* row = patternImage.ptr<cv::Vec2f>(h);
					const cv::Vec2b* minMaxRow = minMaxImage.ptr<cv::Vec2b>(h);
					//cv::Vec2f * outRow = outPatternImage.ptr<cv::Vec2f>(h);
					for (unsigned w = p.x - WINDOW_SIZE; w < p.x + WINDOW_SIZE; w++)
					{
						const cv::Vec2f& pattern = row[w];
						const cv::Vec2b& minMax = minMaxRow[w];
						//cv::Vec2f & outPattern = outRow[w];
						if (StructuredLight::Invalid(pattern))
						{
							continue;
						}
						if ((minMax[1] - minMax[0]) < static_cast<int>(threshold))
						{   //apply threshold and skip
							continue;
						}

						imgPoints.push_back(cv::Point2f(w, h));
						projPoints.push_back(cv::Point2f(pattern));

						//outPattern = pattern;
					}
				}
				cv::Mat H = cv::findHomography(imgPoints, projPoints, CV_RANSAC);
				//std::cout << " H:\n" << H << std::endl;
				cv::Point3d Q = cv::Point3d(cv::Mat(H * cv::Mat(cv::Point3d(p.x, p.y, 1.0))));
				q = cv::Point2f(Q.x / Q.z, Q.y / Q.z);
			}
			else
			{
				return;
			}

			//save
			projCorners.push_back(q);
		}

		ProcessingMessage(QString(" * %1: finished").arg(setName));
		ProcessingSetProgressValue(i + 1);
	}
	ProcessingMessage("");

	std::vector<std::vector<cv::Point3f> > worldCornersActive;
	std::vector<std::vector<cv::Point2f> > cameraCornersActive;
	std::vector<std::vector<cv::Point2f> > projectorCornersActive;
	worldCornersActive.reserve(count);
	cameraCornersActive.reserve(count);
	projectorCornersActive.reserve(count);
	for (unsigned i = 0; i < count; i++)
	{
		std::vector<cv::Point3f> const& worldCorners = cornersWorld.at(i);
		std::vector<cv::Point2f> const& camCorners = cornersCamera.at(i);
		std::vector<cv::Point2f> const& projCorners = cornersProjector.at(i);
		if (worldCorners.size() && camCorners.size() && projCorners.size())
		{   //active set
			worldCornersActive.push_back(worldCorners);
			cameraCornersActive.push_back(camCorners);
			projectorCornersActive.push_back(projCorners);
		}
	}

	if (worldCornersActive.size() < 3)
	{
		ProcessingSetCurrentMessage("ERROR: use at least 3 sets");
		ProcessingMessage("ERROR: use at least 3 sets");
		return;
	}

	int calFlags = 0
				  //+ cv::CALIB_FIX_K1
				  //+ cv::CALIB_FIX_K2
				  //+ cv::CALIB_ZERO_TANGENT_DIST
		+cv::CALIB_FIX_K3
		;

//calibrate the camera ////////////////////////////////////
	ProcessingMessage(QString(" * Calibrate camera [%1x%2]").arg(imageSize.width).arg(imageSize.height));
	std::vector<cv::Mat> camRvecs, camTvecs;
	int camFlags = calFlags;
	calib.camError = cv::calibrateCamera(worldCornersActive, cameraCornersActive, imageSize, calib.camK, calib.camKc, camRvecs, camTvecs, camFlags,
		cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 50, DBL_EPSILON));

//calibrate the projector ////////////////////////////////////
	cv::Size projectorSize(GetProjectorWidth(), GetProjectorHeight());
	ProcessingMessage(QString(" * Calibrate projector [%1x%2]").arg(projectorSize.width).arg(projectorSize.height));
	std::vector<cv::Mat> projRvecs, projTvecs;
	int projFlags = calFlags;
	calib.projError = cv::calibrateCamera(worldCornersActive, projectorCornersActive, projectorSize, calib.projK, calib.projKc, projRvecs, projTvecs, projFlags,
		cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 50, DBL_EPSILON));
/*
	//TMP: estimate an initial stereo R and T
	double errStereo = 0.0;
	std::vector<cv::Point3f> wpts;
	std::vector<cv::Point2f> cipts, pipts;
	for (size_t i=0; i<worldCornersActive.size(); ++i)
	{
	  auto const& worldCurr = worldCornersActive.at(i);
	  auto const& camCurr = cameraCornersActive.at(i);
	  auto const& projCurr = projectorCornersActive.at(i);
	  cv::Matx33d Rc; cv::Rodrigues(camRvecs.at(i), Rc); cv::Point3d Tc = camTvecs.at(i);
	  cv::Matx33d Rp; cv::Rodrigues(projRvecs.at(i), Rp); cv::Point3d Tp = projTvecs.at(i);
	  for (size_t j=0; j<worldCurr.size(); ++j)
	  {
		auto const& wpt = worldCurr.at(j);

		cv::Matx31d wptc = Rc*cv::Point3d(wpt) + Tc;
		wpts.push_back( cv::Point3f(wptc(0,0),wptc(1,0),wptc(2,0)) );

		cipts.push_back(camCurr.at(j));
		pipts.push_back(projCurr.at(j));
	  }
	}

	//cam Ransac
	cv::Vec3d camRvec, camTvec;
	cv::solvePnPRansac(wpts, cipts, calib.camK, calib.camKc, camRvec, camTvec);
	std::cerr << "Cam (Ransac) R,T: R " << camRvec << " T " << camTvec << std::endl;

	{ //reproj error
	  double errCam = 0.0;
	  std::vector<cv::Point2f> imgPts;
	  cv::projectPoints(wpts, camRvec, camTvec, calib.camK, calib.camKc, imgPts);
	  for (size_t i=0; i<imgPts.size(); ++i)
	  {
		auto const& p1 = imgPts.at(i);
		auto const& p2 = cipts.at(i);
		double err = cv::norm(p1-p2);
		errCam += err;
	  }
	  errCam /= imgPts.size();
	  std::cerr << " errCam (Ransac):  " << errCam << std::endl;
	}

	//cam LM
	bool rv1 = cv::solvePnP(wpts, cipts, calib.camK, calib.camKc, camRvec, camTvec, true);
	std::cerr << "Cam (LM) R,T: R " << camRvec << " T " << camTvec << std::endl;

	{ //reproj error
	  double errCam = 0.0;
	  std::vector<cv::Point2f> imgPts;
	  cv::projectPoints(wpts, camRvec, camTvec, calib.camK, calib.camKc, imgPts);
	  for (size_t i=0; i<imgPts.size(); ++i)
	  {
		auto const& p1 = imgPts.at(i);
		auto const& p2 = cipts.at(i);
		double err = cv::norm(p1-p2);
		errCam += err;
		errStereo += err;
	  }
	  errCam /= imgPts.size();
	  std::cerr << " errCam (LM):  " << errCam << std::endl;
	}

	//proj Ransac
	cv::Vec3d projRvec, projTvec;
	cv::solvePnPRansac(wpts, pipts, calib.projK, calib.projKc, projRvec, projTvec);
	std::cerr << "Proj (Ransac) R,T: R " << projRvec << " T " << projTvec << std::endl;

	{ //reproj error
	  double errPrj = 0.0;
	  std::vector<cv::Point2f> projPts;
	  cv::projectPoints(wpts, projRvec, projTvec, calib.projK, calib.projKc, projPts);
	  for (size_t i=0; i<projPts.size(); ++i)
	  {
		auto const& p1 = projPts.at(i);
		auto const& p2 = pipts.at(i);
		double err = cv::norm(p1-p2);
		errPrj += err;
	  }
	  errPrj /= projPts.size();
	  std::cerr << " errPrj (Ransac):  " << errPrj << std::endl;
	}

	//proj LM
	bool rv2 = cv::solvePnP(wpts, pipts, calib.projK, calib.projKc, projRvec, projTvec, true);
	std::cerr << "Proj (LM) R,T: R " << projRvec << " T " << projTvec << std::endl;

	{ //reproj error
	  double errPrj = 0.0;
	  std::vector<cv::Point2f> projPts;
	  cv::projectPoints(wpts, projRvec, projTvec, calib.projK, calib.projKc, projPts);
	  for (size_t i=0; i<projPts.size(); ++i)
	  {
		auto const& p1 = projPts.at(i);
		auto const& p2 = pipts.at(i);
		double err = cv::norm(p1-p2);
		errPrj += err;
		errStereo += err;
	  }
	  errPrj /= projPts.size();
	  std::cerr << " errPrj (LM):  " << errPrj << std::endl;
	}

	errStereo /= 2*wpts.size();;
	std::cerr << " errStereo:  " << errStereo << std::endl;
*/

	//stereo calibration
	ProcessingMessage(" * Calibrate stereo");
	cv::Mat E, F;
	calib.stereoError = cv::stereoCalibrate(worldCornersActive, cameraCornersActive, projectorCornersActive, calib.camK, calib.camKc, calib.projK, calib.projKc,
		imageSize /*ignored*/, calib.R, calib.T, E, F,
		cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 150, DBL_EPSILON),
		cv::CALIB_FIX_INTRINSIC /*cv::CALIB_USE_INTRINSIC_GUESS*/ + calFlags);
//print to console
	calib.Display();

	//print to GUI
	std::stringstream stream;
	calib.Display(stream);
	ProcessingMessage("\n **** Calibration results ****\n");
	ProcessingMessage(QString::fromStdString(stream.str()));

	//save to file
	QString path = config.value(Settings::App::Root_Directory).toString();
	QString filename = path + "/calibration.yml";
	if (calib.SaveCalibration(filename))
	{
		ProcessingMessage(QString("Calibration saved: %1").arg(filename));
	}
	else
	{
		ProcessingMessage(QString("[ERROR] Saving %1 failed").arg(filename));
	}

	//save to MATLAB format
	filename = path + "/calibration.m";
	if (calib.SaveCalibration(filename))
	{
		ProcessingMessage(QString("Calibration saved [MATLAB]: %1").arg(filename));
	}
	else
	{
		ProcessingMessage(QString("[ERROR] Saving %1 failed").arg(filename));
	}

	//save corners
	for (unsigned i = 0; i < count; i++)
	{
		std::vector<cv::Point3f> const& worldCorners = cornersWorld.at(i);
		std::vector<cv::Point2f> const& camCorners = cornersCamera.at(i);
		std::vector<cv::Point2f> const& projCorners = cornersProjector.at(i);

		QString filename0 = QString("%1/world_%2.txt").arg(path).arg(i, 2, 10, QLatin1Char('0'));
		FILE* fp0 = fopen(qPrintable(filename0), "w");
		if (!fp0)
		{
			std::cout << "ERROR: could no open " << filename0.toStdString() << std::endl;
			return;
		}
		QString filename1 = QString("%1/cam_%2.txt").arg(path).arg(i, 2, 10, QLatin1Char('0'));
		FILE* fp1 = fopen(qPrintable(filename1), "w");
		if (!fp1)
		{
			std::cout << "ERROR: could no open " << filename1.toStdString() << std::endl;
			return;
		}
		QString filename2 = QString("%1/proj_%2.txt").arg(path).arg(i, 2, 10, QLatin1Char('0'));
		FILE* fp2 = fopen(qPrintable(filename2), "w");
		if (!fp2)
		{
			fclose(fp1);
			std::cout << "ERROR: could no open " << filename2.toStdString() << std::endl;
			return;
		}

		std::cout << "Saved " << filename0.toStdString() << std::endl;
		std::cout << "Saved " << filename1.toStdString() << std::endl;
		std::cout << "Saved " << filename2.toStdString() << std::endl;

		std::vector<cv::Point3f>::const_iterator iter0 = worldCorners.begin();
		std::vector<cv::Point2f>::const_iterator iter1 = camCorners.begin();
		std::vector<cv::Point2f>::const_iterator iter2 = projCorners.begin();
		for (unsigned j = 0; j < worldCorners.size(); j++, ++iter0, ++iter1, ++iter2)
		{
			fprintf(fp0, "%lf %lf %lf\n", iter0->x, iter0->y, iter0->z);
			fprintf(fp1, "%lf %lf\n", iter1->x, iter1->y);
			fprintf(fp2, "%lf %lf\n", iter2->x, iter2->y);
		}
		fclose(fp0);
		fclose(fp1);
		fclose(fp2);
	}

	ProcessingMessage("Calibration finished");
}

bool Application::DecodeGraySet(unsigned level, cv::Mat& patternImage, cv::Mat& minMaxImage, QWidget* parentWidget) const
{
	if (model.rowCount() < static_cast<int>(level))
	{   //out of bounds
		return false;
	}

	patternImage = cv::Mat();
	minMaxImage = cv::Mat();

	//progress
	QProgressDialog* progress = NULL;
	if (parentWidget)
	{
		progress = new QProgressDialog("Decoding...", "Abort", 0, 100, parentWidget,
			Qt::Dialog | Qt::CustomizeWindowHint | Qt::WindowCloseButtonHint);
		progress->setWindowModality(Qt::WindowModal);
		progress->setWindowTitle("Processing");
		progress->setMinimumWidth(400);
		progress->show();
	}

	if (ProcessingCanceled() || (progress && progress->wasCanceled()))
	{   //abort
		ProcessingSetCurrentMessage("Decode canceled");
		ProcessingMessage("Decode canceled");
		if (progress)
		{
			progress->close();
			delete progress;
			progress = NULL;
		}
		return false;
	}
	processEvents();

	//parameters
	const float b = config.value(Settings::Decode::B, Settings::Decode::B_Default_Value).toFloat();
	const unsigned m = config.value(Settings::Decode::M, Settings::Decode::M_Default_Value).toUInt();

	//estimate direct component
	std::vector<cv::Mat> images;
	int totalImages = model.rowCount(model.index(level, 0));
	int totalPatterns = totalImages / 2 - 1;
	const int directLightCount = 4;
	const int directLightOffset = 4;
	if (totalPatterns < directLightCount + directLightOffset)
	{   //too few images
		ProcessingSetCurrentMessage("ERROR: too few pattern images");
		ProcessingMessage("ERROR: too few pattern images");
		return false;
	}
	if (progress)
	{
		progress->setLabelText("Decoding: estimating direct and global light components...");
		processEvents();
	}

	QList<unsigned> directComponentImages;
	for (unsigned i = 0; i < directLightCount; i++)
	{
		int index = totalImages - totalPatterns - directLightCount - directLightOffset + i + 1;
		directComponentImages.append(index);
		directComponentImages.append(index + totalPatterns);
	}
	//QList<unsigned> directComponentImages(QList<unsigned>() << 15 << 16 << 17 << 18 << 35 << 36 << 37 << 38);
	foreach(unsigned i, directComponentImages)
	{
		images.push_back(GetImage(level, i - 1));
	}
	cv::Mat directLight = StructuredLight::EstimateDirectLight(images, b);
	ProcessingMessage("Estimate direct and global light components... done.");

	if (progress)
	{
		progress->setValue(50);
		progress->setLabelText("Decoding: projector column and row values...");
		processEvents();
	}

	std::vector<std::string> imageNames;

	QModelIndex parent = model.index(level, 0);
	unsigned levelCount = static_cast<unsigned>(model.rowCount(parent));
	for (unsigned i = 0; i < levelCount; i++)
	{
		QModelIndex index = model.index(i, 0, parent);
		std::string filename = model.data(index, ImageFilenameRole).toString().toStdString();
		std::cout << "[decode_set " << level << "] Filename: " << filename << std::endl;

		imageNames.push_back(filename);
	}

	if (ProcessingCanceled() || (progress && progress->wasCanceled()))
	{   //abort
		ProcessingSetCurrentMessage("Decode canceled");
		ProcessingMessage("Decode canceled");
		if (progress)
		{
			progress->close();
			delete progress;
			progress = NULL;
		}
		return false;
	}
	processEvents();

	ProcessingMessage("Decoding, please wait...");
	cv::Size projectorSize(GetProjectorWidth(), GetProjectorHeight());
	bool rv = StructuredLight::DecodePattern(imageNames, patternImage, minMaxImage, projectorSize, StructuredLight::RobustDecode | StructuredLight::GrayPatternDecode, directLight, m);

	if (progress)
	{
		progress->setValue(100);
		progress->setLabelText(QString("Decoding: %1").arg((rv ? "finished" : "failed")));
		processEvents();

		progress->close();
		delete progress;
		progress = NULL;
		processEvents();
	}

	return rv;
}

bool Application::LoadCalibration(QWidget* parentWidget)
{
	QString name = config.value(Settings::Calibration::File, config.value(Settings::App::Root_Directory)).toString();
	QString filename = QFileDialog::getOpenFileName(parentWidget, "Open calibration", name, "Calibration (*.yml)");
	if (!filename.isEmpty() && calib.LoadCalibration(filename))
	{   //ok
		config.setValue(Settings::Calibration::File, filename);
		mainWin.show_message(QString("Calibration loaded from %1").arg(filename));
		calib.Display();
		return true;
	}
	if (!filename.isEmpty())
	{   //error
		QMessageBox::critical(parentWidget, "Error", QString("Calibration not loaded from %1").arg(filename));
	}
	return false;
}

bool Application::SaveCalibration(QWidget* parentWidget)
{
	if (!calib.IsValid())
	{   //invalid calibration
		QMessageBox::critical(parentWidget, "Error", "No valid calibration found.");
		return false;
	}
	QString name = config.value(Settings::Calibration::File, config.value(Settings::App::Root_Directory)).toString();
	QString filename = QFileDialog::getSaveFileName(parentWidget, "Save calibration", name, "Calibration (*.yml *.m)");
	if (!filename.isEmpty() && calib.SaveCalibration(filename))
	{   //ok
		config.setValue(Settings::Calibration::File, filename);
		mainWin.show_message(QString("Calibration saved to %1").arg(filename));
		calib.Display();
		return true;
	}
	if (!filename.isEmpty())
	{   //error
		QMessageBox::critical(parentWidget, "Error", QString("Calibration not saved to %1").arg(filename));
	}
	return false;
}

void Application::ReconstructModel(int level, Scan3d::Pointcloud& pointcloud, QWidget* parentWidget)
{
	//never keep the previous cloud: on any early return it would be saved (and compensated) again
	pointcloud.Clear();
	pointcloudGpCompensated = false;

	if (level < 0 || level >= model.rowCount())
	{   //invalid row
		return;
	}
	if (!calib.IsValid())
	{   //invalid calibration
		QMessageBox::critical(parentWidget, "Error", "No valid calibration found.");
		return;
	}

	//decode first
	Decode(level, parentWidget);
	if (patternList.size() <= static_cast<size_t>(level) || minMaxList.size() <= static_cast<size_t>(level))
	{   //error: decode failed
		return;
	}

	cv::Mat patternImage = patternList.at(level);
	cv::Mat minMaxImage = minMaxList.at(level);;
	cv::Mat colorImage = GetImage(level, 0, ColorImageRole);

	if (!patternImage.data || !minMaxImage.data)
	{   //error: decode failed
		return;
	}

	cv::Size projectorSize(GetProjectorWidth(), GetProjectorHeight());
	int threshold = config.value(Settings::Decode::Threshold, Settings::Decode::Threshold_Default_Value).toInt();;
	double maxDist = config.value(Settings::Reconstruction::Max_Dist, Settings::Reconstruction::Max_Dist_Default_Value).toDouble();;

	Scan3d::ReconstructModel(pointcloud, calib, patternImage, minMaxImage, colorImage, projectorSize, threshold, maxDist, parentWidget);

	//compensation: right after triangulation, before normals or any filtering
	pointcloudGpCompensated = ApplyGpCompensation(pointcloud, parentWidget);

	//debug: dump code to file
	/*
	QString path = config.value(Settings::App::Root_Directory).toString();
	QModelIndex index = model.index(level, 0);
	QString setName = model.data(index, Qt::DisplayRole).toString();
	DumpDecoded(qPrintable(QString("%1/%2/decode_dump.sl").arg(path).arg(setName)), 0, patternImage, minMaxImage, colorImage);
	*/

	//save the projector view
	if (projectorViewList.size() < model.rowCount<size_t>())
	{
		projectorViewList.resize(model.rowCount());
	}
	pointcloud.colors.copyTo(projectorViewList[level]);
}

void Application::ReconstructModelDump(cv::Mat2f const& patternImage, cv::Mat2b const& minMaxImage, cv::Mat3b const& colorImage, Scan3d::Pointcloud& pointcloud, QWidget* parentWidget)
{
	//never keep the previous cloud: on any early return it would be saved (and compensated) again
	pointcloud.Clear();
	pointcloudGpCompensated = false;

	if (!patternImage.data || !minMaxImage.data || !colorImage.data)
	{   //invalid dump
		return;
	}
	if (!calib.IsValid())
	{   //invalid calibration
		QMessageBox::critical(parentWidget, "Error", "No valid calibration found.");
		return;
	}

	cv::Size projectorSize(GetProjectorWidth(), GetProjectorHeight());
	int threshold = config.value(Settings::Decode::Threshold, Settings::Decode::Threshold_Default_Value).toInt();;
	double maxDist = config.value(Settings::Reconstruction::Max_Dist, Settings::Reconstruction::Max_Dist_Default_Value).toDouble();;

	Scan3d::ReconstructModel(pointcloud, calib, patternImage, minMaxImage, colorImage, projectorSize, threshold, maxDist, parentWidget);

	//compensation: right after triangulation, before normals or any filtering
	pointcloudGpCompensated = ApplyGpCompensation(pointcloud, parentWidget);
}

void Application::ComputeNormals(Scan3d::Pointcloud& pointcloud)
{
	Scan3d::ComputeNormals(pointcloud);
}

bool Application::LoadGpCompensation(QWidget* parentWidget)
{
	QString name = config.value(Settings::Compensation::Gp_File, config.value(Settings::App::Root_Directory)).toString();
	QString filename = QFileDialog::getOpenFileName(parentWidget, "Open GP compensation model", name, "GP model (*.yml)");
	if (filename.isEmpty())
	{
		return false;
	}

	std::string error;
	if (!gpCompensation.Load(filename.toStdString(), error))
	{
		QMessageBox::critical(parentWidget, "Error", QString("GP compensation model not loaded from %1\n%2").arg(filename).arg(QString::fromStdString(error)));
		return false;
	}
	config.setValue(Settings::Compensation::Gp_File, filename);
	config.setValue(Settings::Compensation::Gp_Enabled, true);

	if (calib.IsValid() && !gpCompensation.MatchesCalibration(calib.camK, calib.camKc, calib.projK, calib.projKc, calib.R, calib.T, error))
	{
		QMessageBox::warning(parentWidget, "GP compensation",
			QString("The model was loaded, but it does NOT match the current calibration:\n%1\n\n"
				"Pointclouds will be saved WITHOUT compensation until the calibration\n%2\nis loaded.")
			.arg(QString::fromStdString(error)).arg(QString::fromStdString(gpCompensation.CalibrationFilename())));
	}
	mainWin.show_message(QString("GP compensation loaded from %1").arg(filename));
	return true;
}

bool Application::ApplyGpCompensation(Scan3d::Pointcloud& pointcloud, QWidget* parentWidget)
{
	if (!config.value(Settings::Compensation::Gp_Enabled, Settings::Compensation::Gp_Enabled_Default_Value).toBool() || !pointcloud.points.data)
	{   //disabled, or reconstruction canceled
		return false;
	}

	std::string error;
	if (!gpCompensation.IsLoaded())
	{   //first use in this session: load the model saved in the configuration
		QString filename = config.value(Settings::Compensation::Gp_File).toString();
		if (filename.isEmpty() || !gpCompensation.Load(filename.toStdString(), error))
		{
			QMessageBox::warning(parentWidget, "GP compensation",
				QString("GP compensation model could not be loaded (%1).\nThe pointcloud is NOT compensated.")
				.arg(filename.isEmpty() ? QString("no model selected") : QString::fromStdString(error)));
			return false;
		}
	}
	if (!gpCompensation.MatchesCalibration(calib.camK, calib.camKc, calib.projK, calib.projKc, calib.R, calib.T, error))
	{
		QMessageBox::warning(parentWidget, "GP compensation",
			QString("%1\nThe pointcloud is NOT compensated.").arg(QString::fromStdString(error)));
		return false;
	}

	int outside = 0;
	int corrected = 0;
	{
		const BusyCursorGuard cursorGuard;
		processEvents();
		corrected = gpCompensation.Apply(pointcloud.points, &outside);
	}
	std::cout << "[GPCompensation] Corrected " << corrected << " points (" << outside
		<< " outside the training volume)" << std::endl;
	mainWin.show_message(QString("GP compensation applied to %1 points (%2 outside the training volume)").arg(corrected).arg(outside));
	return corrected >= 0;
}

void Application::CompensateXyzFile(QWidget* parentWidget)
{
	if (!gpCompensation.IsLoaded() && !LoadGpCompensation(parentWidget))
	{
		return;
	}
	QString input = QFileDialog::getOpenFileName(parentWidget, "Compensate pointcloud file", config.value(Settings::App::Root_Directory).toString(),
		"PolyWorks ASCII (*.xyz *.txt)");
	if (input.isEmpty())
	{
		return;
	}
	QFileInfo info(input);
	if (info.completeBaseName().endsWith("_gp", Qt::CaseInsensitive))
	{   //already compensated: compensating again would apply the correction twice
		QMessageBox::warning(parentWidget, "GP compensation",
			QString("%1\nis already compensated (name ends with _gp).\nSelect the original, uncompensated file.").arg(input));
		return;
	}
	QString output = QDir(info.absolutePath()).filePath(info.completeBaseName() + "_gp." + info.suffix());

	std::string error;
	int outside = 0;
	int n = -1;
	{
		const BusyCursorGuard cursorGuard;
		processEvents();
		n = gpCompensation.ApplyXyzFile(input.toStdString(), output.toStdString(), error, &outside);
	}
	if (n < 0)
	{
		QMessageBox::critical(parentWidget, "Error", QString::fromStdString(error));
		return;
	}
	mainWin.show_message(QString("Compensated %1 points (%2 outside the training volume): %3").arg(n).arg(outside).arg(output));
}

void Application::GetChessboardWorldCoords(std::vector<cv::Point3f>& worldCorners, cv::Size cornerCount, cv::Size cornerSize)
{
	//generate world object coordinates
	for (int h = 0; h < cornerCount.height; h++)
	{
		for (int w = 0; w < cornerCount.width; w++)
		{
			worldCorners.push_back(cv::Point3f(cornerSize.width * w, cornerSize.height * h, 0.f));
		}
	}
}

/// <summary>
/// Detects chessboard interior corners across all selected image sets and computes their corresponding world coordinates.
/// </summary>
/// <remarks>
/// For each selected set, loads the grayscale image, optionally downscales it for faster detection, runs OpenCV's findChessboardCorners with
/// FAST_CHECK for early rejection, scales the detected corners back to original resolution, and refines them with cornerSubPix. World coordinates are
/// generated assuming a planar checkerboard at Z=0.
/// Improvements over ExtractChessboardCorners: - Removed COGNEX dead code and cognexChessboard control variable. - Added CALIB_CB_FAST_CHECK flag
/// for faster rejection of images without a chessboard. - Avoided unnecessary deep copy when imageScale == 1 (direct reference instead). - Used
/// cv::INTER_AREA interpolation for higher-quality downscaling. - Reports a message when corners are not found in a set. - Properly marks allFound =
/// false when detection fails. - Uses bitwise OR for flags and modern cv::TermCriteria constants. - Uses range-based for and const qualifiers
/// following C++11 best practices.
/// </remarks>
bool Application::ExtractChessboardCornersV2(void)
{
	// Read checkerboard parameters from configuration.
	cornerCount = cv::Size(
		config.value(Settings::Chessboard::Columns).toUInt(),
		config.value(Settings::Chessboard::Rows).toUInt()
	);
	cornerSize = cv::Size2f(
		config.value(Settings::Chessboard::Width).toDouble(),
		config.value(Settings::Chessboard::Height).toDouble()
	);

	const unsigned count = static_cast<unsigned>(model.rowCount());

	// Initialize progress tracking.
	ProcessingSetProgressTotal(count);
	ProcessingSetProgressValue(0);
	ProcessingSetCurrentMessage("Extracting corners...");

	// Clear and allocate storage for results.
	cornersWorld.clear();
	cornersCamera.clear();
	cornersWorld.resize(count);
	cornersCamera.resize(count);

	cv::Size imageSize(0, 0);
	int imageScale = 1;
	bool allFound = true;

	// Detection flags for findChessboardCorners.
	const int detectFlags = cv::CALIB_CB_ADAPTIVE_THRESH
		| cv::CALIB_CB_NORMALIZE_IMAGE;

	for (unsigned i = 0; i < count; i++)
	{
		const QModelIndex index = model.index(i, 0);
		const QString setName = model.data(index, Qt::DisplayRole).toString();
		const bool checked = (model.data(index, Qt::CheckStateRole).toInt() == Qt::Checked);

		// Skip unselected sets.
		if (!checked)
		{
			ProcessingMessage(QString(" * %1: skip (not selected)").arg(setName));
			ProcessingSetProgressValue(i + 1);
			continue;
		}

		ProcessingSetCurrentMessage(QString("Extracting corners... %1").arg(setName));

		// Load the grayscale image for this set.
		const cv::Mat grayImage = GetImage(i, 1, GrayImageRole);
		if (grayImage.rows < 1)
		{
			ProcessingMessage(QString(" * %1: skip (failed to load image)").arg(setName));
			ProcessingSetProgressValue(i + 1);
			continue;
		}

		// Validate consistent image size across all sets; compute downscale factor on first image.
		if (imageSize.width == 0)
		{
			imageSize = grayImage.size();
			if (imageSize.width > 1024)
			{
				imageScale = cvRound(imageSize.width / 1024.0);
			}
		}
		else if (imageSize != grayImage.size())
		{
			ProcessingMessage(QString("ERROR: image of different size: set %1").arg(setName));
			return false;
		}

		// Downscale image for faster corner detection; use direct reference if scale is 1.
		cv::Mat smallImg;
		if (imageScale > 1)
		{
			cv::resize(grayImage, smallImg,
				cv::Size(grayImage.cols / imageScale, grayImage.rows / imageScale),
				0, 0, cv::INTER_AREA);
		}
		else
		{
			smallImg = grayImage;
		}

		// Check for user cancellation before the expensive detection step.
		if (ProcessingCanceled())
		{
			ProcessingSetCurrentMessage("Extract corners canceled");
			ProcessingMessage("Extract corners canceled");
			return false;
		}

		// Detect chessboard corners on the (possibly downscaled) image.
		std::vector<cv::Point2f>& camCorners = cornersCamera[i];
		std::vector<cv::Point3f>& worldCorners = cornersWorld[i];

		if (cv::findChessboardCorners(smallImg, cornerCount, camCorners, detectFlags))
		{
			ProcessingMessage(QString(" * %1: found %2 corners").arg(setName).arg(camCorners.size()));

			// Scale corners back to original image resolution.
			if (imageScale > 1)
			{
				for (auto& corner : camCorners)
				{
					corner *= static_cast<float>(imageScale);
				}
			}

			// Refine corner positions at sub-pixel accuracy on the full-resolution image.
			cv::cornerSubPix(grayImage, camCorners, cv::Size(11, 11), cv::Size(-1, -1),
				cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.1));

			// Generate planar world coordinates for the detected corners.
			GetChessboardWorldCoordsV2(worldCorners, cornerCount, cornerSize);
		}
		else
		{
			// No corners detected for this set.
			allFound = false;
			ProcessingMessage(QString(" * %1: chessboard not found!").arg(setName));
		}

		ProcessingSetProgressValue(i + 1);
	}

	ProcessingSetCurrentMessage("Extract corners finished");
	ProcessingSetProgressValue(count);
	return allFound;
}

/// <summary>
/// Generates planar world coordinates for a chessboard pattern assuming Z=0.
/// </summary>
/// <param name="worldCorners">Output vector of 3D world points.</param>
/// <param name="cornerCount">Interior corner grid dimensions (cols x rows).</param>
/// <param name="cornerSize">Physical size of each square (width x height) in mm.</param>
/// <remarks>
/// Improved over GetChessboardWorldCoords: - Accepts cv::Size2f to preserve floating-point precision for non-integer square sizes. - Uses
/// reserve() and emplace_back() to minimize memory reallocations.
/// </remarks>
void Application::GetChessboardWorldCoordsV2(std::vector<cv::Point3f>& worldCorners, cv::Size cornerCount, cv::Size2f cornerSize)
{
	worldCorners.reserve(cornerCount.width * cornerCount.height);
	for (int h = 0; h < cornerCount.height; h++)
	{
		for (int w = 0; w < cornerCount.width; w++)
		{
			worldCorners.emplace_back(cornerSize.width * w, cornerSize.height * h, 0.f);
		}
	}
}

void Application::MakePatternImages(int level, cv::Mat& colImage, cv::Mat& rowImage)
{
	colImage = cv::Mat();
	rowImage = cv::Mat();

	if (level < 0 || level >= model.rowCount())
	{   //invalid level
		return;
	}
	if (patternList.size() < static_cast<size_t>(level) || minMaxList.size() < static_cast<size_t>(level))
	{   //no decoded
		return;
	}

	cv::Mat const& patternImage = patternList.at(level);
	cv::Mat const& minMaxImage = minMaxList.at(level);

	if (!patternImage.data || !minMaxImage.data)
	{   //no decoded
		return;
	}

	//apply threshold
	int threshold = config.value(Settings::Decode::Threshold, Settings::Decode::Threshold_Default_Value).toInt();
	cv::Mat patternImageNew = cv::Mat(patternImage.size(), patternImage.type());
	for (int h = 0; h < patternImage.rows; h++)
	{
		const cv::Vec2f* patternRow = patternImage.ptr<cv::Vec2f>(h);
		const cv::Vec2b* minMaxRow = minMaxImage.ptr<cv::Vec2b>(h);
		cv::Vec2f* patternNewRow = patternImageNew.ptr<cv::Vec2f>(h);
		for (int w = 0; w < patternImage.cols; w++)
		{
			cv::Vec2f const& pattern = patternRow[w];
			cv::Vec2b const& minMax = minMaxRow[w];
			cv::Vec2f& patternNew = patternNewRow[w];

			if (StructuredLight::Invalid(pattern) || (minMax[1] - minMax[0]) < static_cast<int>(threshold))
			{   //invalid
				patternNew = cv::Vec2f(StructuredLight::Pixel_Uncertain, StructuredLight::Pixel_Uncertain);
			}
			else
			{   //ok
				patternNew = pattern;
			}
		}   //for each column
	}   //for each row

	colImage = StructuredLight::ColorizePattern(patternImageNew, 0, GetProjectorWidth(level));
	rowImage = StructuredLight::ColorizePattern(patternImageNew, 1, GetProjectorHeight(level));
}

cv::Mat Application::GetProjectorView(int level, bool forceUpdate)
{
	if (level < 0 || level >= model.rowCount())
	{   //invalid row
		return cv::Mat();
	}
	if (patternList.size() <= static_cast<size_t>(level))
	{   //not decoded
		return cv::Mat();
	}
	if (projectorViewList.size() < model.rowCount<size_t>())
	{
		projectorViewList.resize(model.rowCount());
	}

	cv::Mat& projector_image = projectorViewList[level];

	if (!projector_image.data || forceUpdate)
	{   //make projector view with the current configuration
		int threshold = config.value(Settings::Decode::Threshold, Settings::Decode::Threshold_Default_Value).toInt();

		cv::Mat patternImage = patternList.at(level);
		cv::Mat minMaxImage = minMaxList.at(level);;
		cv::Mat colorImage = GetImage(level, 0, ColorImageRole);
		cv::Size projectorSize(GetProjectorWidth(), GetProjectorHeight());

		projector_image = Scan3d::MakeProjectorView(patternImage, minMaxImage, colorImage, projectorSize, threshold);
	}

	return projector_image;
}

void Application::SelectNone(void)
{
	int count = model.rowCount();
	for (int i = 0; i < count; i++)
	{
		model.setData(model.index(i, 0), Qt::Unchecked, Qt::CheckStateRole);
	}
}

void Application::SelectAll(void)
{
	int count = model.rowCount();
	for (int i = 0; i < count; i++)
	{
		model.setData(model.index(i, 0), Qt::Checked, Qt::CheckStateRole);
	}
}
} // namespace smcp
