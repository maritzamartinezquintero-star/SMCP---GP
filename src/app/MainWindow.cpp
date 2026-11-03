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

#include "app/MainWindow.h"

#include <QMessageBox>
#include <QFileDialog>
#include <QButtonGroup>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <QPainter>
#include <QIntValidator>
#include <QDoubleValidator>

#include <algorithm>
#include <iostream>

#include "core/StructuredLight.h"

#include "app/Application.h"
#include "core/IoUtil.h"

#include "ui/AboutDialog.h"
#include "ui/CalibrationDialog.h"
#ifdef USE_SPINNAKER
#include "ui/CaptureDialog.h"
#endif
#include "export/IOExport.h"
#include "core/BusyCursorGuard.h"
#include "ui/pointcloud_editor/PointcloudEditorDialog.h"

namespace smcp
{

MainWindow::MainWindow(QWidget* parent, Qt::WindowFlags flags) :
	QMainWindow(parent, flags)
{
	setupUi(this);

#ifdef Q_OS_OSX
	image_tree->setFocusPolicy(Qt::ClickFocus);
#endif //Q_OS_OSX

	image_tree->setModel(&(APP->model));
	connect(APP, &Application::root_dir_changed, this, &MainWindow::_on_root_dir_changed);
	connect(image_tree->selectionModel(), &QItemSelectionModel::currentChanged,
		this, &MainWindow::_on_image_tree_currentChanged);

	QSettings& config = APP->config;

	setWindowTitle(Settings::App::Window_Name);

	threshold_spin->blockSignals(true);
	threshold_spin->setRange(0, 255);
	threshold_spin->setValue(config.value(Settings::Decode::Threshold, Settings::Decode::Threshold_Default_Value).toUInt());
	threshold_spin->blockSignals(false);

	b_line->blockSignals(true);
	b_line->setValidator(new QDoubleValidator(0.0, 1.0, 6, this));
	b_line->setText(config.value(Settings::Decode::B, Settings::Decode::B_Default_Value).toString());
	b_line->blockSignals(false);

	m_spin->blockSignals(true);
	m_spin->setRange(0, 255);
	m_spin->blockSignals(false);
	m_spin->setValue(config.value(Settings::Decode::M, Settings::Decode::M_Default_Value).toUInt());

	corner_count_x_spin->blockSignals(true);
	corner_count_x_spin->setRange(1, 255);
	corner_count_x_spin->blockSignals(false);
	corner_count_x_spin->setValue(config.value(Settings::Chessboard::Columns, Settings::Chessboard::Columns_Default_Value).toUInt());

	corner_count_y_spin->blockSignals(true);
	corner_count_y_spin->setRange(1, 255);
	corner_count_y_spin->blockSignals(false);
	corner_count_y_spin->setValue(config.value(Settings::Chessboard::Rows, Settings::Chessboard::Rows_Default_Value).toUInt());

	corners_width_line->blockSignals(true);
	corners_width_line->setValidator(new QDoubleValidator(this));
	corners_width_line->setText(config.value(Settings::Chessboard::Width, Settings::Chessboard::Width_Default_Value).toString());
	corners_width_line->blockSignals(false);

	corners_height_line->blockSignals(true);
	corners_height_line->setValidator(new QDoubleValidator(this));
	corners_height_line->setText(config.value(Settings::Chessboard::Height, Settings::Chessboard::Height_Default_Value).toString());
	corners_height_line->blockSignals(false);

	homography_window_spin->blockSignals(true);
	homography_window_spin->setRange(0, 1024);
	homography_window_spin->setValue(config.value(Settings::Calibration::H_Win, Settings::Calibration::H_Win_Default_Value).toUInt());
	homography_window_spin->blockSignals(false);

	//displayOriginalRadio->blockSignals(true);
	//displayOriginalRadio->setChecked(true);
	//displayOriginalRadio->blockSignals(false);

	max_dist_line->blockSignals(true);
	max_dist_line->setValidator(new QDoubleValidator(this));
	max_dist_line->setText(config.value(Settings::Reconstruction::Max_Dist, Settings::Reconstruction::Max_Dist_Default_Value).toString());
	max_dist_line->blockSignals(false);

	normals_check->blockSignals(true);
	normals_check->setChecked(config.value(Settings::Reconstruction::Save_Normals, Settings::Reconstruction::Save_Normals_Default_Value).toBool());
	normals_check->blockSignals(false);

	colors_check->blockSignals(true);
	colors_check->setChecked(config.value(Settings::Reconstruction::Save_Colors, Settings::Reconstruction::Save_Colors_Default_Value).toBool());
	colors_check->blockSignals(false);

	binary_file_check->blockSignals(true);
	binary_file_check->setChecked(config.value(Settings::Reconstruction::Save_Binary, Settings::Reconstruction::Save_Binary_Default_Value).toBool());
	binary_file_check->blockSignals(false);

	//set contextual menu for layout changing
	auto horizontalAction = new QAction("Horizontal", current_image_group);
	auto verticalAction = new QAction("Vertical", current_image_group);
	auto layoutActionGroup = new QActionGroup(current_image_group);
	layoutActionGroup->addAction(horizontalAction);
	layoutActionGroup->addAction(verticalAction);
	horizontalAction->setCheckable(true);
	verticalAction->setCheckable(true);
	horizontalAction->setChecked(true);
	verticalAction->setChecked(false);
	connect(horizontalAction, SIGNAL(triggered(bool)), this, SLOT(_on_horizontal_layout_action_triggered(bool)));
	connect(verticalAction, SIGNAL(triggered(bool)), this, SLOT(_on_vertical_layout_action_triggered(bool)));
	current_image_group->addActions(layoutActionGroup->actions());
	current_image_group->setContextMenuPolicy(Qt::ActionsContextMenu);

	// Group view buttons so only one can be checked at a time (exclusive behavior).
	auto viewButtonGroup = new QButtonGroup(this);
	viewButtonGroup->setExclusive(true);
	viewButtonGroup->addButton(image_view_button);
	viewButtonGroup->addButton(pattern_view_button);
	viewButtonGroup->addButton(projector_view_button);
	viewButtonGroup->addButton(view_3d_view_button);

	// GP compensation actions (Calibration menu).
	menuCalibration->addSeparator();
	auto loadGpAction = menuCalibration->addAction("Load GP compensation model...");
	auto applyGpAction = menuCalibration->addAction("Apply GP compensation on reconstruction");
	auto compensateFileAction = menuCalibration->addAction("Compensate existing .xyz file...");
	applyGpAction->setCheckable(true);
	applyGpAction->setChecked(config.value(Settings::Compensation::Gp_Enabled, Settings::Compensation::Gp_Enabled_Default_Value).toBool());
	connect(loadGpAction, &QAction::triggered, this, [this, applyGpAction]()
		{
			if (APP->LoadGpCompensation(this))
			{
				applyGpAction->setChecked(true);
			}
		});
	connect(applyGpAction, &QAction::toggled, this, [](bool checked)
		{
			APP->config.setValue(Settings::Compensation::Gp_Enabled, checked);
		});
	connect(compensateFileAction, &QAction::triggered, this, [this]()
		{
			APP->CompensateXyzFile(this);
		});

	// Set view.
	ResetView();

	show_message("Ready");
}

MainWindow::~MainWindow()
{}

void MainWindow::on_change_dir_action_triggered(bool checked)
{
	APP->ChangeRootDir(this);
}

void MainWindow::on_load_calibration_action_triggered(bool checked)
{
	APP->LoadCalibration(this);
	// TODO: Remove glwidget->updateCamera();
}

void MainWindow::on_save_calibration_action_triggered(bool checked)
{
	APP->SaveCalibration(this);
}

void MainWindow::on_display_calibration_action_triggered(bool checked)
{
	CalibrationDialog dialog(this, Qt::WindowCloseButtonHint);
	dialog.exec();
}

/// <summary>
/// Loads and displays the corresponding image(s) in the preview labels based on the active view.
/// </summary>
/// <param name="current">The newly selected model index.</param>
/// <param name="previous">The previously selected model index (not used).</param>
/// <remarks>
/// Resolves the set level and image row from the model hierarchy, then loads image content according to the active view button. Detected chessboard
/// corners (camera or projector) are overlaid as circles on the image when available. The busy cursor is managed automatically via BusyCursorGuard
/// (RAII).
/// </remarks>
void MainWindow::_on_image_tree_currentChanged(const QModelIndex& current, const QModelIndex& previous) const
{
	// Clear previous image content.
	image1_label->Clear();
	image2_label->Clear();

	if (!current.isValid())
	{
		return;
	}

	Application* const app = APP;

	// Resolve set level and image row from the model hierarchy.
	unsigned level = 0, row = 0;
	const QModelIndex parent = app->model.parent(current);
	if (parent.parent().isValid())
	{
		// Child item: parent is the set, current is the specific image.
		level = parent.row();
		row = current.row();
	}
	else
	{
		// Top-level item: treat as set root and show the first image.
		level = current.row();
		row = 0;
	}

	cv::Mat image1, image2;

	// Show busy cursor while loading image data (restored automatically on scope exit).
	const BusyCursorGuard cursorGuard;
	QApplication::processEvents();

	// Load image content based on the active view button (mutually exclusive).
	if (image_view_button->isChecked())
	{
		image1 = app->GetImage(level, row, ColorImageRole);
	}
	else if (pattern_view_button->isChecked() && app->patternList.size() > level)
	{
		app->MakePatternImages(level, image1, image2);
	}
	else if (projector_view_button->isChecked())
	{
		image1 = app->GetProjectorView(level);
	}

	// Resolve detected corners to overlay: camera corners for standard views,
	// projector corners (scaled to image resolution) for the projector view.
	std::vector<cv::Point2f> corners;
	if (!projector_view_button->isChecked() && level < app->cornersCamera.size())
	{
		corners = app->cornersCamera.at(level);
	}
	else if (projector_view_button->isChecked() && level < app->cornersProjector.size())
	{
		corners = app->cornersProjector.at(level);

		// Scale projector corner coordinates to match the displayed image resolution.
		const float scaleX = image1.cols * 1.f / app->GetProjectorWidth();
		const float scaleY = image1.rows * 1.f / app->GetProjectorHeight();
		for (auto& corner : corners)
		{
			corner.x *= scaleX;
			corner.y *= scaleY;
		}
	}

	// Overlay detected corners as red circles on each available image.
	if (!corners.empty())
	{
		const auto drawCorners = [&corners](cv::Mat& img)
			{
				if (!img.rows) { return; }
				img = img.clone();
				for (const auto& corner : corners)
				{
					cv::circle(img, corner, 6, CV_RGB(255, 0, 0), 2);
				}
			};

		drawCorners(image1);
		drawCorners(image2);
	}

	// Update the preview labels with the loaded images.
	image1_label->setPixmap(QPixmap::fromImage(IoUtil::ToQImage(image1)));
	image2_label->setPixmap(QPixmap::fromImage(IoUtil::ToQImage(image2)));
}

void MainWindow::show_message(const QString& message) const
{
	if (!message.isEmpty())
	{
		statusBar()->showMessage(message);
	}
	else
	{
		statusBar()->clearMessage();
	}
	QApplication::processEvents();
}

void MainWindow::on_corner_count_x_spin_valueChanged(int i)
{
	APP->config.setValue(Settings::Chessboard::Columns, i);
}

void MainWindow::on_corner_count_y_spin_valueChanged(int i)
{
	APP->config.setValue(Settings::Chessboard::Rows, i);
}

void MainWindow::on_corners_width_line_editingFinished() const
{
	APP->config.setValue(Settings::Chessboard::Width, corners_width_line->text().toDouble());
}

void MainWindow::on_corners_height_line_editingFinished() const
{
	APP->config.setValue(Settings::Chessboard::Height, corners_height_line->text().toDouble());
}

void MainWindow::on_threshold_button_clicked(bool checked)
{
	int row = GetCurrentSet();
	if (row < 0)
	{
		//nothing selected
		return;
	}

	APP->GetProjectorView(row, true);
	UpdateCurrentImage();
}

void MainWindow::on_threshold_spin_valueChanged(int i)
{
	APP->config.setValue(Settings::Decode::Threshold, i);
}

void MainWindow::on_b_line_editingFinished() const
{
	APP->config.setValue(Settings::Decode::B, b_line->text().toDouble());
}

void MainWindow::on_m_spin_valueChanged(int i)
{
	APP->config.setValue(Settings::Decode::M, i);
}

void MainWindow::on_homography_window_spin_valueChanged(int i)
{
	APP->config.setValue(Settings::Calibration::H_Win, i);
}

void MainWindow::on_max_dist_line_editingFinished() const
{
	APP->config.setValue(Settings::Reconstruction::Max_Dist, max_dist_line->text().toDouble());
}

void MainWindow::on_normals_check_stateChanged(int state)
{
	APP->config.setValue(Settings::Reconstruction::Save_Normals, (state == Qt::Checked));
}

void MainWindow::on_colors_check_stateChanged(int state)
{
	APP->config.setValue(Settings::Reconstruction::Save_Colors, (state == Qt::Checked));
}

void MainWindow::on_binary_file_check_stateChanged(int state)
{
	APP->config.setValue(Settings::Reconstruction::Save_Binary, (state == Qt::Checked));
}

void MainWindow::on_quit_action_triggered(bool checked)
{
	close();
	APP->quit();
}

void MainWindow::on_save_vertical_image_action_triggered(bool checked)
{
	if (image1_label->pixmap()->isNull())
	{
		QMessageBox::critical(this, "Error", "Vertical image is empy.");
		return;
	}
	QString filename = QFileDialog::getSaveFileName(this, "Save vertical image", "saved_image_vertical.png",
		"Images (*.png)");
	if (!filename.isEmpty())
	{
		image1_label->pixmap()->save(filename);
		show_message(QString("Vertical Image saved: %1").arg(filename));
	}
}

void MainWindow::on_save_horizontal_image_action_triggered(bool checked)
{
	if (image2_label->pixmap()->isNull())
	{
		QMessageBox::critical(this, "Error", "Horizontal image is empy.");
		return;
	}
	QString filename = QFileDialog::getSaveFileName(this, "Save horizontal image", "saved_image_horizontal.png",
		"Images (*.png)");
	if (!filename.isEmpty())
	{
		image2_label->pixmap()->save(filename);
		show_message(QString("Horizontal Image saved: %1").arg(filename));
	}
}

void MainWindow::_on_root_dir_changed(const QString& dirname)
{
	//update user interface
	image1_label->Clear();
	image2_label->Clear();

	ResetView();

	setWindowTitle(QString("%1 - %2").arg(Settings::App::Window_Name, dirname));

	QModelIndex index = APP->model.index(0, 0);
	image_tree->selectionModel()->clearSelection();
	if (APP->model.rowCount() > 0)
	{
		image_tree->blockSignals(true);
		image_tree->selectionModel()->select(index, QItemSelectionModel::SelectCurrent);
		_on_image_tree_currentChanged(index, index);
		image_tree->blockSignals(false);
		show_message(QString("%1 set read").arg(APP->model.rowCount()));
	}
}

void MainWindow::on_about_action_triggered(bool checked)
{
	AboutDialog dialog(this, Qt::WindowCloseButtonHint);
	dialog.exec();
}

void MainWindow::UpdateCurrentImage(QModelIndex current)
{
	QModelIndex index = current;
	if (!index.isValid())
	{
		index = image_tree->selectionModel()->currentIndex();
	}
	if (!index.isValid())
	{
		index = APP->model.index(0, 0);
	}
	_on_image_tree_currentChanged(index, index);
}

void MainWindow::_on_horizontal_layout_action_triggered(bool checked) const
{
	QLayout* oldLayout = current_image_group->layout();
	if (oldLayout)
	{
		delete oldLayout;
	}
	auto newLayout = new QHBoxLayout(current_image_group);
	newLayout->addWidget(image1_label);
	newLayout->addWidget(image2_label);
	newLayout->addWidget(pointcloud_preview);
	current_image_group->setLayout(newLayout);
}

void MainWindow::_on_vertical_layout_action_triggered(bool checked) const
{
	QLayout* oldLayout = current_image_group->layout();
	if (oldLayout)
	{
		delete oldLayout;
	}
	auto newLayout = new QVBoxLayout(current_image_group);
	newLayout->addWidget(image1_label);
	newLayout->addWidget(image2_label);
	newLayout->addWidget(pointcloud_preview);
	current_image_group->setLayout(newLayout);
}

void MainWindow::on_reconstruct_dump_action_triggered(bool checked)
{
	QString rootDir = APP->GetRootDir();
	QString filename = QFileDialog::getOpenFileName(this, "Open decoded files", rootDir, "Decoded dump (*.sl)");
	if (filename.isEmpty())
	{
		return;
	}

	int type = 0;
	cv::Mat2f patternImage;
	cv::Mat2b minMaxImage;
	cv::Mat3b colorImage;
	APP->LoadDump(qPrintable(filename), type, patternImage, minMaxImage, colorImage);

	show_message("Reconstruction...");

	//parameters
	bool normals = APP->config.value(Settings::Reconstruction::Save_Normals, Settings::Reconstruction::Save_Normals_Default_Value).toBool();
	bool colors = APP->config.value(Settings::Reconstruction::Save_Colors, Settings::Reconstruction::Save_Colors_Default_Value).toBool();
	bool binary = APP->config.value(Settings::Reconstruction::Save_Binary, Settings::Reconstruction::Save_Binary_Default_Value).toBool();

	Scan3d::Pointcloud& pointcloud = APP->pointcloud;
	APP->ReconstructModelDump(patternImage, minMaxImage, colorImage, pointcloud, this);

	if (!pointcloud.points.data)
	{
		//no points: reconstruction canceled or failed
		show_message("Reconstruction failed");
		return;
	}

	//compute normals
	if (normals)
	{
		//busy cursor
		show_message("Computing normals...");
		QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));
		QApplication::processEvents();

		APP->ComputeNormals(pointcloud);

		//restore regular cursor
		QApplication::restoreOverrideCursor();
		QApplication::processEvents();
	}

	//save the points
	QString name = rootDir + (APP->pointcloudGpCompensated ? "/pointcloud_gp" : "/pointcloud");
	filename = QFileDialog::getSaveFileName(this, "Save pointcloud", name + ".ply", "Pointclouds (*.ply)");
	if (!filename.isEmpty())
	{
		//busy cursor
		show_message(QString("Saving to %1...").arg(filename));
		QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));
		QApplication::processEvents();

		unsigned plyFlags = IoUtil::PlyPoints
			| (colors ? IoUtil::PlyColors : 0)
			| (normals ? IoUtil::PlyNormals : 0)
			| (binary ? IoUtil::PlyBinary : 0);

		IoUtil::WritePly(filename.toStdString(), pointcloud, plyFlags);

		//restore regular cursor
		QApplication::restoreOverrideCursor();
		QApplication::processEvents();
		show_message(QString("Pointcloud saved: %1").arg(filename));
		std::cout << QString("Pointcloud saved: %1").arg(filename).toStdString() << std::endl;
	}
}

int MainWindow::GetCurrentSet(void) const
{
	QModelIndex index = image_tree->selectionModel()->currentIndex();
	if (!index.isValid())
	{
		index = APP->model.index(0, 0);
	}
	while (index.parent().parent().isValid())
	{
		index = index.parent();
	}
	if (!index.isValid())
	{
		return -1;
	}
	return index.row();
}

void MainWindow::on_select_all_button_clicked(bool checked)
{
	APP->SelectAll();
}

void MainWindow::on_select_none_button_clicked(bool checked)
{
	APP->SelectNone();
}

/* ACTION BUTTONS ========================================================================== */

/// <summary>
/// Opens the capture dialog to configure and execute a new image acquisition session.
/// </summary>
/// <param name="clicked">Indicates whether the button is in a clicked state (not used).</param>
/// <remarks>
/// The dialog is opened in modal mode, blocking interaction with the main window until the capture session is completed or dismissed by the user.
/// </remarks>
void MainWindow::on_capture_action_button_clicked(bool clicked)
{
#ifdef USE_SPINNAKER
	// Create and open the capture dialog in modal mode.
	CaptureDialog dialog(this);
	dialog.setModal(true);
	dialog.exec();
#else
	QMessageBox::information(this, tr("Capture"),
		tr("This build does not include camera support (SMCP_WITH_SPINNAKER=OFF)."));
#endif
}

/// <summary>
/// Runs the chessboard corner detection pipeline and updates the current image view with the results.
/// </summary>
/// <param name="checked">Indicates whether the button is in a checked state (not used).</param>
/// <remarks>
/// Resets any previous processing state before running corner detection. Once the extraction completes, the current image is refreshed to reflect the
/// detected corners. Progress and output are shown through the processing dialog.
/// </remarks>
void MainWindow::on_extract_corners_action_button_clicked(bool checked)
{
	show_message("Searching chessboard corners...");

	// Reset previous processing state and prepare the dialog for the corner detection run.
	APP->ProcessingReset();
	APP->processingDialog.setWindowTitle("Corner detection");
	APP->processingDialog.show();
	QApplication::processEvents();

	// Execute chessboard corner extraction and refresh the image view with the results.
	APP->ExtractChessboardCornersV2();
	UpdateCurrentImage();

	// Mark processing as finished, display the dialog with results and wait for dismissal.
	APP->processingDialog.Finish();
	APP->processingDialog.exec();
	APP->processingDialog.hide();
	QApplication::processEvents();

	ResetView();
	show_message("Ready");
}

/// <summary>
/// Runs the full decode pipeline over all captures and updates the current image view with the results.
/// </summary>
/// <param name="checked">Indicates whether the button is in a checked state (not used).</param>
/// <remarks>
/// Resets any previous processing state before decoding. Once decoding completes, the current image is refreshed to reflect the updated results.
/// Progress and output are shown through the processing dialog.
/// </remarks>
void MainWindow::on_decode_action_button_clicked(bool checked)
{
	show_message("Decoding...");

	// Reset previous processing state and prepare the dialog for the decode run.
	APP->ProcessingReset();
	APP->processingDialog.setWindowTitle("Decode");
	APP->processingDialog.show();
	QApplication::processEvents();

	// Execute decoding over all captures and refresh the image view.
	APP->DecodeAll();
	UpdateCurrentImage();

	// Mark processing as finished, display the dialog with results and wait for dismissal.
	APP->processingDialog.Finish();
	APP->processingDialog.exec();
	APP->processingDialog.hide();
	QApplication::processEvents();

	ResetView();
	show_message("Ready");
}

/// <summary>
/// Executes the full calibration pipeline and reflects the results in the current view.
/// </summary>
/// <param name="checked">Indicates whether the button is in a checked state (not used).</param>
/// <remarks>
/// Resets any previous processing state before running calibration. Once the calibration completes, the current image and camera are refreshed to
/// reflect the updated results. Progress and output are shown through the processing dialog.
/// </remarks>
void MainWindow::on_calibration_action_button_clicked(bool checked)
{
	show_message("Running calibration...");

	// Reset previous processing state and prepare the dialog for the calibration run.
	APP->ProcessingReset();
	APP->processingDialog.setWindowTitle("Calibration");
	APP->processingDialog.show();
	QApplication::processEvents();

	// Execute calibration and refresh the image and camera to reflect updated results.
	APP->Calibrate();
	UpdateCurrentImage();
	// TODO: Remove glwidget->updateCamera();

	// Mark processing as finished, display the dialog with results and wait for dismissal.
	APP->processingDialog.Finish();
	APP->processingDialog.exec();
	APP->processingDialog.hide();
	QApplication::processEvents();

	ResetView();
	show_message("Ready");
}

/// <summary>
/// Slot triggered when the reconstruction action button is clicked. Runs the 3D reconstruction pipeline for the currently selected set and saves the
/// resulting pointcloud to disk in the format chosen by the user.
/// </summary>
/// <param name="checked">Indicates whether the button is in a checked state (not used).</param>
/// <remarks>
/// Requires a set to be selected; aborts silently if none is active. After reconstruction, optionally computes normals based on the current
/// configuration. Prompts the user with a save dialog supporting XYZ (PolyWorks ASCII) and PLY (ASCII or Binary) formats. On success, resets the view
/// to reflect the updated processing state.
/// </remarks>
void MainWindow::on_reconstruction_action_button_clicked(bool checked)
{
	int row = GetCurrentSet();
	if (row < 0)
	{
		// Abort if no set is selected.
		return;
	}

	show_message("Reconstruction...");

	// Read export configuration flags.
	bool normals = APP->config.value(Settings::Reconstruction::Save_Normals, Settings::Reconstruction::Save_Normals_Default_Value).toBool();
	bool colors = APP->config.value(Settings::Reconstruction::Save_Colors, Settings::Reconstruction::Save_Colors_Default_Value).toBool();
	bool binary = APP->config.value(Settings::Reconstruction::Save_Binary, Settings::Reconstruction::Save_Binary_Default_Value).toBool();

	// Run reconstruction and validate that points were produced.
	Scan3d::Pointcloud& pointcloud = APP->pointcloud;
	APP->ReconstructModel(row, pointcloud, this);
	if (!pointcloud.points.data)
	{
		// Reconstruction was canceled or failed; no pointcloud available.
		show_message("Reconstruction failed");
		return;
	}

	// Optionally compute surface normals from the reconstructed pointcloud.
	if (normals)
	{
		show_message("Computing normals...");
		QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));
		QApplication::processEvents();

		APP->ComputeNormals(pointcloud);

		QApplication::restoreOverrideCursor();
		QApplication::processEvents();
	}

	// Build default filename from the scan set name.
	const QString setName = APP->model.data(APP->model.index(row, 0), Qt::DisplayRole).toString();
	const QString defaultPath = QDir(APP->GetRootDir()).filePath(setName + (APP->pointcloudGpCompensated ? "_gp" : ""));

	// Offer both PLY and XYZ formats in the save dialog.
	const QString filter = "PolyWorks ASCII (*.xyz);;PLY ASCII (*.ply);;PLY Binary (*.ply)";
	QString selectedFilter;
	const QString filename = QFileDialog::getSaveFileName(
		this, "Save pointcloud", defaultPath + ".xyz", filter, &selectedFilter);

	if (filename.isEmpty())
	{
		return;
	}

	// Save based on selected format.
	show_message(QStringLiteral("Saving to %1...").arg(filename));
	{
		const BusyCursorGuard cursorGuard;
		QApplication::processEvents();

		bool success = false;

		if (selectedFilter.startsWith("PolyWorks"))
		{
			// Direct ASCII export.
			const auto xyzFormat = colors
				? IOExport::XyzFormat::XyzRgb
				: IOExport::XyzFormat::Xyz;
			success = IOExport::WriteXyz(filename.toStdString(), pointcloud, xyzFormat);
		}
		else
		{
			// Standard PLY export.
			const bool isBinary = selectedFilter.contains("Binary");
			unsigned plyFlags = IoUtil::PlyPoints
				| (colors ? IoUtil::PlyColors : 0)
				| (normals ? IoUtil::PlyNormals : 0)
				| (isBinary ? IoUtil::PlyBinary : 0);
			success = IoUtil::WritePly(filename.toStdString(), pointcloud, plyFlags);
		}

		if (!success)
		{
			show_message("Failed to save pointcloud.");
			return;
		}
	}

	const QString message = QStringLiteral("Pointcloud saved: %1").arg(filename);
	show_message(message);
	qInfo().noquote() << message;

	ResetView();
}

/// <summary>
/// Opens the point cloud editor (modal) over the .xyz files of the working directory:
/// outlier removal, RANSAC plane/sphere fitting, 3D comparison and export.
/// </summary>
/// <param name="checked">Indicates whether the button is in a checked state (not used).</param>
void MainWindow::on_pointcloud_action_button_clicked(bool checked)
{
	PointcloudEditorDialog::Execute(APP->GetRootDir(), this);
}

/* PREVIEW BUTTONS ========================================================================= */

/// <summary>
/// Updates the visibility of the main view components based on the requested view type.
/// </summary>
/// <param name="view">The view type to activate (Image, Pattern, Projector, or View3D).</param>
void MainWindow::SetView(ViewType view)
{
	// Resolve visibility flags for each component based on the requested view type.
	bool showImage1 = false;
	bool showImage2 = false;
	bool showPointcloudPreview = false;

	switch (view)
	{
	case ViewType::Image:
		showImage1 = true;
		break;

	case ViewType::Pattern:
		showImage1 = true;
		showImage2 = true;
		break;

	case ViewType::Projector:
		showImage1 = true;
		break;

	case ViewType::View3D:
		showPointcloudPreview = true;
		break;
	}

	// Apply visibility to each component.
	image1_label->setVisible(showImage1);
	image2_label->setVisible(showImage2);
	pointcloud_preview->setVisible(showPointcloudPreview);
}

/// <summary>
/// Resets the active view to Image and updates the enabled state of all view buttons based on the current processing state.
/// </summary>
/// <remarks>
/// Enables pattern_view_button and projector_view_button only when at least one decoded pattern is available in patternList. Enables
/// view_3d_view_button only when a reconstructed pointcloud exists. Called after every pipeline step and during initialization to keep the UI
/// consistent with the application state.
/// </remarks>
void MainWindow::ResetView()
{
	// Determine which views are available based on current processing state.
	const bool decoded = std::any_of(APP->patternList.begin(), APP->patternList.end(),
		[](const cv::Mat& m) { return !m.empty(); });
	const bool reconstructed = !APP->pointcloud.points.empty();

	// Enable/disable view buttons based on available data.
	pattern_view_button->setEnabled(decoded);
	projector_view_button->setEnabled(decoded);
	view_3d_view_button->setEnabled(reconstructed);

	// Reset active view to image.
	image_view_button->click();
}

/// <summary>
/// Switches the preview to display the original captured image.
/// </summary>
/// <param name="checked">True when the button transitions to checked state.</param>
void MainWindow::on_image_view_button_clicked(bool checked)
{
	if (checked)
	{
		SetView(ViewType::Image);
		UpdateCurrentImage();
	}
}

/// <summary>
/// Switches the preview to display the decoded Gray code pattern images (horizontal and vertical).
/// </summary>
/// <param name="checked">True when the button transitions to checked state.</param>
void MainWindow::on_pattern_view_button_clicked(bool checked)
{
	if (checked)
	{
		SetView(ViewType::Pattern);
		UpdateCurrentImage();
	}
}

/// <summary>
/// Switches the preview to display the projector view reconstructed from the decoded patterns.
/// </summary>
/// <param name="checked">True when the button transitions to checked state.</param>
void MainWindow::on_projector_view_button_clicked(bool checked)
{
	if (checked)
	{
		SetView(ViewType::Projector);
		UpdateCurrentImage();
	}
}

/// <summary>
/// Switches the preview to display the reconstructed 3D pointcloud in the OpenGL widget.
/// </summary>
/// <param name="checked">True when the button transitions to checked state.</param>
void MainWindow::on_view_3d_view_button_clicked(bool checked)
{
	if (checked)
	{
		SetView(ViewType::View3D);
		UpdateCurrentImage();

		pointcloud_preview->loadPointcloud();
	}
}
} // namespace smcp
