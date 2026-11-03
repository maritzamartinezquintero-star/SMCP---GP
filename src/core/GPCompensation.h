#pragma once

// Compensation of the reconstructed point cloud with the model trained in Python (gp_smcp.py):
//     p_f = p (1 + s(z, rho))                                   physical block (6 coefficients)
//     p  <- p_f - [ B (p_f - p0) + sum_j k52(p_f, P_j) W_j ]     GP residual field (one term per pass)
// Points are in the camera frame, in mm, right after triangulation. The model is loaded from the
// .yml written by exporta_modelo_yml.py and is only valid with the calibration it was trained with.

#include <string>
#include <vector>

#include <opencv2/core/core.hpp>

namespace smcp
{
class GPCompensation
{
public:
	/// <summary>
	/// Loads the model from a cv::FileStorage .yml. On failure the model stays unloaded.
	/// </summary>
	/// <param name="filename">Model file (modelo_gp_smcp.yml).</param>
	/// <param name="error">Reason of the failure, if any.</param>
	/// <returns>True if the model was loaded.</returns>
	bool Load(const std::string& filename, std::string& error);

	void Clear();
	bool IsLoaded() const { return _loaded; }
	const std::string& Filename() const { return _filename; }
	const std::string& CalibrationFilename() const { return _calibrationFilename; }

	/// <summary>
	/// Checks that the given calibration is the one the model was trained with.
	/// </summary>
	/// <returns>True if all matrices match (relative tolerance 1e-9).</returns>
	bool MatchesCalibration(const cv::Mat& camK, const cv::Mat& camKc, const cv::Mat& projK, const cv::Mat& projKc,
		const cv::Mat& R, const cv::Mat& T, std::string& error) const;

	/// <summary>
	/// Corrects one point (mm, camera frame).
	/// </summary>
	void CorrectPoint(const double in[3], double out[3]) const;

	/// <summary>
	/// Returns true if the point lies inside the volume covered by the training data (with margin).
	/// </summary>
	bool InsideTrainingVolume(const double p[3], double marginMm = 25.0) const;

	/// <summary>
	/// Corrects in place every valid (non-NaN) point of a CV_32FC3 point cloud. Runs in parallel.
	/// </summary>
	/// <param name="points">Point cloud matrix (CV_32FC3).</param>
	/// <param name="outside">Optional: number of corrected points outside the training volume.</param>
	/// <returns>Number of corrected points, or -1 if the model is not loaded or the matrix type is wrong.</returns>
	int Apply(cv::Mat& points, int* outside = nullptr) const;

	/// <summary>
	/// Corrects an ASCII file (X Y Z [extra columns]) and writes the result keeping the extra columns.
	/// Used to validate the C++ implementation against Python and to compensate scans saved before.
	/// </summary>
	/// <returns>Number of corrected points, or -1 on error.</returns>
	int ApplyXyzFile(const std::string& input, const std::string& output, std::string& error,
		int* outside = nullptr) const;

private:
	struct Pass
	{
		cv::Vec3d p0;
		cv::Matx33d B;
		std::vector<cv::Vec3d> supportScaled;   // P_rep / l
		std::vector<cv::Vec3d> weights;         // W
	};

	bool _loaded = false;
	std::string _filename;
	std::string _calibrationFilename;
	double _a[6] = { 0, 0, 0, 0, 0, 0 };
	double _muZ = 0.0;
	double _sdZ = 1.0;
	cv::Vec3d _l;
	cv::Vec3d _boxMin, _boxMax;
	std::vector<Pass> _passes;
	cv::Mat _calib[6];   // camK, camKc, projK, projKc, R, T
};
} // namespace smcp
