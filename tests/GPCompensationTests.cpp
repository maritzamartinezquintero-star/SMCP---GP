#include <cmath>
#include <algorithm>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/core/core.hpp>

#include "core/GPCompensation.h"

// Checks the C++ GP compensation against reference values computed in Python
// (exporta_modelo_yml.aplica with modelo_gp_smcp.npz). Arguments: model .yml, reference .txt.
namespace
{
	bool Check(bool condition, const char* message)
	{
		if (!condition) std::cerr << "FAIL: " << message << '\n';
		return condition;
	}

	struct Reference
	{
		double in[3];
		double expected[3];
	};

	std::vector<Reference> ReadReference(const std::string& filename)
	{
		std::vector<Reference> rows;
		std::ifstream file(filename);
		std::string line;
		while (std::getline(file, line))
		{
			if (line.empty() || line[0] == '#') continue;
			std::istringstream ss(line);
			Reference r;
			if (ss >> r.in[0] >> r.in[1] >> r.in[2] >> r.expected[0] >> r.expected[1] >> r.expected[2])
			{
				rows.push_back(r);
			}
		}
		return rows;
	}

	double DistanceUm(const double a[3], const double b[3])
	{
		const double dx = a[0] - b[0], dy = a[1] - b[1], dz = a[2] - b[2];
		return std::sqrt(dx * dx + dy * dy + dz * dz) * 1000.0;
	}
}

int main(int argc, char* argv[])
{
	if (argc < 3)
	{
		std::cerr << "usage: SMCP_gp_tests <modelo_gp_smcp.yml> <gp_reference.txt>\n";
		return 2;
	}
	const std::string modelFile = argv[1];
	const std::vector<Reference> reference = ReadReference(argv[2]);
	bool success = Check(reference.size() >= 3, "reference points are read");

	smcp::GPCompensation gp;
	std::string error;
	success &= Check(gp.Load(modelFile, error), "model loads");
	if (!gp.IsLoaded())
	{
		std::cerr << error << '\n';
		return 1;
	}

	// Point by point: same tolerance as valida_cpp_vs_python.py (0.01 um).
	double maxUm = 0.0;
	for (const Reference& r : reference)
	{
		double out[3];
		gp.CorrectPoint(r.in, out);
		maxUm = std::max(maxUm, DistanceUm(out, r.expected));
	}
	std::cout << "CorrectPoint vs Python: max difference " << maxUm << " um\n";
	success &= Check(maxUm < 0.01, "CorrectPoint matches Python within 0.01 um");

	// The last two reference points lie outside the training volume by construction.
	const size_t n = reference.size();
	success &= Check(gp.InsideTrainingVolume(reference[0].in), "first point is inside the training volume");
	success &= Check(!gp.InsideTrainingVolume(reference[n - 1].in) && !gp.InsideTrainingVolume(reference[n - 2].in),
		"points beyond the margin are outside the training volume");

	// Whole cloud (CV_32FC3): NaN points are skipped and left untouched.
	cv::Mat points(1, static_cast<int>(n) + 1, CV_32FC3);
	for (size_t i = 0; i < n; i++)
	{
		points.at<cv::Vec3f>(0, static_cast<int>(i)) = cv::Vec3f(
			static_cast<float>(reference[i].in[0]), static_cast<float>(reference[i].in[1]), static_cast<float>(reference[i].in[2]));
	}
	const float nan = std::numeric_limits<float>::quiet_NaN();
	points.at<cv::Vec3f>(0, static_cast<int>(n)) = cv::Vec3f(nan, nan, nan);
	int outside = -1;
	const int corrected = gp.Apply(points, &outside);
	success &= Check(corrected == static_cast<int>(n), "Apply corrects every valid point");
	success &= Check(outside == 2, "Apply counts the points outside the training volume");
	success &= Check(std::isnan(points.at<cv::Vec3f>(0, static_cast<int>(n))[0]), "Apply leaves NaN points untouched");
	double maxApplyUm = 0.0;
	for (size_t i = 0; i < n; i++)
	{
		const cv::Vec3f& p = points.at<cv::Vec3f>(0, static_cast<int>(i));
		const double got[3] = { p[0], p[1], p[2] };
		maxApplyUm = std::max(maxApplyUm, DistanceUm(got, reference[i].expected));
	}
	// float32 storage limits the resolution to ~0.1 um at z ~ 1000 mm.
	success &= Check(maxApplyUm < 1.0, "Apply on a float cloud matches Python within 1 um");

	cv::Mat wrongType(1, 1, CV_64FC3);
	success &= Check(gp.Apply(wrongType) == -1, "Apply rejects a matrix that is not CV_32FC3");

	// Calibration check: the calibration stored in the model matches itself and a perturbed one does not.
	cv::FileStorage fs(modelFile, cv::FileStorage::READ);
	cv::Mat camK, camKc, projK, projKc, R, T;
	fs["calib_cam_K"] >> camK;
	fs["calib_cam_kc"] >> camKc;
	fs["calib_proj_K"] >> projK;
	fs["calib_proj_kc"] >> projKc;
	fs["calib_R"] >> R;
	fs["calib_T"] >> T;
	fs.release();
	success &= Check(gp.MatchesCalibration(camK, camKc, projK, projKc, R, T, error), "training calibration is accepted");
	cv::Mat otherT = T.clone();
	otherT.at<double>(0) += 0.01;
	success &= Check(!gp.MatchesCalibration(camK, camKc, projK, projKc, R, otherT, error), "a different calibration is rejected");

	// ASCII file: extra columns (RGB) are kept.
	const std::string input = "gp_test_input.xyz";
	const std::string output = "gp_test_input_gp.xyz";
	{
		std::ofstream file(input);
		file.precision(12);
		file << reference[0].in[0] << ' ' << reference[0].in[1] << ' ' << reference[0].in[2] << " 10 20 30\n";
	}
	success &= Check(gp.ApplyXyzFile(input, output, error) == 1, "ApplyXyzFile corrects the file");
	{
		std::ifstream file(output);
		double got[3];
		int r = 0, g = 0, b = 0;
		file >> got[0] >> got[1] >> got[2] >> r >> g >> b;
		success &= Check(DistanceUm(got, reference[0].expected) < 0.01, "ApplyXyzFile matches Python");
		success &= Check(r == 10 && g == 20 && b == 30, "ApplyXyzFile keeps the extra columns");
	}
	std::remove(input.c_str());
	std::remove(output.c_str());

	if (success) std::cout << "All GP compensation tests passed\n";
	return success ? 0 : 1;
}
