#include "core/GPCompensation.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

/* Internal Helpers ==================================================================== */
namespace
{
	const char* const CALIB_KEYS[6] = { "cam_K", "cam_kc", "proj_K", "proj_kc", "R", "T" };

	/// <summary>
	/// Reads a matrix node as CV_64F; returns false if missing or with the wrong number of elements.
	/// </summary>
	bool ReadMatrix(const cv::FileStorage& fs, const std::string& key, cv::Mat& out, int expectedTotal, std::string& error)
	{
		cv::Mat m;
		fs[key] >> m;
		if (m.empty() || (expectedTotal > 0 && static_cast<int>(m.total()) != expectedTotal))
		{
			error = "missing or invalid '" + key + "'";
			return false;
		}
		m.convertTo(out, CV_64F);
		return true;
	}

	/// <summary>
	/// Wraps a functor as a cv::ParallelLoopBody (OpenCV 2.4 parallel_for_ does not accept lambdas).
	/// </summary>
	template <typename F>
	class FunctorBody : public cv::ParallelLoopBody
	{
	public:
		explicit FunctorBody(const F& f) : f_(f) {}
		void operator()(const cv::Range& range) const override { f_(range); }

	private:
		F f_;
	};

	template <typename F>
	void ParallelForRange(int n, const F& f)
	{
		cv::parallel_for_(cv::Range(0, n), FunctorBody<F>(f));
	}
}

namespace smcp
{
bool GPCompensation::Load(const std::string& filename, std::string& error)
{
	Clear();

	cv::FileStorage fs(filename, cv::FileStorage::READ);
	if (!fs.isOpened())
	{
		error = "cannot open " + filename;
		return false;
	}

	cv::Mat a, l, boxMin, boxMax;
	if (!ReadMatrix(fs, "a", a, 6, error) || !ReadMatrix(fs, "l", l, 3, error)
		|| !ReadMatrix(fs, "caja_min", boxMin, 3, error) || !ReadMatrix(fs, "caja_max", boxMax, 3, error))
	{
		return false;
	}
	for (int i = 0; i < 6; i++)
	{
		_a[i] = a.at<double>(i);
	}
	_l = cv::Vec3d(l.at<double>(0), l.at<double>(1), l.at<double>(2));
	_boxMin = cv::Vec3d(boxMin.at<double>(0), boxMin.at<double>(1), boxMin.at<double>(2));
	_boxMax = cv::Vec3d(boxMax.at<double>(0), boxMax.at<double>(1), boxMax.at<double>(2));

	_muZ = static_cast<double>(fs["mu_z"]);
	_sdZ = static_cast<double>(fs["sd_z"]);
	const int passes = static_cast<int>(fs["pasadas"]);
	if (_sdZ <= 0.0 || passes < 1 || _l[0] <= 0.0 || _l[1] <= 0.0 || _l[2] <= 0.0)
	{
		error = "invalid mu_z/sd_z/pasadas/l";
		return false;
	}

	for (int k = 1; k <= passes; k++)
	{
		const std::string s = std::to_string(k);
		cv::Mat p0, B, P, W;
		if (!ReadMatrix(fs, "p0_" + s, p0, 3, error) || !ReadMatrix(fs, "B_" + s, B, 9, error)
			|| !ReadMatrix(fs, "P_rep_" + s, P, 0, error) || !ReadMatrix(fs, "W_" + s, W, 0, error))
		{
			return false;
		}
		if (P.cols != 3 || W.cols != 3 || P.rows != W.rows)
		{
			error = "P_rep_" + s + " / W_" + s + " must be Nx3 with the same N";
			return false;
		}

		Pass pass;
		pass.p0 = cv::Vec3d(p0.at<double>(0), p0.at<double>(1), p0.at<double>(2));
		for (int i = 0; i < 9; i++)
		{
			pass.B.val[i] = B.at<double>(i);   // row-major, same as numpy
		}
		pass.supportScaled.resize(P.rows);
		pass.weights.resize(P.rows);
		for (int j = 0; j < P.rows; j++)
		{
			pass.supportScaled[j] = cv::Vec3d(P.at<double>(j, 0) / _l[0], P.at<double>(j, 1) / _l[1], P.at<double>(j, 2) / _l[2]);
			pass.weights[j] = cv::Vec3d(W.at<double>(j, 0), W.at<double>(j, 1), W.at<double>(j, 2));
		}
		_passes.push_back(pass);
	}

	for (int i = 0; i < 6; i++)
	{
		if (!ReadMatrix(fs, std::string("calib_") + CALIB_KEYS[i], _calib[i], 0, error))
		{
			return false;
		}
	}
	fs["calibracion_archivo"] >> _calibrationFilename;
	fs.release();

	_filename = filename;
	_loaded = true;
	std::cout << "[GPCompensation] Loaded " << filename << ": " << _passes.size() << " pass(es), "
		<< _passes[0].supportScaled.size() << " support points, trained with " << _calibrationFilename << std::endl;
	return true;
}

void GPCompensation::Clear()
{
	_loaded = false;
	_filename.clear();
	_calibrationFilename.clear();
	_passes.clear();
	for (auto& m : _calib)
	{
		m.release();
	}
}

bool GPCompensation::MatchesCalibration(const cv::Mat& camK, const cv::Mat& camKc, const cv::Mat& projK,
	const cv::Mat& projKc, const cv::Mat& R, const cv::Mat& T, std::string& error) const
{
	if (!_loaded)
	{
		error = "model not loaded";
		return false;
	}
	const cv::Mat* current[6] = { &camK, &camKc, &projK, &projKc, &R, &T };
	for (int i = 0; i < 6; i++)
	{
		cv::Mat c;
		if (current[i]->empty() || current[i]->total() != _calib[i].total())
		{
			error = std::string("calibration '") + CALIB_KEYS[i] + "' has a different size";
			return false;
		}
		current[i]->convertTo(c, CV_64F);
		c = c.reshape(1, _calib[i].rows);
		if (cv::norm(c, _calib[i]) > 1e-9 * std::max(1.0, cv::norm(_calib[i])))
		{
			error = std::string("calibration '") + CALIB_KEYS[i] + "' differs from the one used to train the model ("
				+ _calibrationFilename + ")";
			return false;
		}
	}
	return true;
}

void GPCompensation::CorrectPoint(const double in[3], double out[3]) const
{
	// physical block: p_f = p (1 + s(z, rho))
	const double zn = (in[2] - _muZ) / _sdZ;
	const double rho2 = (in[0] * in[0] + in[1] * in[1]) / (in[2] * in[2]);
	const double s = _a[0] + _a[1] * zn + _a[2] * zn * zn + _a[3] * rho2 + _a[4] * rho2 * rho2 + _a[5] * zn * rho2;
	cv::Vec3d x(in[0] * (1.0 + s), in[1] * (1.0 + s), in[2] * (1.0 + s));

	// GP residual field, one term per pass: e = B (x - p0) + sum_j k52(x, P_j) W_j
	static const double S5 = std::sqrt(5.0);
	for (const Pass& pass : _passes)
	{
		cv::Vec3d e = pass.B * (x - pass.p0);
		const cv::Vec3d xs(x[0] / _l[0], x[1] / _l[1], x[2] / _l[2]);
		const size_t n = pass.supportScaled.size();
		for (size_t j = 0; j < n; j++)
		{
			const cv::Vec3d& ps = pass.supportScaled[j];
			const double dx = xs[0] - ps[0], dy = xs[1] - ps[1], dz = xs[2] - ps[2];
			const double t = S5 * std::sqrt(dx * dx + dy * dy + dz * dz);   // sqrt(5) r
			const double k = (1.0 + t + t * t / 3.0) * std::exp(-t);
			e += k * pass.weights[j];
		}
		x -= e;
	}
	out[0] = x[0];
	out[1] = x[1];
	out[2] = x[2];
}

bool GPCompensation::InsideTrainingVolume(const double p[3], double marginMm) const
{
	for (int i = 0; i < 3; i++)
	{
		if (p[i] < _boxMin[i] - marginMm || p[i] > _boxMax[i] + marginMm)
		{
			return false;
		}
	}
	return true;
}

int GPCompensation::Apply(cv::Mat& points, int* outside) const
{
	if (!_loaded || points.empty() || points.type() != CV_32FC3)
	{
		return -1;
	}
	std::atomic<int> corrected(0), nOut(0);
	ParallelForRange(points.rows, [&](const cv::Range& range)
		{
			int n = 0, localOut = 0;
			for (int h = range.start; h < range.end; h++)
			{
				cv::Vec3f* row = points.ptr<cv::Vec3f>(h);
				for (int w = 0; w < points.cols; w++)
				{
					cv::Vec3f& p = row[w];
					if (std::isnan(p[0]) || std::isnan(p[1]) || std::isnan(p[2]))
					{
						continue;
					}
					const double in[3] = { p[0], p[1], p[2] };
					double out[3];
					CorrectPoint(in, out);
					if (!InsideTrainingVolume(in))
					{
						localOut++;
					}
					p[0] = static_cast<float>(out[0]);
					p[1] = static_cast<float>(out[1]);
					p[2] = static_cast<float>(out[2]);
					n++;
				}
			}
			corrected += n;
			nOut += localOut;
		});
	if (outside)
	{
		*outside = nOut;
	}
	return corrected;
}

int GPCompensation::ApplyXyzFile(const std::string& input, const std::string& output, std::string& error,
	int* outside) const
{
	if (!_loaded)
	{
		error = "model not loaded";
		return -1;
	}
	std::ifstream in(input);
	if (!in.is_open())
	{
		error = "cannot open " + input;
		return -1;
	}

	// read all lines first, then correct in parallel
	std::vector<cv::Vec3d> pts;
	std::vector<std::string> extra;
	std::string line;
	while (std::getline(in, line))
	{
		std::istringstream ss(line);
		cv::Vec3d p;
		if (!(ss >> p[0] >> p[1] >> p[2]))
		{
			continue;   // empty or header line
		}
		std::string rest;
		std::getline(ss, rest);
		pts.push_back(p);
		extra.push_back(rest);
	}
	in.close();

	std::vector<cv::Vec3d> corrected(pts.size());
	std::atomic<int> nOut(0);
	ParallelForRange(static_cast<int>(pts.size()), [&](const cv::Range& r)
		{
			int localOut = 0;
			for (int i = r.start; i < r.end; i++)
			{
				CorrectPoint(pts[i].val, corrected[i].val);
				if (!InsideTrainingVolume(pts[i].val))
				{
					localOut++;
				}
			}
			nOut += localOut;
		});

	std::ofstream out(output, std::ios::out | std::ios::trunc);
	if (!out.is_open())
	{
		error = "cannot write " + output;
		return -1;
	}
	out << std::setprecision(12);
	for (size_t i = 0; i < pts.size(); i++)
	{
		out << corrected[i][0] << ' ' << corrected[i][1] << ' ' << corrected[i][2] << extra[i] << '\n';
	}
	out.close();

	if (outside)
	{
		*outside = nOut;
	}
	return static_cast<int>(pts.size());
}
} // namespace smcp
