/*
Copyright (c) 2012, Daniel Moreno and Gabriel Taubin
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

// Single source of the configuration keys and their default values (QSettings).
// The application stores the settings in user scope using the platform native
// format; on Windows: HKEY_CURRENT_USER\Software\<Organization>\<App_Name>.
namespace smcp::Settings
{
namespace App
{
	const auto Organization = "CENAM";
	const auto App_Name = "SMCP";
	const auto Window_Name = "SMCP: Camera-Projector Measuring System";

	const auto Root_Directory = "App/RootDirectory";
}

namespace MainWindow
{
	const auto Geometry = "Window/Geometry";
	const auto State = "Window/State";
}

namespace Camera
{
	const auto Serial_Number = "Camera/SerialNumber";

	const auto Black_Level = "Camera/BlackLevel";
	constexpr double Black_Level_Default_Value = 10;
	const auto Exposure_Time = "Camera/ExposureTime";
	constexpr double Exposure_Time_Default_Value = 4162.55; // 8325.1 - 4162.55;
	const auto Gain = "Camera/Gain";
	constexpr double Gain_Default_Value = 1;
	const auto Gamma = "Camera/Gamma";
	constexpr double Gamma_Default_Value = 1.0;
}

namespace Projector
{
	const auto Screen = "Projector/Screen";
	constexpr int Screen_Default_Value = 0;

	const auto Pattern_Count = "GrayCode/PatternCount";
	constexpr int Pattern_Count_Default_Value = 10;
}

namespace Capture
{
	const auto Wait_Time = "Capture/WaitTime";
	constexpr int Wait_Time_Default_Value = 500;

	const auto Continuous = "Capture/Continuous";
	constexpr int Continuous_Default_Value = 100;
}

namespace Chessboard
{
	// Interior corners (columns x rows) and the size of each square (mm).
	const auto Columns = "Chessboard/Columns";
	constexpr int Columns_Default_Value = 22;

	const auto Rows = "Chessboard/Rows";
	constexpr int Rows_Default_Value = 15;

	const auto Width = "Chessboard/Width";
	constexpr double Width_Default_Value = 15;

	const auto Height = "Chessboard/Height";
	constexpr double Height_Default_Value = 15;
}

namespace Decode
{
	const auto Threshold = "Decode/Threshold";
	constexpr int Threshold_Default_Value = 20;

	// Robust decoding (Moreno & Taubin): b is a ratio in [0, 1] and m a number of levels.
	const auto B = "Decode/B";
	constexpr double B_Default_Value = 0.5;

	const auto M = "Decode/M";
	constexpr int M_Default_Value = 5;
}

namespace Calibration
{
	const auto H_Win = "Calibration/HWin";
	constexpr int H_Win_Default_Value = 60;

	const auto Shadow_Threshold = "Calibration/ShadowThreshold";
	constexpr int Shadow_Threshold_Default_Value = 0;

	const auto File = "Calibration/File";
}

namespace Reconstruction
{
	const auto Max_Dist = "Reconstruction/MaxDist";
	constexpr double Max_Dist_Default_Value = 100.0;

	const auto Save_Normals = "Reconstruction/SaveNormals";
	constexpr bool Save_Normals_Default_Value = true;

	const auto Save_Colors = "Reconstruction/SaveColors";
	constexpr bool Save_Colors_Default_Value = true;

	const auto Save_Binary = "Reconstruction/SaveBinary";
	constexpr bool Save_Binary_Default_Value = true;
}

namespace Compensation
{
	// GP compensation model (modelo_gp_smcp.yml) and whether it is applied after triangulation.
	const auto Gp_File = "Compensation/GpFile";
	const auto Gp_Enabled = "Compensation/GpEnabled";
	constexpr bool Gp_Enabled_Default_Value = false;
}
} // namespace smcp::Settings
