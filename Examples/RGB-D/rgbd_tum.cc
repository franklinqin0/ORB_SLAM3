/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<sstream>
#include<vector>
#include<string>
#include<unistd.h>

#include<opencv2/core/core.hpp>

#include<System.h>

using namespace std;

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    if(argc < 5)
    {
        cerr << endl
             << "Usage: ./rgbd_tum path_to_vocabulary path_to_settings path_to_sequence path_to_association [options]" << endl
             << "Options:" << endl
             << "  --traj <file>            Output camera trajectory file (TUM format). Default: CameraTrajectory.txt" << endl
             << "  --keyframe-traj <file>   Output keyframe trajectory file (TUM format). Default: KeyFrameTrajectory.txt" << endl
             << "  --map <file>         Output sparse map as PLY. Default: integrated_mesh_init.ply" << endl
             << "  --intrinsics <file>      Read 3x3 K matrix (row-major) to override fx, fy, cx, cy" << endl
             << "  -h, --help               Show this help and exit" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageFilenamesRGB;
    vector<string> vstrImageFilenamesD;
    vector<double> vTimestamps;
    string strAssociationFilename = string(argv[4]);
    LoadImages(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);

    // Optional arguments with defaults
    string outTrajFile = "CameraTrajectory.txt";
    string outKFFile = "KeyFrameTrajectory.txt";
    string outMapPLY = "integrated_mesh_init.ply";
    string intrinsicsFile = "";

    // Parse optional flags starting from argv[5]
    for(int i = 5; i < argc; ++i)
    {
        string a = argv[i];
        if(a == "--traj")
        {
            if(i + 1 < argc) { outTrajFile = argv[++i]; }
            else { cerr << "Missing value for --traj" << endl; return 1; }
        }
        else if(a == "--keyframe-traj")
        {
            if(i + 1 < argc) { outKFFile = argv[++i]; }
            else { cerr << "Missing value for --keyframe-traj" << endl; return 1; }
        }
        else if(a == "--map")
        {
            if(i + 1 < argc) { outMapPLY = argv[++i]; }
            else { cerr << "Missing value for --map" << endl; return 1; }
        }
        else if(a == "--intrinsics")
        {
            if(i + 1 < argc) { intrinsicsFile = argv[++i]; }
            else { cerr << "Missing value for --intrinsics" << endl; return 1; }
        }
        else if(a == "-h" || a == "--help")
        {
            cerr << endl
                 << "Usage: ./rgbd_tum path_to_vocabulary path_to_settings path_to_sequence path_to_association [options]" << endl
                 << "Options:" << endl
                 << "  --traj <file>            Output camera trajectory file (TUM format). Default: CameraTrajectory.txt" << endl
                 << "  --keyframe-traj <file>   Output keyframe trajectory file (TUM format). Default: KeyFrameTrajectory.txt" << endl
                 << "  --map <file>         Output sparse map as PLY. Default: integrated_mesh_init.ply" << endl
                 << "  --intrinsics <file>      Read 3x3 K matrix (row-major) to override fx, fy, cx, cy" << endl
                 << "  -h, --help               Show this help and exit" << endl;
            return 0;
        }
        else
        {
            cerr << "Unknown option: " << a << endl;
            return 1;
        }
    }

    // Check consistency in the number of images and depthmaps
    int nImages = vstrImageFilenamesRGB.size();
    if(vstrImageFilenamesRGB.empty())
    {
        cerr << endl << "No images found in provided path." << endl;
        return 1;
    }
    else if(vstrImageFilenamesD.size()!=vstrImageFilenamesRGB.size())
    {
        cerr << endl << "Different number of images for rgb and depth." << endl;
        return 1;
    }

    // Optionally override intrinsics by patching a temporary settings file
    string settingsPath = string(argv[2]);
    string patchedSettingsPath;
    if(!intrinsicsFile.empty())
    {
        // Read 3x3 intrinsics matrix (row-major) from text file
        ifstream fin(intrinsicsFile.c_str());
        if(!fin.is_open())
        {
            cerr << "Failed to open intrinsics file: " << intrinsicsFile << endl;
            return 1;
        }
        vector<double> k(9, 0.0);
        for(int i = 0; i < 9; ++i)
        {
            if(!(fin >> k[i]))
            {
                cerr << "Failed to read 3x3 K matrix (need 9 numbers) from: " << intrinsicsFile << endl;
                return 1;
            }
        }
        fin.close();

        double fx = k[0];
        double fy = k[4];
        double cx = k[2];
        double cy = k[5];

        // Load original YAML
        ifstream fyaml(settingsPath.c_str());
        if(!fyaml.is_open())
        {
            cerr << "Failed to open settings file: " << settingsPath << endl;
            return 1;
        }
        vector<string> lines;
        string line;
        while(std::getline(fyaml, line)) lines.push_back(line);
        fyaml.close();

        auto replace_key_value = [](string &ln, const string &key, const string &value)->bool
        {
            // Trim leading spaces for key detection
            size_t i = 0;
            while(i < ln.size() && (ln[i] == ' ' || ln[i] == '\t')) ++i;
            const string keyColon = key + ":";
            if(ln.size() >= i + keyColon.size() && ln.compare(i, keyColon.size(), keyColon) == 0)
            {
                ln = ln.substr(0, i) + keyColon + " " + value;
                return true;
            }
            return false;
        };

        bool r_fx1=false, r_fy1=false, r_cx1=false, r_cy1=false;
        bool r_fx=false,  r_fy=false,  r_cx=false,  r_cy=false;

        ostringstream sfx, sfy, scx, scy;
        sfx.setf(std::ios::fixed); sfy.setf(std::ios::fixed); scx.setf(std::ios::fixed); scy.setf(std::ios::fixed);
        sfx.precision(6); sfy.precision(6); scx.precision(6); scy.precision(6);
        sfx << fx; sfy << fy; scx << cx; scy << cy;

        for(string &ln : lines)
        {
            if(!r_fx1) r_fx1 = replace_key_value(ln, "Camera1.fx", sfx.str());
            if(!r_fy1) r_fy1 = replace_key_value(ln, "Camera1.fy", sfy.str());
            if(!r_cx1) r_cx1 = replace_key_value(ln, "Camera1.cx", scx.str());
            if(!r_cy1) r_cy1 = replace_key_value(ln, "Camera1.cy", scy.str());

            if(!r_fx)  r_fx  = replace_key_value(ln, "Camera.fx", sfx.str());
            if(!r_fy)  r_fy  = replace_key_value(ln, "Camera.fy", sfy.str());
            if(!r_cx)  r_cx  = replace_key_value(ln, "Camera.cx", scx.str());
            if(!r_cy)  r_cy  = replace_key_value(ln, "Camera.cy", scy.str());
        }

        // If none of the keys existed, append Camera1.* at end
        if(!(r_fx1||r_fx))   lines.push_back(string("Camera1.fx: ") + sfx.str());
        if(!(r_fy1||r_fy))   lines.push_back(string("Camera1.fy: ") + sfy.str());
        if(!(r_cx1||r_cx))   lines.push_back(string("Camera1.cx: ") + scx.str());
        if(!(r_cy1||r_cy))   lines.push_back(string("Camera1.cy: ") + scy.str());

        // Write patched YAML to temporary path
        ostringstream tmpname;
        tmpname << "/tmp/orbslam3_settings_" << getpid() << ".yaml";
        patchedSettingsPath = tmpname.str();

        ofstream fout(patchedSettingsPath.c_str());
        if(!fout.is_open())
        {
            cerr << "Failed to write patched settings file: " << patchedSettingsPath << endl;
            return 1;
        }
        for(const string &l : lines) fout << l << '\n';
        fout.close();

        settingsPath = patchedSettingsPath;
        cout << "[INFO] Using intrinsics from '" << intrinsicsFile << "' -> fx=" << fx << ", fy=" << fy << ", cx=" << cx << ", cy=" << cy << endl;
        cout << "[INFO] Patched settings written to: " << settingsPath << endl;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1], settingsPath.c_str(), ORB_SLAM3::System::RGBD, true);
    float imageScale = SLAM.GetImageScale();

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat imRGB, imD;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image and depthmap from file
        imRGB = cv::imread(string(argv[3])+"/"+vstrImageFilenamesRGB[ni],cv::IMREAD_UNCHANGED); //,cv::IMREAD_UNCHANGED);
        imD = cv::imread(string(argv[3])+"/"+vstrImageFilenamesD[ni],cv::IMREAD_UNCHANGED); //,cv::IMREAD_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(imRGB.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(argv[3]) << "/" << vstrImageFilenamesRGB[ni] << endl;
            return 1;
        }

        if(imageScale != 1.f)
        {
            int width = imRGB.cols * imageScale;
            int height = imRGB.rows * imageScale;
            cv::resize(imRGB, imRGB, cv::Size(width, height));
            cv::resize(imD, imD, cv::Size(width, height));
        }

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        // Pass the image to the SLAM system
        SLAM.TrackRGBD(imRGB,imD,tframe);

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveTrajectoryTUM(outTrajFile);
    SLAM.SaveKeyFrameTrajectoryTUM(outKFFile);   

    // Save the sparse map as PLY
    SLAM.SaveMapToPLY(outMapPLY);

    return 0;
}

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps)
{
    ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());
    while(!fAssociation.eof())
    {
        string s;
        getline(fAssociation,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB, sD;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenamesRGB.push_back(sRGB);
            ss >> t;
            ss >> sD;
            vstrImageFilenamesD.push_back(sD);

        }
    }
}
