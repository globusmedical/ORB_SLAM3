/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
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

#include <iostream>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <unordered_map>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "System.h"

using namespace std;

void LoadImages(const string &strPathLeft, const string &strPathRight, const string &strPathTimes,
                vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimeStamps);

void LoadDRBPoses(const string &strPosesPath, std::unordered_map<long long int, Eigen::Matrix4d> &drbPoses);
void writePosesToFile(const string& strPosesPath, const vector<std::pair<long long int, Sophus::SE3f>>& vTcw);

int main(int argc, char **argv)
{
    if(argc < 4)
    {
        cerr << endl << "Usage: ./stereo_euroc_prerect path_to_vocabulary path_to_settings path_to_sequence_folder_1 path_to_times_file_1 (path_to_image_folder_2 path_to_times_file_2 ... path_to_image_folder_N path_to_times_file_N) (trajectory_file_name)" << endl;

        return 1;
    }
    std::cout << "Sleeping for 5 secs" << endl << std::flush;
    std::this_thread::sleep_for(std::chrono::seconds(5));
    
    bool wait = true;
    if (!memcmp(argv[1], "--wait=", 7)) {
      --argc;
      wait = argv++[1][7] != '0';
    }

    const int num_seq = (argc-3)/2;
    cout << "num_seq = " << num_seq << endl;
    bool bFileName = false;
    string file_name;
    if (bFileName)
    {
        file_name = string(argv[argc-1]);
        cout << "file name: " << file_name << endl;
    }

    // Load all sequences:
    int seq;
    vector< vector<string> > vstrImageLeft;
    vector< vector<string> > vstrImageRight;
    vector< vector<double> > vTimestampsCam;
    vector<int> nImages;
    unordered_map<long long int, Eigen::Matrix4d> drbPoses;

    vstrImageLeft.resize(num_seq);
    vstrImageRight.resize(num_seq);
    vTimestampsCam.resize(num_seq);
    nImages.resize(num_seq);

    int tot_images = 0;
    for (seq = 0; seq<num_seq; seq++)
    {
        cout << "Loading images for sequence " << seq << "...";

        string pathSeq(argv[3*seq + 3]);

        string pathTimeStamps = pathSeq + "/timestamps.txt";
        string pathCam0 = pathSeq + "/mav0/cam0/data";
        string pathCam1 = pathSeq + "/mav0/cam1/data";
        string pathDrbPoses = pathSeq + "/poses.txt";

        LoadImages(pathCam0, pathCam1, pathTimeStamps, vstrImageLeft[seq], vstrImageRight[seq], vTimestampsCam[seq]);
        LoadDRBPoses(pathDrbPoses, drbPoses);
        
        cout << "LOADED!" << endl;

        nImages[seq] = vstrImageLeft[seq].size();
        tot_images += nImages[seq];
    }

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(tot_images);

    cout << endl << "-------" << endl;
    cout.precision(17);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::STEREO, true);

    cv::Mat imLeft, imRight;
    // variable to store vectore of 
    vector <std::pair<long long int, Sophus::SE3f>> vTcw;
    vTcw.reserve(5000);
    vector <std::pair<long long int, Sophus::SE3f>> vTcd;
    vTcd.reserve(5000);
    for (seq = 0; seq<num_seq; seq++)
    {
        
        // Seq loop

        double t_rect = 0;
        double t_track = 0;
        int num_rect = 0;
        int proccIm = 0;
        for(int ni=0; ni<nImages[seq]; ni++, proccIm++)
        {
	    std::cout << 100 * ni / nImages[seq] << "%\r" << std::flush;
            // Read left and right images from file
            imLeft = cv::imread(vstrImageLeft[seq][ni],cv::IMREAD_UNCHANGED);
            imRight = cv::imread(vstrImageRight[seq][ni],cv::IMREAD_UNCHANGED);

            if(imLeft.empty())
            {
                cerr << endl << "Failed to load image at: "
                     << string(vstrImageLeft[seq][ni]) << endl;
                return 1;
            }

            if(imRight.empty())
            {
                cerr << endl << "Failed to load image at: "
                     << string(vstrImageRight[seq][ni]) << endl;
                return 1;
            }

            double tframe = vTimestampsCam[seq][ni];
    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
    #endif
            long long int tstamp = static_cast<long long int>(round(tframe * 1e6));
            cout << "tstamp::" << tstamp << endl;
            Eigen::Matrix4d T_c_drb = Eigen::Matrix4d::Zero();
            if (drbPoses.find(tstamp) != drbPoses.end())
            {
                T_c_drb = drbPoses[tstamp];
                vTcd.push_back(std::make_pair(tstamp, Sophus::SE3f(T_c_drb.cast<float>())));
            }
                
            // Pass the images to the SLAM system
            auto Twc = SLAM.TrackStereo(imLeft,imRight,tframe, vector<ORB_SLAM3::IMU::Point>(), vstrImageLeft[seq][ni], /*onlyInitWithDrb*/true, T_c_drb);
            if (Twc && Twc->translation().norm() > 0)
                vTcw.push_back(std::make_pair(tstamp, *Twc));

    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
    #endif

#ifdef REGISTER_TIMES
            t_track = t_rect + std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t2 - t1).count();
            SLAM.InsertTrackTime(t_track);
#endif

            double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

            vTimesTrack[ni]=ttrack;

            // Wait to load the next frame
            double T=0;
            if(ni<nImages[seq]-1)
                T = vTimestampsCam[seq][ni+1]-tframe;
            else if(ni>0)
                T = tframe-vTimestampsCam[seq][ni-1];

            if (wait && ttrack < T) {
                long usec = static_cast<long>((T - ttrack) * 1e6);
                std::this_thread::sleep_for(std::chrono::microseconds(usec));
            }
        }

        if(seq < num_seq - 1)
        {
            cout << "Changing the dataset" << endl;

            SLAM.ChangeDataset();
        }

    }
    
    // write a function to write vTcw to a file
    writePosesToFile("slamtrajectory.txt", vTcw);
    writePosesToFile("drbtrajectory.txt", vTcd);
    
    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    if (bFileName)
    {
        const string kf_file =  "kf_" + string(argv[argc-1]) + ".txt";
        const string f_file =  "f_" + string(argv[argc-1]) + ".txt";
        SLAM.SaveTrajectoryEuRoC(f_file);
        SLAM.SaveKeyFrameTrajectoryEuRoC(kf_file);
    }
    else
    {
        SLAM.SaveTrajectoryEuRoC("CameraTrajectory.txt");
        SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
    }

    return 0;
}

void LoadImages(const string &strPathLeft, const string &strPathRight, const string &strPathTimes,
                vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimeStamps)
{
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);
    vstrImageLeft.reserve(5000);
    vstrImageRight.reserve(5000);
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            vstrImageLeft.push_back(strPathLeft + "/" + ss.str() + ".png");
            vstrImageRight.push_back(strPathRight + "/" + ss.str() + ".png");
            double t;
            ss >> t;
            vTimeStamps.push_back(t/1e9);

        }
    }
}


void LoadDRBPoses(const string &strPosesPath, std::unordered_map<long long int, Eigen::Matrix4d> &drbPoses)
{
    ifstream fDrbPoses;
    fDrbPoses.open(strPosesPath.c_str());

    // Transformation from GNS to OpenCV coordinate system
    Eigen::Matrix4d T_cv_gns = Eigen::Matrix4d::Zero();
    T_cv_gns(0, 0) = -1;
    T_cv_gns(1, 2) = 1;
    T_cv_gns(2, 1) = 1;
    T_cv_gns(3, 3) = 1;

    // Transformation from Wide to Tall image coordinate system (Unrolled/Rolled)
    Eigen::Matrix4d T_tallIm_wideIm = Eigen::Matrix4d::Zero();
    T_tallIm_wideIm(0, 1) = 1;
    T_tallIm_wideIm(1, 0) = -1;
    T_tallIm_wideIm(2, 2) = 1;
    T_tallIm_wideIm(3, 3) = 1;
    
    // Transformation from Unrectified to Rectified coordinate system
    Eigen::Matrix4d T_rect_unrect = Eigen::Matrix4d::Identity();

    // HS P13 Values
   /* T_rect_unrect(0, 0) = 0.994558853084219;
    T_rect_unrect(0, 1) = 0.008337477748085;
    T_rect_unrect(0, 2) = 0.103842063811366;
    T_rect_unrect(1, 0) = -0.007013491912108;
    T_rect_unrect(1, 1) = 0.999889481440259;
    T_rect_unrect(1, 2) = -0.013108616873251;
    T_rect_unrect(2, 0) = -0.103939880137522;
    T_rect_unrect(2, 1) = 0.012308995488303;
    T_rect_unrect(2, 2) = 0.994507410704951;*/

    // HS P1.2 Values
    T_rect_unrect(0, 0) = 0.994863943339954;
    T_rect_unrect(0, 1) = -0.002161664462900;
    T_rect_unrect(0, 2) = 0.101198129670592;
    T_rect_unrect(1, 0) = 0.001479505864314;
    T_rect_unrect(1, 1) = 0.999975680456105;
    T_rect_unrect(1, 2) = 0.006815391312918;
    T_rect_unrect(2, 0) = -0.101210401167437;
    T_rect_unrect(2, 1) = -0.006630663850669;
    T_rect_unrect(2, 2) = 0.994842946897864;

    while (!fDrbPoses.eof()) {
        string s;
        getline(fDrbPoses, s);
        if (s[0] == '/' || s[14]=='0')
            continue;

        if (!s.empty()) {
            string item;
            size_t pos = 0;
            double data[11];
            int count = 0;
            while ((pos = s.find(',')) != string::npos) {
                item = s.substr(0, pos);
                s.erase(0, pos + 1);
                if (item[1] == 'e')
                  continue;
                data[count++] = stod(item);
            }
            item = s.substr(0, pos);
            data[10] = stod(item);

            if (count >= 10) 
            {
                Eigen::Vector3d t(data[4], data[5], data[6]);
                Eigen::Quaterniond Q(/*w,x,y,z*/data[7], data[8], data[9], data[10]);
                auto R = Q.normalized().toRotationMatrix();
                Eigen::Matrix4d T_cGNS_drb;
                // Set to Identity to make bottom row of Matrix [0,0,0,1]
                T_cGNS_drb.setIdentity();
                T_cGNS_drb.block<3, 3>(0, 0) = R;
                T_cGNS_drb.block<3, 1>(0, 3) = t / 1e3;
                
                Eigen::Matrix4d T_c_drb = T_rect_unrect * T_tallIm_wideIm * T_cv_gns * T_cGNS_drb;
                                
                long long int tstamp = static_cast<long long int>(round(data[0]));
                drbPoses.insert(make_pair(tstamp, T_c_drb));
            }
            
        }
    }
}


// write a function to write vTcw to a file
void writePosesToFile(const string& strPosesPath, const vector<std::pair<long long int, Sophus::SE3f>>& poses)
{
    ofstream fPoses;
    fPoses.open(strPosesPath.c_str());

    fPoses << "# timestamp tx ty tz qx qy qz qw"<<endl;
    for (size_t i = 0; i < poses.size(); i++)
    {
        Sophus::SE3f Twc = poses[i].second;
        auto R = Twc.rotationMatrix();
        auto t = Twc.translation();
        Eigen::Quaternionf Q(R);

        fPoses << fixed << setprecision(6) << poses[i].first << ", " << t(0) << ", " << t(1) << ", " << t(2) << ", " << Q.x() << ", " << Q.y() << ", " << Q.z() << ", " << Q.w() << endl;
    }
    fPoses.close();
}