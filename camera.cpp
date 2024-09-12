#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <string>
#include <fstream>
#include <sstream>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <dirent.h>
#include <chrono>
#include <iomanip>
#include <thread>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <boost/filesystem.hpp>
#include <gpiod.h>
#include <yaml-cpp/yaml.h>

using namespace std;
using namespace cv;
using namespace std::chrono;

ofstream csvFile; // File stream for CSV
ofstream csvFile2;
ofstream sys_log;
int h_idx = 0;
bool L_cam = false, C_cam = false, R_cam = false; // Default to all cameras on
int cam_type = 1; // Default camera mode
bool trig_mod = false;
bool visualization = false;
bool distortion = false;
bool first_frame_saved = false;

std::string hdd_out_path, hdd_log_path;

void writeImagePropertiesToYAML(const string& filename, int width, int height, const string& format, int quality, int HFoV, int VFoV);

// initial the csv file
void initCSV_hdd(const string& filename) {
    csvFile.open(filename, ios::out | ios::app);
    csvFile << "Timestamp,Filename\n"; // Write the header if necessary
}

// initial the csv file
void initCSV_ram(const string& filename) {
    csvFile2.open(filename, ios::out | ios::app);
    csvFile2 << "Timestamp,Filename\n"; // Write the header if necessary
}

// for closing csv file after done
void closeCSV() {
    csvFile.close();
}

// boolean function that check if directory exist
bool directoryExists(const string& path) {
    struct stat info;
    if (stat(path.c_str(), &info) != 0){
        return false;
    }
    else if (info.st_mode & S_IFDIR){
        return true;
    }
    else
        return false;
}

// create directory if there isnt exist
void ensureDirectoryExists(const std::string& path) {
    if (!directoryExists(path)) {
        mkdir(path.c_str(), 0777);
    }
}

struct FileEntry {
    string path;
    time_t ctime;
    FileEntry(const string& path, time_t ctime) : path(path), ctime(ctime) {}
};

bool compareFileEntry(const FileEntry& a, const FileEntry& b) {
    return a.ctime < b.ctime;
}

// keep last 300 image frame and remove others
void remove_oldest_frame(const string& output_folder) {
    DIR* dir;
    struct dirent* ent;
    struct stat file_stat;
    vector<FileEntry> files;
    if ((dir = opendir(output_folder.c_str())) != NULL) {
        while ((ent = readdir(dir)) != NULL) {
            string file_name = ent->d_name;
            string file_path = output_folder + "/" + file_name;
            if(file_name == "." || file_name == "..") continue;
            if (stat(file_path.c_str(), &file_stat) == 0) {
                if (S_ISREG(file_stat.st_mode)) {
                    files.emplace_back(file_path, file_stat.st_ctime);
                }
            }
        }
        closedir(dir);
        sort(files.begin(), files.end(), compareFileEntry);
        if (files.size() > 300) {
            remove(files.front().path.c_str());
        }
    } else {
        // sys_log << now << setw(6) << setfill('0') << "," << "Camera" << "," << "ERROR: Failed to open directory" << endl;
        // cerr << "Failed to open directory: " << output_folder << endl;
    }
}

// Function to copy files from RAM to SSD asynchronously
void copyFileAsync(const string& source, const string& destination) {
    std::thread([source, destination]() {
        std::ifstream src(source, std::ios::binary);
        std::ofstream dst(destination, std::ios::binary);
        dst << src.rdbuf();
    }).detach();
}

// to save images in the directory
void saveFrame(const Mat& left_frame, const Mat& right_frame, const Mat& center_frame, const string& outputFolderRam_left, const string& outputFolderHDD_left, const string& outputFolderRam_right, const string& outputFolderHDD_right, const string& outputFolderRam_center, const string& outputFolderHDD_center) {
    auto now = high_resolution_clock::now();
    auto now_sec = duration_cast<seconds>(now.time_since_epoch()).count();  // seconds since epoch
    auto now_microsec = duration_cast<microseconds>(now.time_since_epoch()).count() % 1000000;  // microseconds part
    // cout << "timestamp while storing: " << now_sec << setw(6) << setfill('0') << now_microsec << endl;
    stringstream filename1_left, filename2_left, filename1_right, filename2_right, filename1_center, filename2_center;
    filename1_left << outputFolderRam_left << "/" << now_sec << setw(6) << setfill('0') << now_microsec << ".jpg";
    filename2_left << outputFolderHDD_left << "/" << now_sec << setw(6) << setfill('0') << now_microsec << ".jpg";
    filename1_right << outputFolderRam_right << "/" << now_sec << setw(6) << setfill('0') << now_microsec << ".jpg";
    filename2_right << outputFolderHDD_right << "/" << now_sec << setw(6) << setfill('0') << now_microsec << ".jpg";
    filename1_center << outputFolderRam_center << "/" << now_sec << setw(6) << setfill('0') << now_microsec << ".jpg";
    filename2_center << outputFolderHDD_center << "/" << now_sec << setw(6) << setfill('0') << now_microsec << ".jpg";
    vector<int> compression_params = {IMWRITE_JPEG_QUALITY, 95};
    if (L_cam) imwrite(filename1_left.str(), left_frame, compression_params);
    if (R_cam) imwrite(filename1_right.str(), right_frame, compression_params);
    if (C_cam) imwrite(filename1_center.str(), center_frame, compression_params);

    if (L_cam) copyFileAsync(filename1_left.str(), filename2_left.str());
    if (R_cam) copyFileAsync(filename1_right.str(), filename2_right.str());
    if (C_cam) copyFileAsync(filename1_center.str(), filename2_center.str());

    remove_oldest_frame(outputFolderRam_left);
    remove_oldest_frame(outputFolderRam_right);
    remove_oldest_frame(outputFolderRam_center);
    std::stringstream timestamp;
    timestamp << now_sec << setw(6) << setfill('0') << now_microsec;

    if (L_cam) {
        csvFile << timestamp.str() << "," << filename1_left.str() << endl;
        csvFile2 << timestamp.str() << "," << filename2_left.str() << endl;
    }
    if (R_cam) {
        csvFile << timestamp.str() << "," << filename1_right.str() << endl;
        csvFile2 << timestamp.str() << "," << filename2_right.str() << endl;
    }
    if (C_cam) {
        csvFile << timestamp.str() << "," << filename1_center.str() << endl;
        csvFile2 << timestamp.str() << "," << filename2_center.str() << endl;
    }

    if (!first_frame_saved) {
        int width, height;
        if (L_cam)  width = left_frame.cols;
        if (L_cam)  height = left_frame.rows;
        if (R_cam)  width = right_frame.cols;
        if (R_cam)  height = right_frame.rows;
        if (C_cam)  width = center_frame.cols;
        if (C_cam)  height = center_frame.rows;
        string format = "jpg";
        int quality = 95;
        int HFoV = 86;
        int VFoV = 70;
        writeImagePropertiesToYAML("/media/ramdisk/image_properties.yaml", width, height, format, quality, HFoV, VFoV);
        writeImagePropertiesToYAML("/home/jetson/image_properties.yaml", width, height, format, quality, HFoV, VFoV);
        first_frame_saved = true;
    }

}

int get_last_digit_of_bus_info(const std::string& device_path) {
    if (access(device_path.c_str(), F_OK) != -1) {
        int fd = open(device_path.c_str(), O_RDWR);
        if (fd < 0) {
            perror("Opening video device");
            return -1;
        }

        struct v4l2_capability vd;
        if (ioctl(fd, VIDIOC_QUERYCAP, &vd) == -1) {
            perror("Querying capabilities");
            close(fd);
            return -1;
        }

        close(fd);

        // Convert bus_info to string
        std::string bus_info(reinterpret_cast<char*>(vd.bus_info));

        // Extract the last character and convert it to an integer
        if (!bus_info.empty()) {
            char last_char = bus_info.back();
            if (std::isdigit(last_char)) {
                return last_char - '0';
            }
        }
    }

    return -1; // Return -1 if there's an error
}

struct CameraCalibration {
    int image_width;
    int image_height;
    string camera_name;
    double exposure_time; // Added exposure time
    Mat camera_matrix;
    string distortion_model;
    Mat distortion_coefficients;
    Mat rectification_matrix;
    Mat projection_matrix;
};


CameraCalibration readCalibration(const string& filename) {
    YAML::Node config = YAML::LoadFile(filename);
    CameraCalibration calib;

    try { 
        calib.image_width = config["image_width"].as<int>();
        calib.image_height = config["image_height"].as<int>();
        calib.camera_name = config["camera_name"].as<string>();
        calib.exposure_time = config["exposure_time"].as<double>(); // This might also need to be handled carefully
        
        auto cam_matrix = config["camera_matrix"]["data"].as<vector<double>>();
        calib.camera_matrix = Mat(3, 3, CV_64F, cam_matrix.data()).clone();

        calib.distortion_model = config["distortion_model"].as<string>();

        auto dist_coeffs = config["distortion_coefficients"]["data"].as<vector<double>>();
        calib.distortion_coefficients = Mat(1, dist_coeffs.size(), CV_64F, dist_coeffs.data()).clone();

        auto rect_matrix = config["rectification_matrix"]["data"].as<vector<double>>();
        calib.rectification_matrix = Mat(3, 3, CV_64F, rect_matrix.data()).clone();

        auto proj_matrix = config["projection_matrix"]["data"].as<vector<double>>();
        calib.projection_matrix = Mat(3, 4, CV_64F, proj_matrix.data()).clone();
    } catch (const YAML::TypedBadConversion<int>& e) {
        cerr << "Error: Failed to convert a YAML value to an integer in " << filename << ": " << e.what() << endl;
        throw; // rethrow the exception after logging
    } catch (const YAML::TypedBadConversion<double>& e) {
        cerr << "Error: Failed to convert a YAML value to a double in " << filename << ": " << e.what() << endl;
        throw;
    }

    return calib;
}



bool setExposureTime(const std::string& device_path, double exposure_time) {
    int fd = open(device_path.c_str(), O_RDWR);
    if (fd < 0) {
        perror("Opening video device");
        return false;
    }

    struct v4l2_control control;
    control.id = V4L2_CID_EXPOSURE_ABSOLUTE;
    control.value = static_cast<int>(exposure_time);

    if (ioctl(fd, VIDIOC_S_CTRL, &control) == -1) {
        perror("Setting exposure time");
        close(fd);
        return false;
    }

    close(fd);
    return true;
}


void writeImagePropertiesToYAML(const string& filename, int width, int height, const string& format, int quality, int HFoV, int VFoV) {
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "image_width" << YAML::Value << width;
    out << YAML::Key << "image_height" << YAML::Value << height;
    out << YAML::Key << "image_format" << YAML::Value << format;
    out << YAML::Key << "compression_quality" << YAML::Value << quality;
    out << YAML::Key << "horizontal FoV" << YAML::Value << HFoV;
    out << YAML::Key << "Vertical FoV" << YAML::Value << VFoV;
    out << YAML::EndMap;

    ofstream fout(filename);
    fout << out.c_str();
}


void parseArguments(int argc, char** argv) {
    for (int i = 1; i < argc; ++i) {
        string arg = argv[i];

        if (arg == "cam_type1") {
            cam_type = 1;
        } else if (arg == "cam_type2") {
            cam_type = 2;
        } else if (arg == "R_cam") {
            R_cam = true;
        } else if (arg == "C_cam") {
            C_cam = true;
        } else if (arg == "L_cam") {
            L_cam = true;
        } else if (arg == "trig_mod") {
            trig_mod = true;
        } else if (arg == "visualization") {
            visualization = true;
        } else if (arg == "distortion") {
            distortion = true;
        } else if (arg == "--log-dir" && i + 1 < argc) {
            hdd_log_path = argv[++i]; // Move to the next argument to get the directory path
        } else if (arg == "--output-dir" && i + 1 < argc) {
            hdd_out_path = argv[++i]; // Move to the next argument to get the output directory path
        }
    }
}


int main(int argc, char** argv) {
    parseArguments(argc, argv); 
    cout << hdd_out_path << endl;
    bool health_check = false;
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-health") == 0) {
            health_check = true;
            std::cout << "Health check enabled" << std::endl;
        }
    }
    char index = false;
    constexpr int LINE = 22;
    constexpr char CHIP[] = "/dev/gpiochip1";
    gpiod_chip* chip = gpiod_chip_open(CHIP);
    if (!chip) {
        throw std::runtime_error("Failed to open GPIO chip");
    }

    gpiod_line* gpio_line = gpiod_chip_get_line(chip, LINE);
    if (!gpio_line) {
        gpiod_chip_close(chip);
        throw std::runtime_error("Failed to get GPIO line");
    }

    if (gpiod_line_request_output(gpio_line, "blink", 0) < 0) {
        gpiod_chip_close(chip);
        throw std::runtime_error("Failed to request line as output");
    }

    std::string device0_path = "/dev/video0";
    std::string device1_path = "/dev/video2";
    std::string device2_path = "/dev/video4";
    std::string device_path_right;
    std::string device_path_left;
    std::string device_path_center;

    std::time_t now = std::time(nullptr);
    std::tm* now_tm = std::localtime(&now);
    char datetime[30];
    chrono::time_point<chrono::system_clock> end, start;
    int a = 0;
    int i = 0;
    int n = 0;
    int down_width = 300;
    int down_height = 200;
    Mat resized_down;
    int up_width = 600;
    int up_height = 400;
    Mat resized_up;
    Mat left_frame;
    Mat right_frame;
    Mat center_frame;
    Mat rotated;
    bool idx = true;
    CameraCalibration calib_left, calib_right, calib_center;
    int last_digit_1 = get_last_digit_of_bus_info(device0_path);
    int last_digit_2 = get_last_digit_of_bus_info(device1_path);
    int last_digit_3 = get_last_digit_of_bus_info(device2_path);

    std::string sys_log_path = hdd_log_path;
    std::string outputFolderRam_left;
    std::string outputFolderRam_right;
    std::string outputFolderRam_center;
    std::string outputFolderHDD_left;
    std::string outputFolderHDD_right;
    std::string outputFolderHDD_center;
    struct v4l2_control control1, control2, control3;

    std::strftime(datetime, sizeof(datetime), "%Y-%m-%d_%H-%M-%S", now_tm);

    if (L_cam){
        if (last_digit_1 == 3 || last_digit_2 == 3 || last_digit_3 == 3){
            h_idx++;
            sys_log << now << setw(6) << setfill('0') << "," << "Camera" << "," << "left camera plugged correctly" << endl;
        } else {
            sys_log << now << setw(6) << setfill('0') << "," << "Camera" << "," << "error in left camera or port 3 in hub" << endl;
        }
    }

    if (R_cam){
        if (last_digit_1 == 4 || last_digit_2 == 4 || last_digit_3 == 4){
            h_idx++;
            sys_log << now << setw(6) << setfill('0') << "," << "Camera" << "," << "right camera plugged correctly" << endl;
        } else {
            sys_log << now << setw(6) << setfill('0') << "," << "Camera" << "," << "error in right camera or port 3 in hub" << endl;
        }
    }

    if (C_cam){
        if (last_digit_1 == 2 || last_digit_2 == 2 || last_digit_3 == 2){
            h_idx++;
            sys_log << now << setw(6) << setfill('0') << "," << "Camera" << "," << "center camera plugged correctly" << endl;
        } else {
            sys_log << now << setw(6) << setfill('0') << "," << "Camera" << "," << "error in center camera or port 3 in hub" << endl;
        }
    }

    if (L_cam) {
        if (last_digit_1 == 3) {
            device_path_left = device0_path;
        } else if (last_digit_2 == 3) {
            device_path_left = device1_path;
        } else if (last_digit_3 == 3) {
            device_path_left = device2_path;
        } else {
            printf("There isn't a left camera\n");
        }
        outputFolderRam_left = "/media/ramdisk/left_camera";
        outputFolderHDD_left = hdd_out_path + "left_camera/imgs_" +  std::string(datetime);
        calib_left = readCalibration("/home/jetson/FlightDataPardis/left_calibration.yaml");
    }

    if (R_cam) {
        if (last_digit_1 == 4) {
            device_path_right = device0_path;
        } else if (last_digit_2 == 4) {
            device_path_right = device1_path;
        } else if (last_digit_3 == 4) {
            device_path_right = device2_path;
        } else {
            printf("There isn't a right camera\n");
        }
        outputFolderRam_right = "/media/ramdisk/right_camera";
        outputFolderHDD_right = hdd_out_path + "right_camera/imgs_" +  std::string(datetime);
        calib_right = readCalibration("/home/jetson/FlightDataPardis/right_calibration.yaml");
    }

    if (C_cam) {
        if (last_digit_1 == 2) {
            device_path_center = device0_path;
        } else if (last_digit_2 == 2) {
            device_path_center = device1_path;
        } else if (last_digit_3 == 2) {
            device_path_center = device2_path;
        } else {
            printf("There isn't a center camera\n");
        }
        outputFolderRam_center = "/media/ramdisk/center_camera";
        outputFolderHDD_center = hdd_out_path + "/" + "center_camera/imgs_" + std::string(datetime);
        calib_center = readCalibration("/home/jetson/FlightDataPardis/center_calibration.yaml");
    }
    string hdd_path_center = hdd_out_path + "/" + "center_camera/";
    string hdd_path_left = hdd_out_path + "/" + "left_camera/";
    string hdd_path_right = hdd_out_path + "/" + "right_camera/";

    mkdir(hdd_path_center.c_str(), 0777);
    mkdir(hdd_path_left.c_str(), 0777);
    mkdir(hdd_path_right.c_str(), 0777);

    if (L_cam) {
        ensureDirectoryExists(outputFolderHDD_left);
        ensureDirectoryExists(outputFolderRam_left);
        initCSV_hdd(outputFolderHDD_left + "/timestamps.csv");
        initCSV_ram("/media/ramdisk/lef_camera_timestamps.csv");
    }
    if (R_cam) {
        ensureDirectoryExists(outputFolderHDD_right);
        ensureDirectoryExists(outputFolderRam_right);
        initCSV_hdd(outputFolderHDD_right + "/timestamps.csv");
        initCSV_ram("/media/ramdisk/right_camera_timestamps.csv");
    }
    if (C_cam) {
        ensureDirectoryExists(outputFolderHDD_center);
        ensureDirectoryExists(outputFolderRam_center);
        initCSV_hdd(outputFolderHDD_center + "/timestamps.csv");
        initCSV_ram("/media/ramdisk/center_camera_timestamps.csv");
    }

    cv::VideoCapture capture1, capture2, capture3;
    // std::string center_camera_trig_dir = "/media/ramdisk/center_camera_trig/";
    // ensureDirectoryExists(center_camera_trig_dir);
    if (L_cam){
        capture1.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
        capture1.open(device_path_left, cv::CAP_V4L2);
        capture1.set(CAP_PROP_FPS, 60);
        capture1.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        capture1.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
        if (!capture1.isOpened()) {
            sys_log << now << setw(6) << setfill('0') << "," << "Camera" << "," << "ERROR: Could not open camera_left on port 3 in hub" << endl;
            cerr << "ERROR: Could not open camera_left" << endl;
            return -1;
        }

        // if (!setExposureTime(device_path_left, calib_left.exposure_time)) {
        //     cerr << "ERROR: Could not set exposure time for camera_left" << endl;
        //     return -1;
        // }

        control1.id = V4L2_CID_BACKLIGHT_COMPENSATION;
        control1.value = 0;  
        int fd1 = open(device_path_left.c_str(), O_RDWR);
        if (ioctl(fd1, VIDIOC_S_CTRL, &control1) == -1) {
            std::cerr << "Failed to set control: " << strerror(errno) << std::endl;
            close(fd1);
            return -1;
        }
        close(fd1);
    }

    if (R_cam){
        capture2.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
        capture2.open(device_path_right, cv::CAP_V4L2);
        capture2.set(CAP_PROP_FPS, 60);
        capture2.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        capture2.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
        if (!capture2.isOpened()) {
            sys_log << now << setw(6) << setfill('0') << "," << "Camera" << "," << "ERROR: Could not open camera_right on port 4 in hub" << endl;
            cerr << "ERROR: Could not open camera_right" << endl;
            return -1;
        }
        // if (!setExposureTime(device_path_right, calib_right.exposure_time)) {
        //     cerr << "ERROR: Could not set exposure time for camera_right" << endl;
        //     return -1;
        // }

        control2.id = V4L2_CID_BACKLIGHT_COMPENSATION;
        control2.value = 0;  
        int fd2 = open(device_path_right.c_str(), O_RDWR);
        if (ioctl(fd2, VIDIOC_S_CTRL, &control2) == -1) {
            std::cerr << "Failed to set control: " << strerror(errno) << std::endl;
            close(fd2);
            return -1;
        }
        close(fd2);
    }

    if (C_cam){
        capture3.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
        capture3.open(device_path_center, cv::CAP_V4L2);
        capture3.set(CAP_PROP_FPS, 60);
        capture3.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        capture3.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
        if (!capture3.isOpened()) {
            sys_log << now << setw(6) << setfill('0') << "," << "Camera" << "," << "ERROR: Could not open center_camera on port 4 in hub" << endl;
            cerr << "ERROR: Could not open center_camera" << endl;
            return -1;
        }

        // if (!setExposureTime(device_path_center, calib_center.exposure_time)) {
        //     cerr << "ERROR: Could not set exposure time for camera_center" << endl;
        //     return -1;
        // }

        control3.id = V4L2_CID_BACKLIGHT_COMPENSATION;
        control3.value = 0; 
        int fd3 = open(device_path_center.c_str(), O_RDWR);
        if (ioctl(fd3, VIDIOC_S_CTRL, &control3) == -1) {
            std::cerr << "Failed to set control: " << strerror(errno) << std::endl;
            close(fd3);
            return -1;
        }
        close(fd3);
    }

    Mat new_camera_matrix_left, new_camera_matrix_right, new_camera_matrix_center;
    Mat undistorted_frame_left, undistorted_frame_right, undistorted_frame_center;

    if(cam_type == 1){
        while (true) {
            // std::cout << h_idx;
            if (i == 110 && health_check == true){
                if (h_idx == 4){
                    printf("camera is healthy");
                } else {
                    printf("camera is unhealthy");
                }
            }

            // if (index == false){
            //     if (R_cam) capture2.read(right_frame);
            //     index = true;
            // }

            if (trig_mod){
                auto now = high_resolution_clock::now();
                auto now_sec = duration_cast<seconds>(now.time_since_epoch()).count();  // seconds since epoch
                auto now_microsec = duration_cast<microseconds>(now.time_since_epoch()).count() % 1000000;  // microseconds part
                // cout << "timestamp while trigging: " << now_sec << setw(6) << setfill('0') << now_microsec << endl;

                gpiod_line_set_value(gpio_line, 1);
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                gpiod_line_set_value(gpio_line, 0);
            }

            bool frames_read = true;
            if (L_cam) frames_read &= capture1.read(left_frame);
            if (R_cam) frames_read &= capture2.read(right_frame);
            if (C_cam) frames_read &= capture3.read(center_frame);
            
            if(trig_mod) std::this_thread::sleep_for(std::chrono::milliseconds(5));
            

            // if (frames_read){
            //     h_idx++;
            //     gpiod_line_set_value(gpio_line, 0);
            // } else {
            //     sys_log << now << setw(6) << setfill('0') << "," << "Camera" << "," << "ERROR: No more frames on camera right and left" << endl;
            //     cerr << "ERROR: No more frames on camera right and left" << endl;
            //     break;
            // }

            // Calculate frequency
            start = chrono::system_clock::now();
            if (idx) {
                end = start;
                idx = false;
            }
            chrono::duration<double> elapsed_seconds = start - end;
            double freq = elapsed_seconds.count() > 0 ? 1.0 / elapsed_seconds.count() : 0;
            i++;

            if (visualization){
                a += freq;
                n = a / i;
                cout << "Camera average frequency: " << n << endl;
            }

            if (i == 5) {
                struct v4l2_control control1, control2, control3;

                if (L_cam && trig_mod) {
                    control1.id = V4L2_CID_BACKLIGHT_COMPENSATION;
                    control1.value = 2;  
                    int fd1 = open(device_path_left.c_str(), O_RDWR);
                    if (ioctl(fd1, VIDIOC_S_CTRL, &control1) == -1) {
                        std::cerr << "Failed to set control: " << strerror(errno) << std::endl;
                        close(fd1);
                        return -1;
                    }
                    close(fd1);
                }

                if (R_cam && trig_mod) {
                    control2.id = V4L2_CID_BACKLIGHT_COMPENSATION;
                    control2.value = 2;  
                    int fd2 = open(device_path_right.c_str(), O_RDWR);
                    if (ioctl(fd2, VIDIOC_S_CTRL, &control2) == -1) {
                        std::cerr << "Failed to set control: " << strerror(errno) << std::endl;
                        close(fd2);
                        return -1;
                    }
                    close(fd2);
                }

                if (C_cam && trig_mod) {
                    control3.id = V4L2_CID_BACKLIGHT_COMPENSATION;
                    control3.value = 2; 
                    int fd3 = open(device_path_center.c_str(), O_RDWR);
                    if (ioctl(fd3, VIDIOC_S_CTRL, &control3) == -1) {
                        std::cerr << "Failed to set control: " << strerror(errno) << std::endl;
                        close(fd3);
                        return -1;
                    }
                    close(fd3);
                }
            }
            if (distortion){
                if (L_cam) {
                    new_camera_matrix_left = cv::getOptimalNewCameraMatrix(calib_left.camera_matrix, calib_left.distortion_coefficients, left_frame.size(), 1);
                    cv::Rect validRoi_left;
                    new_camera_matrix_left = cv::getOptimalNewCameraMatrix(calib_left.camera_matrix, calib_left.distortion_coefficients, left_frame.size(), 1, left_frame.size(), &validRoi_left);

                    cv::undistort(left_frame, undistorted_frame_left, calib_left.camera_matrix, calib_left.distortion_coefficients, new_camera_matrix_left);
                    undistorted_frame_left = undistorted_frame_left(validRoi_left);
                }

                if (R_cam) {
                    new_camera_matrix_right = cv::getOptimalNewCameraMatrix(calib_right.camera_matrix, calib_right.distortion_coefficients, right_frame.size(), 1);
                    cv::Rect validRoi_right;
                    new_camera_matrix_right = cv::getOptimalNewCameraMatrix(calib_right.camera_matrix, calib_right.distortion_coefficients, right_frame.size(), 1, right_frame.size(), &validRoi_right);

                    cv::undistort(right_frame, undistorted_frame_right, calib_right.camera_matrix, calib_right.distortion_coefficients, new_camera_matrix_center);
                    undistorted_frame_right = undistorted_frame_right(validRoi_right);
                }

                if (C_cam) {
                    new_camera_matrix_center = cv::getOptimalNewCameraMatrix(calib_center.camera_matrix, calib_center.distortion_coefficients, center_frame.size(), 1);
                    cv::Rect validRoi_center;
                    new_camera_matrix_center = cv::getOptimalNewCameraMatrix(calib_center.camera_matrix, calib_center.distortion_coefficients, center_frame.size(), 1, center_frame.size(), &validRoi_center);

                    cv::undistort(center_frame, undistorted_frame_center, calib_center.camera_matrix, calib_center.distortion_coefficients, new_camera_matrix_center);
                    undistorted_frame_center = undistorted_frame_center(validRoi_center);
                }

                if (visualization){
                    if (L_cam) imshow("left", undistorted_frame_left);
                    if (R_cam) imshow("right", undistorted_frame_right);
                    if (C_cam) imshow("center", undistorted_frame_center);

                }   
                saveFrame(undistorted_frame_left, undistorted_frame_right, undistorted_frame_center, outputFolderRam_left, outputFolderHDD_left, outputFolderRam_right, outputFolderHDD_right, outputFolderRam_center, outputFolderHDD_center);

            }
            else {
                if (visualization){
                    if (L_cam) imshow("left", left_frame);
                    if (R_cam) imshow("right", right_frame);
                    if (C_cam) imshow("center", center_frame);
                } 
                cv::rotate(center_frame, rotated, cv::ROTATE_180);
                saveFrame(right_frame, left_frame,  rotated, outputFolderRam_left, outputFolderHDD_left, outputFolderRam_right, outputFolderHDD_right, outputFolderRam_center, outputFolderHDD_center);

            }

            if (waitKey(1) == 27) {  // 'ESC' key to quit
                cout << "ESC key pressed, exiting..." << endl;
                break;
            }
            end = start;  // Update end to current start for next loop
        }
    }

    if (cam_type == 2){
        while (true) {
            // std::cout << h_idx;

            // Calculate frequency
            start = chrono::system_clock::now();
            if (idx) {
                end = start;
                idx = false;
            }
            chrono::duration<double> elapsed_seconds = start - end;
            double freq = elapsed_seconds.count() > 0 ? 1.0 / elapsed_seconds.count() : 0;
            i++;

            if (visualization){
                a += freq;
                n = a / i;
                cout << "Camera average frequency: " << n << endl;
            }

            if (trig_mod) {
                auto now = high_resolution_clock::now();
                auto now_sec = duration_cast<seconds>(now.time_since_epoch()).count();  // seconds since epoch
                auto now_microsec = duration_cast<microseconds>(now.time_since_epoch()).count() % 1000000;  // microseconds part
                // cout << "timestamp while trigging: " << now_sec << setw(6) << setfill('0') << now_microsec << endl;
                gpiod_line_set_value(gpio_line, 1);
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                gpiod_line_set_value(gpio_line, 0);
            }

            bool frames_read = true;
            if (L_cam) frames_read &= capture1.read(left_frame);
            if (R_cam) frames_read &= capture2.read(right_frame);
            if (C_cam) frames_read &= capture3.read(center_frame);

            if (!frames_read) {
                cerr << "ERROR: Could not read frames from cameras" << endl;
                break;
            }

            if(trig_mod) std::this_thread::sleep_for(std::chrono::milliseconds(10));

            if (i == 50) {
                struct v4l2_control control1, control2, control3;

                if (L_cam && trig_mod) {
                    control1.id = V4L2_CID_BACKLIGHT_COMPENSATION;
                    control1.value = 1;  
                    int fd1 = open(device_path_left.c_str(), O_RDWR);
                    if (ioctl(fd1, VIDIOC_S_CTRL, &control1) == -1) {
                        std::cerr << "Failed to set control: " << strerror(errno) << std::endl;
                        close(fd1);
                        return -1;
                    }
                    close(fd1);
                }

                if (R_cam && trig_mod) {
                    control2.id = V4L2_CID_BACKLIGHT_COMPENSATION;
                    control2.value = 1;  
                    int fd2 = open(device_path_right.c_str(), O_RDWR);
                    if (ioctl(fd2, VIDIOC_S_CTRL, &control2) == -1) {
                        std::cerr << "Failed to set control: " << strerror(errno) << std::endl;
                        close(fd2);
                        return -1;
                    }
                    close(fd2);
                }

                if (C_cam && trig_mod) {
                    control3.id = V4L2_CID_BACKLIGHT_COMPENSATION;
                    control3.value = 1; 
                    int fd3 = open(device_path_center.c_str(), O_RDWR);
                    if (ioctl(fd3, VIDIOC_S_CTRL, &control3) == -1) {
                        std::cerr << "Failed to set control: " << strerror(errno) << std::endl;
                        close(fd3);
                        return -1;
                    }
                    close(fd3);
                }
            }

            if (distortion){
                if (L_cam) {
                    new_camera_matrix_left = cv::getOptimalNewCameraMatrix(calib_left.camera_matrix, calib_left.distortion_coefficients, left_frame.size(), 1);
                    cv::Rect validRoi_left;
                    new_camera_matrix_left = cv::getOptimalNewCameraMatrix(calib_left.camera_matrix, calib_left.distortion_coefficients, left_frame.size(), 1, left_frame.size(), &validRoi_left);

                    cv::undistort(left_frame, undistorted_frame_left, calib_left.camera_matrix, calib_left.distortion_coefficients, new_camera_matrix_left);
                    undistorted_frame_left = undistorted_frame_left(validRoi_left);
                }

                if (R_cam) {
                    new_camera_matrix_right = cv::getOptimalNewCameraMatrix(calib_right.camera_matrix, calib_right.distortion_coefficients, right_frame.size(), 1);
                    cv::Rect validRoi_right;
                    new_camera_matrix_right = cv::getOptimalNewCameraMatrix(calib_right.camera_matrix, calib_right.distortion_coefficients, right_frame.size(), 1, right_frame.size(), &validRoi_right);

                    cv::undistort(right_frame, undistorted_frame_right, calib_right.camera_matrix, calib_right.distortion_coefficients, new_camera_matrix_center);
                    undistorted_frame_right = undistorted_frame_right(validRoi_right);
                }

                if (C_cam) {
                    new_camera_matrix_center = cv::getOptimalNewCameraMatrix(calib_center.camera_matrix, calib_center.distortion_coefficients, center_frame.size(), 1);
                    cv::Rect validRoi_center;
                    new_camera_matrix_center = cv::getOptimalNewCameraMatrix(calib_center.camera_matrix, calib_center.distortion_coefficients, center_frame.size(), 1, center_frame.size(), &validRoi_center);

                    cv::undistort(center_frame, undistorted_frame_center, calib_center.camera_matrix, calib_center.distortion_coefficients, new_camera_matrix_center);
                    undistorted_frame_center = undistorted_frame_center(validRoi_center);
                }

                if (visualization){
                    if (L_cam) imshow("left", undistorted_frame_left);
                    if (R_cam) imshow("right", undistorted_frame_right);
                    if (C_cam) imshow("center", undistorted_frame_center);

                }   

                saveFrame(undistorted_frame_left, undistorted_frame_right, undistorted_frame_center, outputFolderRam_left, outputFolderHDD_left, outputFolderRam_right, outputFolderHDD_right, outputFolderRam_center, outputFolderHDD_center);

            }
            else {
                if (visualization){
                    if (L_cam) imshow("left", left_frame);
                    if (R_cam) imshow("right", right_frame);
                    if (C_cam) imshow("center", center_frame);
                } 

                saveFrame(right_frame, left_frame, center_frame, outputFolderRam_left, outputFolderHDD_left, outputFolderRam_right, outputFolderHDD_right, outputFolderRam_center, outputFolderHDD_center);

            }
            // std::stringstream ss;
            // ss << now_sec << std::setw(6) << std::setfill('0') << now_microsec;
            // std::string timestamp = ss.str();

            // // Add the timestamp to the center image
            // cv::Point textOrg(50, 50);  // Position the text
            // int fontFace = cv::FONT_HERSHEY_SIMPLEX;  // Font type
            // double fontScale = 1;  // Font scale
            // int thickness = 2;  // Text thickness
            // cv::Scalar color(255, 255, 255);  // White color

            // // Apply the text (timestamp) to the undistorted center frame
            // cv::putText(undistorted_frame_center, timestamp, textOrg, fontFace, fontScale, color, thickness);

            // // Create the filename
            // std::stringstream filenamee;
            // filenamee << center_camera_trig_dir << now_sec << setw(6) << setfill('0') << now_microsec << ".jpg";

            // // Save the image and check if it was successful
            // if (!imwrite(filenamee.str(), undistorted_frame_center)) {
            //     cerr << "ERROR: Could not write the image to " << filenamee.str() << endl;
            // } else {
            //     cout << "Image successfully written to " << filenamee.str() << endl;
            // }
            
            if (waitKey(1) == 27) {  // 'ESC' key to quit
                cout << "ESC key pressed, exiting..." << endl;
                break;
            }
            end = start;  // Update end to current start for next loop
        }
    }
    // cv2::Get.image.size(width, height)
    closeCSV(); // close csv file
    if (L_cam) capture1.release();  
    if (R_cam) capture2.release();  
    if (C_cam) capture3.release();  
    destroyAllWindows(); // Close all windows
    return 0;
}
