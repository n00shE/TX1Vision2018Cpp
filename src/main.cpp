/*
TO-DO
Safe ways to quit not relying on opencv windows
adding actual pipeline
networktables
*/

// Standard includes
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <time.h>

// ZED includes
#include <sl/Camera.hpp>

// GRIP
#include "GripPipeline.h"

// OpenCV
#include <opencv2/opencv.hpp>

// NetworkTables
#include "networktables/NetworkTable.h" //networktables

// Using std namespace
using namespace std;
using namespace sl;

#define USE_CHUNKS 1
char key = ' ';

const std::string currentDateTime() {
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
    // for more information about date/time format
    strftime(buf, sizeof(buf), "%Y-%m-%d-%X", &tstruct);

    return buf;
}

std::string timenow = currentDateTime();
std::string meshoutput = "/mnt/c482766e-ece6-4ec0-8ed2-01712e4e5516/recording" + timenow + ".svo";
const char *output = meshoutput.c_str();

// Create ZED objects
sl::Camera zed;
sl::Mat zed_image;  // sl::Mat to hold images
sl::Pose pose;      // sl::Pose to hold pose data
sl::Mesh mesh;      // sl::Mesh to hold the mesh generated during spatial mapping
sl::SpatialMappingParameters spatial_mapping_params;
sl::MeshFilterParameters filter_params;
sl::TRACKING_STATE tracking_state;

// Spatial Mapping status
bool mapping_is_started = false;
std::chrono::high_resolution_clock::time_point t_last;

void startMapping() {
    // clear previously used objects
    mesh.clear();

    // Enable positional tracking before starting spatial mapping
    zed.enableTracking();
    // Enable spatial mapping
    sl::ERROR_CODE err = zed.enableSpatialMapping(spatial_mapping_params);

    if (err == sl::SUCCESS) {
        t_last = std::chrono::high_resolution_clock::now(); // Start a timer, we retrieve the mesh every XXms.
        mapping_is_started = true;
        std::cout << "** Spatial Mapping is started ... **" << std::endl;
    }
}

void shutdown() {
    // Stop the mesh request and extract the whole mesh to filter it and save it as an obj file
    mapping_is_started = false;
    std::cout << "** Stop Spatial Mapping ... **" << std::endl;

    // Extract the whole mesh
    sl::Mesh wholeMesh;
    zed.extractWholeMesh(wholeMesh);
    std::cout << ">> Mesh has been extracted..." << std::endl;

    // Filter the extracted mesh
    wholeMesh.filter(filter_params, USE_CHUNKS);
    std::cout << ">> Mesh has been filtered..." << std::endl;

    // If textures have been saved during spatial mapping, apply them to the mesh
    if (spatial_mapping_params.save_texture) {
        wholeMesh.applyTexture(sl::MESH_TEXTURE_RGB);
        std::cout << ">> Mesh has been textured..." << std::endl;
    }

    //Save as an OBJ file
    std::string saveName = "/mnt/c482766e-ece6-4ec0-8ed2-01712e4e5516/mesh" + timenow + ".obj";
    bool t = wholeMesh.save(saveName.c_str());
    if (t) std::cout << ">> Mesh has been saved under " << saveName << std::endl;
    else std::cout << ">> Failed to save the mesh under  " << saveName << std::endl;

	zed.saveCurrentArea("areamap" + timenow + ".area"); // The actual file will be created asynchronously.
	std::cout << zed.getAreaExportState() << std::endl;

	zed.disableRecording();
	zed.close();

	//if(std::rename("/mnt/c482766e-ece6-4ec0-8ed2-01712e4e5516/recording.svo", meshoutput) < 0) {
		//std::cout << strerror(errno) << '\n';
	//}
}

int main() {
    // Set configuration parameters for the ZED
    sl::InitParameters parameters;
    parameters.camera_resolution = RESOLUTION_VGA;
    parameters.camera_fps = 30; 
    parameters.depth_mode = DEPTH_MODE_PERFORMANCE;
    parameters.coordinate_units = UNIT_METER;
    parameters.coordinate_system = COORDINATE_SYSTEM_RIGHT_HANDED_Y_UP;

    sl::ERROR_CODE err = zed.open(parameters);
    if (err != sl::ERROR_CODE::SUCCESS) {
        std::cout << sl::toString(err) << std::endl;
        zed.close();
        return(1);
    }

    // Print camera information
    printf("ZED Model                 : %s\n", toString(zed.getCameraInformation().camera_model).c_str());
    printf("ZED Serial Number         : %d\n", zed.getCameraInformation().serial_number);
    printf("ZED Firmware              : %d\n", zed.getCameraInformation().firmware_version);
    printf("ZED Camera Resolution     : %dx%d\n", (int) zed.getResolution().width, (int) zed.getResolution().height);
    printf("ZED Camera FPS            : %d\n", (int) zed.getCameraFPS());


    // Configure Spatial Mapping and filtering parameters
    spatial_mapping_params.range_meter = sl::SpatialMappingParameters::get(sl::SpatialMappingParameters::MAPPING_RANGE_FAR);
    spatial_mapping_params.resolution_meter = sl::SpatialMappingParameters::get(sl::SpatialMappingParameters::MAPPING_RESOLUTION_LOW);
    spatial_mapping_params.save_texture = true;
    spatial_mapping_params.max_memory_usage = 2048;
    spatial_mapping_params.use_chunk_only = USE_CHUNKS; // If we use chunks we do not need to keep the mesh consistent
    filter_params.set(sl::MeshFilterParameters::MESH_FILTER_LOW);
    zed.setCameraSettings(CAMERA_SETTINGS_EXPOSURE, 30);

    zed.enableRecording(output, SVO_COMPRESSION_MODE_LOSSY); //sl::SVO_COMPRESSION_MODE_LOSSY
    if (err != SUCCESS) {
        std::cout << "Recording initialization error. " << toString(err) << std::endl;
    if (err == ERROR_CODE_SVO_RECORDING_ERROR) std::cout << " Note : This error mostly comes from a wrong path or missing writing permissions." << std::endl;
    }
    startMapping();
    while (key != 'q') {
        if (zed.grab() == sl::SUCCESS) {
            // Retrieve image
            zed.retrieveImage(zed_image, VIEW_LEFT);
            cv::imshow("VIEW", cv::Mat((int) zed_image.getHeight(), (int) zed_image.getWidth(), CV_8UC4, zed_image.getPtr<sl::uchar1>(sl::MEM_CPU)));
            key = cv::waitKey(5);
            zed.record();
			zed.getPosition(cameraPose, REFERENCE_FRAME_WORLD);
        }
        if (mapping_is_started == false) {
            startMapping();
        }
    }
    if (key == 'q') {
        shutdown();
    }
}
