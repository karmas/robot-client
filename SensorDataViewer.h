#ifndef SENSOR_DATA_VIEWER
#define SENSOR_DATA_VIEWER

#include <vector>

#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>

// forward declarations
class SensorDataHandler;


// This class is responsible for displaying point clouds on a viewer
class SensorDataViewer {
public:
  SensorDataViewer(const std::string& title,
      	    std::vector<SensorDataHandler *> &sensorDataHandlers); 
  void request();
  void initDisplay();
  void updateDisplay();

private:
  SensorDataViewer(const SensorDataViewer&);
  SensorDataViewer& operator=(const SensorDataViewer&);

  pcl::visualization::PCLVisualizer myViewer;
  int myRefreshTime;	// in milliseconds
  std::vector<SensorDataHandler *> &mySensorDataHandlers;
};



void createViewer(SensorDataViewer *&viewer,
    std::vector<SensorDataHandler *> &sensorDataHandlers);

#endif
