#include <sstream>

#include "helpers.h"
#include "SensorDataHandler.h"
#include "SensorDataViewer.h"


// Initialize the viewer window
SensorDataViewer::SensorDataViewer(const std::string& title,
    std::vector<SensorDataHandler *> &sensorDataHandlers)
  : myViewer(title), myRefreshTime(100),
    mySensorDataHandlers(sensorDataHandlers)
{
  myViewer.setBackgroundColor(0,0,0);
  myViewer.addCoordinateSystem(170.0);
  myViewer.initCameraParameters();
  request();
  initDisplay();
}

// Add point clouds to be displayed in window
void SensorDataViewer::initDisplay()
{
  static std::ostringstream os;
  for (size_t i = 0; i < mySensorDataHandlers.size(); i++) {
    os.str("");
    os << "cloud" << i;
    myViewer.addPointCloud(mySensorDataHandlers[i]->displayCloud(),
	os.str());
  }
  myViewer.spinOnce(myRefreshTime/2);
}

// Show the update point clouds
void SensorDataViewer::updateDisplay()
{
  static std::ostringstream os;
  for (size_t i = 0; i < mySensorDataHandlers.size(); i++) {
    os.str("");
    os << "cloud" << i;
    myViewer.updatePointCloud(mySensorDataHandlers[i]->displayCloud(), 
	os.str());
  }
  myViewer.spinOnce(myRefreshTime);
}

// Refresh the viewer window so that it may load new data
void SensorDataViewer::request()
{
  for (size_t i = 0; i < mySensorDataHandlers.size(); i++)
    mySensorDataHandlers[i]->request();
}
