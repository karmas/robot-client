#include "Aria.h"

#include "helpers.h"
#include "ConfigFileReader.h"
#include "OutputHandler.h"

// some message display routines
void echo(const std::string &msg)
{
  std::cout << "\t" << msg << std::endl;
}

void echo(const std::string &id, const int value)
{
  std::cout << "\t" << id << " = " << value << std::endl;
}

void echo(const std::string &id, const double value)
{
  std::cout << "\t" << id << " = " << value << std::endl;
}

void echo(const std::string &id, const std::string &value)
{
  std::cout << "\t" << id << " = " << value << std::endl;
}


void errorExit(std::string msg)
{
  echo(msg);
  Aria::shutdown();
  std::exit(1);
}

// Pack the color information into a single integer which is needed for
// PCL point types.
int rgba(int r, int g, int b) {
  return b + 256*g +256*256*r;
}

// some useful constants
const std::string outDirPrefix = "clouds";

// shuts down aria 
void escapePressed()
{ 
  Aria::shutdown(); 
}


// used to create a name of form botxxx where xxx is the last 3 digits
// of the IP address
const char *createRobotName(const char *IP)
{
  std::string ip = IP;
  int suffixStart = ip.rfind(".") + 1;
  std::string suffix = ip.substr(suffixStart);
  std::string name = "bot" + suffix;
  return name.c_str();
}

// Connects to each IP address in hostsIP.
// The client objects are stored in clients.
void connectHosts(std::vector<ArClientBase *> &clients,
                  const std::vector<HostInfo> &hostsInfo)
{
  ArClientBase *client = NULL;

  int portNums[] = { 7272, 7273 };

  for (unsigned int i = 0; i < hostsInfo.size(); i++) {
    client = new ArClientBase;
    client->setRobotName(createRobotName(hostsInfo[i].ip));
    if (!client->blockingConnect(hostsInfo[i].ip, portNums[i])) {
      echo("unable to connect to", client->getRobotName());
      Aria::shutdown();
      exit(1);
    }
    else {
      echo("connected to", client->getRobotName());
      clients.push_back(client);
    }
  }
}

// create connection to receive point cloud data for all clients
void createPCLReceivers(std::vector<ArClientBase *> &clients,
    			PCLViewer *viewer,
			std::vector<PCLOutputHandler *> &pclClients,
			std::vector<HostInfo> &hostsInfo)
{
  PCLOutputHandler *pclHandler = NULL;

  for (unsigned int i = 0; i < clients.size(); i++) {
    pclHandler = new PCLOutputHandler(clients[i], viewer,
		     hostsInfo[i].locationColor,
		     hostsInfo[i].laserColor,
		     hostsInfo[i].transformInfo.xOffset,
		     hostsInfo[i].transformInfo.yOffset,
		     hostsInfo[i].transformInfo.thetaOffset,
		     hostsInfo[i].requestFreq);
    pclClients.push_back(pclHandler);
  }
}

// just start all the clients
void startClients(std::vector<ArClientBase *> clients)
{
  for (unsigned int i = 0; i < clients.size(); i++) {
    clients[i]->runAsync();
  }
}

// Creats a string representing the current time in a readable format.
// Format: month - day _ hour : min : sec
std::string genTimeStr()
{
  const char SEPARATOR = '_';
  const char DATE_SEPARATOR = '-';
  const char TIME_SEPARATOR = ':';
  time_t seconds = time(NULL);
  struct tm *timeInfo = localtime(&seconds);

  std::ostringstream new_name;
  new_name << timeInfo->tm_mon + 1 << DATE_SEPARATOR
           << timeInfo->tm_mday << SEPARATOR
	   << timeInfo->tm_hour << TIME_SEPARATOR
	   << timeInfo->tm_min << TIME_SEPARATOR
	   << timeInfo->tm_sec;

  return new_name.str();
}

// Creates a directory based on prefix constant and the current time and
// returns that name.
std::string genOutDir()
{
  std::string dirName = outDirPrefix + genTimeStr();

  // error occurred while creating a new directory
  if (mkdir(dirName.c_str(), S_IRWXU | S_IRWXG) == -1) {
    std::cout << "Error creating " << dirName << ": " << std::endl;
    std::cout << strerror(errno) << std::endl;
    return "";
  }

  return dirName;
}

// It creates a new ouput folder where all the point clouds will be stored.
// The output folder has name that is based on a chosen prefix which is
// held in the string variable outDirPrefix and the current time.
void writeCloudToFile(std::vector<PCLOutputHandler *> &pclClients)
{
  // generate a new output directory based on current time
  std::string outDir = genOutDir();
  // error message already spawned by genOutDir
  if (outDir == "") return;

  std::string prefix = "";
  std::string fileName = "";
  std::string extension = ".pcd";
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;

  for (size_t i = 0; i < pclClients.size(); i++) {
    prefix = outDir + "/" + pclClients[i]->getClient()->getHost();
    // robot position cloud filename
    fileName = prefix + extension;
    // write the file
    cloud = pclClients[i]->getRobotCloud();
    pcl::io::savePCDFile(fileName, *cloud);

    // generate cloud files corresponding to the time stamped cloud files
    std::vector<TimeStampedPCL *> *laserClouds = 
      pclClients[i]->getLaserClouds();
    int j = 0;
    for (std::vector<TimeStampedPCL *>::const_iterator it =
	 laserClouds->begin(); it != laserClouds->end(); it++) {
      cloud = (*it)->getCloud();
      std::ostringstream os;
      os << (*it)->getTimeStamp();
      fileName = prefix + "_" + os.str() + extension;
      pcl::io::savePCDFile(fileName, *cloud);
    }
  }

  std::cout << "Wrote clouds to: " << outDir << std::endl;
}
