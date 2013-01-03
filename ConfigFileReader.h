#ifndef CONFIG_FILE_READER_H
#define CONFIG_FILE_READER_H

// initial offsets of robots from global co-ordinates
struct TransformInfo {
  TransformInfo(int xo, int yo, int to)
    : xOffset(xo), yOffset(yo), thetaOffset(to) { }
  int xOffset;
  int yOffset;
  int thetaOffset;
};

// Performs checking of the custom file type
class ConfigFileReader {
public:
  ConfigFileReader(int c, char **v, ArArgumentParser *parser)
    : myArgc(c), myArgv(v), myParser(parser), fileType(-1) { }
  void printHeaders();
  void readHostsFile(vector<const char *> &ips,
                     vector<int> &robotColors,
                     vector<int> &colors,
                     vector<TransformInfo> &transforms);
  const char **stringChunk(const char *start, const char *end,
      			   char separator, int &n);

  static const char *hostsArg;
  static const char *hostsFileHeaders[];
  static const char *hostsFileDescs[];
  static const char *hostsFileFormats[];
  static const int hostsFileTypes = 4;

private:
  int myArgc;
  char **myArgv;
  ArArgumentParser *myParser;
  int fileType;

  int checkFileArg();
  ifstream *getFileType();
  void removeStorage(void *a, int n);
};



#endif
