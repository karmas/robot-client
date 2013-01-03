#include <iostream>
#include <fstream>
#include <vector>
#include <cstdlib>
using namespace std;

#include "Aria.h"
#include "ConfigFileReader.h"
#include "helpers.h"

// this is the required command line argument
const char *ConfigFileReader::hostsArg = "-hosts";
// this are the valid custom file headers
const char *ConfigFileReader::hostsFileHeaders[] = {
  "servers 1.0",
  "servers 1.0 colors",
  "servers 1.0 transform",
  "servers 1.0 paths"
};
// this are the valid custom file headers
const char *ConfigFileReader::hostsFileDescs[] = {
  "A single line contains only IP address",
  "also give red,green,blue values for point cloud",
  "also give x translation, y translation and theta rotation for PCL data",
  "also give red,green,blue values for robot itself"
};
// this are the valid custom file headers
const char *ConfigFileReader::hostsFileFormats[] = {
  "10.10.3.113",
  "10.10.3.113 200,10,10",
  "10.10.3.113 200,10,10 10,1000,5",
  "10.10.3.113 200,10,10 10,1000,5 10,200,10",
};

// check if argument option is given in command line
// and returns the command line index of the filename
int ConfigFileReader::checkFileArg()
{
  int fileIndex = -1;
  if (!myParser->checkArgument(hostsArg))
    errorExit("use : -hosts 'filename'");

  for (int i = 1; i < myArgc; i++) {
    if (strcmp(hostsArg, myArgv[i]) == 0) {
      fileIndex = i + 1;
      break;
    }
  }

  if (fileIndex == -1 || fileIndex >= myArgc)
    errorExit("use : -hosts 'filename'");
  
  return fileIndex;
}

// Figures out the file type by reading the first line.
// If it finds match, it returns file stream pointer and
// sets fileType member
ifstream *ConfigFileReader::getFileType()
{
  const int BUFFER_LEN = 50;
  char buffer[BUFFER_LEN];

  // filename is in this index of myArgv
  int fileIndex = checkFileArg();

  // open file and get file type
  ifstream *file = new ifstream(myArgv[fileIndex], ifstream::in);
  if (file->fail()) errorExit("NO SUCH FILE");
  file->getline(buffer, BUFFER_LEN);

  for (int i = 0; i < hostsFileTypes; i++) {
    if (strcmp(buffer, hostsFileHeaders[i]) == 0) {
      fileType = i;
      return file;
    }
  }

  file->close();
  delete file;

  printHeaders();
  errorExit("VALID HEADER NOT FOUND!!! ");
  return NULL;	// never reached
}

// display the proper headers
void ConfigFileReader::printHeaders()
{
  echo("VALID HEADERS:");
  for (int i = 0; i < hostsFileTypes; i++) {
    echo(hostsFileHeaders[i]);
    cout << "\t\t" << hostsFileDescs[i] << endl;
    cout << "\t\te.g: " << hostsFileFormats[i] << endl;
  }
  cout << endl;
}

// Given a starting and ending point, it fills chunks with addresses
// of clones of fields separated by given separator and returns number
// of fields found.
const char **ConfigFileReader::stringChunk(const char *start,
    const char *end, char separator, int &n)
{
  // maybe the whole section is a single field
  const char *fieldStart = start;
  const char *fieldEnd = end;
  const char **chunks = NULL;

  while( (fieldEnd=strchr(fieldStart, separator)) != NULL  && fieldEnd < end) {
    // create clone
    chunks = (const char **)realloc(chunks, n*sizeof(char *));
    chunks[n-1] = strndup(fieldStart, fieldEnd - fieldStart);

    fieldStart = fieldEnd + 1;
    n++;
  }

  // last field or only field
  // no equivalent of realloc in C++
  chunks = (const char **)realloc(chunks, n*sizeof(char *));
  chunks[n-1] = strndup(fieldStart, end - fieldStart);

  return chunks;
}

// a is a pointer to an array of pointers
void ConfigFileReader::removeStorage(void *a, int n)
{
  char **base = (char **)a;
  for (int i = 0; i < n; i++)
    free(base[i]);	// free individual pointers
  free(base);		// and the whole array
}

// reads files of type : servers 1.0
void ConfigFileReader::readHostsFile(vector<const char *> &ips,
    vector<int> &robotColors, vector<int> &colors,
    vector<TransformInfo> &transforms)
{
  const int BUFFER_LEN = 70;
  char buffer[BUFFER_LEN];
  const char *sectionStart = NULL;
  const char *sectionEnd = NULL;
  const char SECTION_SEPARATOR = ' ';
  const char FIELD_SEPARATOR = ',';
  int fields = 1;	// there is at least one field
  const char **chunks = NULL; // clones of the fields will be stored here

  // first set file type
  ifstream *file = getFileType();

  // process each line
  while (file->good()) {
    // first get line
    file->getline(buffer, BUFFER_LEN);
    if (file->fail() || file->eof()) break;

    // extract IP
    sectionStart = buffer;
    if (fileType > 0) sectionEnd = strchr(sectionStart, SECTION_SEPARATOR);
    else sectionEnd = sectionStart + strlen(sectionStart);
    fields = 1;
    chunks = stringChunk(sectionStart, sectionEnd, FIELD_SEPARATOR, fields);
    // add to ip address list
    ips.push_back(chunks[0]);
    // free the storage for the array but not the fields which hold
    // addresses to strings
    free(chunks);

    if (fileType < 1) continue;

    // extract colors
    sectionStart = sectionEnd + 1;
    if (fileType > 1) sectionEnd = strchr(sectionStart, SECTION_SEPARATOR);
    else sectionEnd = sectionStart + strlen(sectionStart);
    fields = 1;
    chunks = stringChunk(sectionStart, sectionEnd, FIELD_SEPARATOR, fields);
    // store in colors list
    colors.push_back(rgba(atoi(chunks[0]),
	                  atoi(chunks[1]),
			  atoi(chunks[2])));
    // free chunks which is no longer needed
    removeStorage(chunks, fields);

    if (fileType < 2) continue;

    // extract transformations
    sectionStart = sectionEnd + 1;
    if (fileType > 2) sectionEnd = strchr(sectionStart, SECTION_SEPARATOR);
    else sectionEnd = sectionStart + strlen(sectionStart);
    fields = 1;
    chunks = stringChunk(sectionStart, sectionEnd, FIELD_SEPARATOR, fields);
    // store in transformations list
    transforms.push_back(TransformInfo(atoi(chunks[0]),
	                               atoi(chunks[1]), 
				       atoi(chunks[2])));
    // free chunks which is no longer needed
    removeStorage(chunks, fields);

    if (fileType < 3) continue;

    // extract robot colors
    sectionStart = sectionEnd + 1;
    if (fileType > 3) sectionEnd = strchr(sectionStart, SECTION_SEPARATOR);
    else sectionEnd = sectionStart + strlen(sectionStart);
    fields = 1;
    chunks = stringChunk(sectionStart, sectionEnd, FIELD_SEPARATOR, fields);
    // store in colors list
    robotColors.push_back(rgba(atoi(chunks[0]),
	                       atoi(chunks[1]),
			       atoi(chunks[2])));
    // free chunks which is no longer needed
    removeStorage(chunks, fields);
  }

  file->close();
  delete file;
}
