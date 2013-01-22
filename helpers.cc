#include "Aria.h"

#include "helpers.h"

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
