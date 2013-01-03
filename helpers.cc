#include "helpers.h"
#include "Aria.h"

// some message display routines
void echo(const string &msg)
{
  cout << "\t" << msg << endl;
}

void echo(const string &id, const int value)
{
  cout << "\t" << id << " = " << value << endl;
}

void echo(const string &id, const string &value)
{
  cout << "\t" << id << " = " << value << endl;
}

void errorExit(string msg)
{
  echo(msg);
  Aria::shutdown();
  exit(1);
}

// Pack the color information into a single integer which is needed for
// PCL point types.
int rgba(int r, int g, int b) {
  return b + 256*g +256*256*r;
}
