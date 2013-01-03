#ifndef HELPERS_H
#define HELPERS_H

#include <iostream>
using namespace std;

// some message display routines
void echo(const string &msg);
void echo(const string &id, const int value);
void echo(const string &id, const string &value);
void errorExit(string msg);
int rgba(int r, int g, int b);

#endif
