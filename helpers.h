#ifndef HELPERS_H
#define HELPERS_H

#include <iostream>

// some message display routines
void echo(const std::string &msg);
void echo(const std::string &id, const int value);
void echo(const std::string &id, const std::string &value);
void errorExit(std::string msg);
int rgba(int r, int g, int b);

#endif
