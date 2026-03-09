#include "App.h"

#include <iostream>

#include <signal.h>

int main(int argc, char** argv) {
  Application app(argc, argv);

  signal(SIGINT, SIG_DFL);
  return 0;
}
