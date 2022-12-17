#include "status.h"

#include <cstdio>

int
main(int argc, char** argv)
{
  if (argc != 2)
  {
    std::printf("usage: %s %s\n", argv[0], "<path/to/pbf/file>");
    return cosm::EXIT_FAILURE;
  }

  return cosm::EXIT_SUCCESS;
}