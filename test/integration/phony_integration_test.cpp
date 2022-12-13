#include <count-osm/count-osm.h>

// src
#include <defer.h>

// libc
#include <cstdint>
#include <cstdio>

// linux
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

void
usage(const char* exeName) noexcept
{
  printf("Usage:\n\t %s <osm-pbf-file>\n", exeName);
}

int
error(const char* what) noexcept
{
  fprintf(stderr, "error: %s\n", what);
  return 1;
}

int
main(int argc, char** argv)
{
  if (argc < 2)
  {
    usage(argv[0]);
    return 1;
  }

  void* mapped_file = nullptr;
  uint64_t mapped_file_size = 0;

  auto defer_unmap =
    cosm::make_defer([mapped_file, mapped_file_size]() noexcept {
      if (mapped_file)
      {
        munmap(mapped_file, mapped_file_size);
      }
    });

  {
    // open file
    int fd = open(argv[1], O_RDONLY);
    if (fd == -1)
    {
      return error("open");
    }

    auto defer_closefile = cosm::make_defer([fd]() { close(fd); });

    struct stat sb;
    if (fstat(fd, &sb) == -1)
    {
      return error("stat");
    }

    mapped_file_size = sb.st_size;
    mapped_file =
      mmap(nullptr, mapped_file_size, PROT_READ, MAP_PRIVATE, fd, 0);
    if (mapped_file == MAP_FAILED)
    {
      return error("mmap");
    }
  }

  // we now have a mmaped file
  printf("File size in bytes: %lu\n", mapped_file_size);
}