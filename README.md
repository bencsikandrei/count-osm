# Count OSM

## Requires

* g++11/clang-13 or higher
* cmake 3.22 or higher
* make or ninja
* (optional) conan - does help a lot

## How to configure

Assumes you have clone the repo and you have `cd`ed into it.

### Conan

```sh
cmake -Sconan -Bbuild/gcc-11-Release -DCMAKE_BUILD_TYPE=Release
```

### No conan (you have to install deps manually)

```sh
cmake -S. -Bbuild/gcc-11-Release -DCMAKE_BUILD_TYPE=Release
```

## Advanced

### Unit tests

```sh
cmake -S. -Bbuild/gcc-11-Release -DCMAKE_BUILD_TYPE=Release
    -DCOUNT_OSM_UNIT_TESTS=ON
```

### Benchmarks

```sh
cmake -S. -Bbuild/gcc-11-Release -DCMAKE_BUILD_TYPE=Release
    -DCOUNT_OSM_BENCHMARKS=ON
```

