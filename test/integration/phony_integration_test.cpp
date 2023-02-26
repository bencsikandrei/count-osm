#include <count-osm/count-osm.h>

// src
#include <allocator.h>
#include <branch_hints.h>
#include <defer.h>
#include <lock_guard.h>
#include <move.h>
#include <mutex.h>
#include <numbers.h>
#include <osm_pbf.h>
#include <protobuf.h>
#include <span.h>
#include <thread.h>

// third party
#include <libdeflate.h>

// libc
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

// linux
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

// simd
#include <immintrin.h>

// don't want to add STL stuff, but here we go
#include <queue>
#include <thread>

#define COSM_DONT_DEBUG

template<typename What>
static void
debug_only([[maybe_unused]] What&& what)
{
#ifndef COSM_DONT_DEBUG
  what();
#endif // COSM_DONT_DEBUG
}

#define COSM_DEBUG(code)                                                       \
  debug_only(                                                                  \
    [&]()                                                                      \
    {                                                                          \
      do                                                                       \
      {                                                                        \
        code                                                                   \
      } while (false);                                                         \
    });

static void
usage(const char* exeName) noexcept
{
  printf("Usage:\n\t %s <osm-pbf-file>\n", exeName);
}

static int
error(const char* what) noexcept
{
  fprintf(stderr, "error: %s\n", what);
  return 1;
}

bool
read_field(cosm::pbf_field* __restrict__ field,
           const char** __restrict__ begin,
           const char* __restrict__ end)
{
  assert(field && "Pass valid values for field");
  assert(begin && "Pass valid values for begin");
  assert(end && "Pass valid values for end");

  cosm::varint_result decoded = cosm::decode_varint_u64(begin, end);
  if (decoded.sts != cosm::varint_status::ok)
  {
    return false;
  }

  field->key = decoded.value;

  switch (field->wire_type())
  {
    case cosm::pbf_wt::pbf_wt_varint:
      decoded = cosm::decode_varint_u64(begin, end);
      if (decoded.sts != cosm::varint_status::ok)
      {
        return false;
      }
      field->data.varint = decoded.value;
      break;
    case cosm::pbf_wt::pbf_wt_length_delim:
      decoded = cosm::decode_varint_u64(begin, end);
      if (decoded.sts != cosm::varint_status::ok)
      {
        return false;
      }
      field->data.length_delim =
        cosm::span<const char>{ reinterpret_cast<const char*>(*begin),
                                decoded.value };
      *begin += field->data.length_delim.size();
      break;
    default:
      // we don't care about these, if they appear we could process them
      return false;
  }

  return true;
}

template<typename callback_t>
static bool
iterate_fields(const char** begin, const char* end, callback_t&& cb) noexcept
{
  assert(begin && "Pass valid values for begin");
  assert(end && "Pass valid values for end");

  cosm::pbf_field field;
  while (*begin < end)
  {
    if (!read_field(&field, begin, end) || !cb(field))
    {
      return false;
    }
  }
  return true;
}

static bool
decode_file_header(cosm::span<const char> header,
                   cosm::span<const char>* __restrict__ type,
                   int32_t* __restrict__ size) noexcept
{
  const char* b = header.begin();
  return iterate_fields(
    &b,
    header.end(),
    [type, size](cosm::pbf_field& field) -> bool
    {
      constexpr uint32_t blob_header_type = 1;
      constexpr uint32_t blob_header_datasize = 3;
      switch (field.key)
      {
        case cosm::protobuf_key(blob_header_type, cosm::pbf_wt_length_delim):
          assert(type);
          *type = field.data.length_delim;
          break;
        case cosm::protobuf_key(blob_header_datasize, cosm::pbf_wt_varint):
          assert(size);
          *size = field.data.varint;
          break;
        default:
          break;
      }
      return true;
    });
}

size_t
fast_count_dense_ids(cosm::span<const char> dense_nodes_id)
{
  size_t count = 0;
  // we need to count the number of varints, we can thus count the bytes that
  // have the 8th bit set to 0 (1 means there's still bytes in that number)

  const char* p = dense_nodes_id.begin();
  const char* e = dense_nodes_id.end();

#ifdef __AVX2__
  constexpr int64_t unroll_factor = 32;
  const int64_t unrolled_elements = dense_nodes_id.size() / unroll_factor;
  const __m256i _32_0x80 = _mm256_set1_epi8(-128);
  for (int64_t i = 0; i < unrolled_elements; ++i)
  {
    __m256i chars = _mm256_loadu_si256(reinterpret_cast<const __m256i*>(p));
    __m256i chars_and_0x80 = _mm256_and_si256(chars, _32_0x80);
    unsigned mask = _mm256_movemask_epi8(chars_and_0x80);
    count += __builtin_popcount(~mask);
    p += unroll_factor;
  }
#endif

  for (; p != e; ++p)
  {
    count += ((*p) > 0);
  }

  return count;
}

static bool
count_dense_nodes(size_t* nodes_count, cosm::span<const char> dense_nodes)
{
  const char* b = dense_nodes.begin();
  return iterate_fields(
    &b,
    dense_nodes.end(),
    [nodes_count](cosm::pbf_field& field) -> bool
    {
      static constexpr uint32_t dense_nodes_id = 1;
      if (field.key ==
          cosm::protobuf_key(dense_nodes_id, cosm::pbf_wt_length_delim))
      {
        *nodes_count += fast_count_dense_ids(field.data.length_delim);
      }
      return true;
    });
}

template<typename callback_t>
static bool
decode_primitive_groups(cosm::span<const char> primitive_groups,
                        callback_t&& cb)
{
  const char* b = primitive_groups.begin();
  return iterate_fields(
    &b,
    primitive_groups.end(),
    [&cb](cosm::pbf_field& field) -> bool
    {
      // TODO: do we need the 'nodes' field = 1?
      static constexpr uint32_t primitivegroup_dense_nodes = 2;
      static constexpr uint32_t primitivegroup_ways = 3;
      static constexpr uint32_t primitivegroup_relations = 4;
      switch (field.key)
      {
        case cosm::protobuf_key(primitivegroup_dense_nodes,
                                cosm::pbf_wt_length_delim):
          cb(cosm::entity_state::nodes, field.data.length_delim);
          break;
        case cosm::protobuf_key(primitivegroup_ways, cosm::pbf_wt_length_delim):
          cb(cosm::entity_state::ways, field.data.length_delim);
          break;
        case cosm::protobuf_key(primitivegroup_relations,
                                cosm::pbf_wt_length_delim):
          cb(cosm::entity_state::relations, field.data.length_delim);
          break;
        default:
          break;
      }
      return true;
    });
}

template<typename callback_t>
static bool
decode_primitive_block(cosm::span<const char> primitive_block,
                       callback_t&& cb) noexcept
{
  const char* b = primitive_block.begin();
  return iterate_fields(
    &b,
    primitive_block.end(),
    [&cb](cosm::pbf_field& field) -> bool
    {
      static constexpr uint32_t primitiveblock_primitivegroup = 2;
      if (field.key == cosm::protobuf_key(primitiveblock_primitivegroup,
                                          cosm::pbf_wt_length_delim))
      {
        cb(field.data.length_delim);
      }
      return true;
    });
}

static bool
decode_header_blob(cosm::span<const char> blob_data,
                   cosm::pbf_blob* __restrict__ blob) noexcept
{
  const char* b = blob_data.begin();
  const bool res = iterate_fields(
    &b,
    blob_data.end(),
    [blob](cosm::pbf_field& field) -> bool
    {
      // these are not always in this order!
      constexpr uint32_t blob_raw = 1;
      constexpr uint32_t blob_raw_size = 2;
      constexpr uint32_t blob_zlib = 3;
      // do we care about the others? anyone implement those?
      switch (field.key)
      {
        case cosm::protobuf_key(blob_raw, cosm::pbf_wt_length_delim):
          blob->compression_type = cosm::pbf_blob::compression::none;
          blob->data = field.data.length_delim;
          break;
        case cosm::protobuf_key(blob_raw_size, cosm::pbf_wt_varint):
          blob->raw_size = static_cast<uint32_t>(field.data.varint);
          break;
        case cosm::protobuf_key(blob_zlib, cosm::pbf_wt_length_delim):
          blob->compression_type = cosm::pbf_blob::compression::zlib;
          blob->data = field.data.length_delim;
          break;
        default:
          break;
      }
      return true;
    });

  // let's unify the compressed and uncompressed data
  if (blob->compression_type == cosm::pbf_blob::compression::none)
  {
    blob->raw_size = static_cast<uint32_t>(blob->data.size());
  }

  return res;
}

struct osm_counter
{
  size_t nodes = 0;
  size_t ways = 0;
  size_t relations = 0;
};

enum class read_data_status
{
  ok = 0,
  e_decode_file_header,
  e_decode_header_blob,
  e_decompress_create,
  e_buffer_allocation,
  e_decode_data_file_header,
  e_decode_data_header_blob,
  e_decompress_data_blob,
  e_decompress_unuexpected_size,
  e_not_enough_memory,
  e_thread_creation_failed,
  e_thread_join_failed
};

constexpr const char*
read_data_status_string(read_data_status sts) noexcept
{
  switch (sts)
  {
    case read_data_status::ok:
      return "ok";
    case read_data_status::e_decode_file_header:
      return "e_decode_file_header";
    case read_data_status::e_decode_header_blob:
      return "e_decode_header_blob";
    case read_data_status::e_decompress_create:
      return "e_decompress_create";
    case read_data_status::e_buffer_allocation:
      return "e_buffer_allocation";
    case read_data_status::e_decode_data_file_header:
      return "e_decode_data_file_header";
    case read_data_status::e_decode_data_header_blob:
      return "e_decode_data_header_blob";
    case read_data_status::e_decompress_data_blob:
      return "e_decompress_data_blob";
    case read_data_status::e_decompress_unuexpected_size:
      return "e_decompress_unuexpected_size";
    case read_data_status::e_not_enough_memory:
      return "e_not_enough_memory";
    case read_data_status::e_thread_creation_failed:
      return "e_thread_creation_failed";
    case read_data_status::e_thread_join_failed:
      return "e_thread_join_failed";
  }
  return "unknown error!";
}

struct alignas(64) worker
{
  int thread_index = 0;
  cosm::thread thread;
  libdeflate_decompressor* thread_decompressor = nullptr;
  char* thread_buffer = nullptr;
  osm_counter thread_counter;
};

static_assert(sizeof(worker) == 64);

struct work_item
{
  const char* data = nullptr;
  // we know that these are smaller than 2GB
  uint32_t raw_size = 0;
  // raw size is (almost) always larger than the compressed size
  uint32_t size : 31 = 0;
  uint32_t compression_type : 1 = 0;
};

static_assert(sizeof(work_item) == 16);

alignas(64) worker thread_workers[12];
alignas(64) std::thread thread_threads[12];
alignas(64) std::queue<work_item> thread_queue;
alignas(64) cosm::mutex thread_mutex;
static constexpr size_t decompress_buffer_size = 8 * 1'024 * 1'024;

void*
do_work(void* arg)
{
  worker* w = static_cast<worker*>(arg);

  // ALL worker fields are initialized before this call!

  // the only thing that the thread owns is the decompressor, we must free it
  auto free_decompressor = cosm::make_defer(
    [w]() noexcept
    {
      libdeflate_free_decompressor(w->thread_decompressor);
      w->thread_decompressor = nullptr;
    });

  constexpr int max_wi_per_pop = 4;
  while (true)
  {
    work_item wi[max_wi_per_pop];
    int actual_work_items = 0;
    {
      cosm::lock_guard<cosm::mutex> lck(thread_mutex);
      while (!thread_queue.empty() && actual_work_items < max_wi_per_pop)
      {
        wi[actual_work_items++] = thread_queue.front();
        thread_queue.pop();
      }
    }

    for (int i = 0; i < actual_work_items; ++i)
    {
      if (wi[i].data == nullptr)
      {
        // stop token
        return arg;
      }

      const char* actual_data = reinterpret_cast<const char*>(wi[i].data);
      size_t actual_data_size = wi[i].size;

      if (wi[i].compression_type ==
          static_cast<uint32_t>(cosm::pbf_blob::compression::zlib))
      {
        size_t actual_decompressed_size = decompress_buffer_size;
        libdeflate_result res =
          libdeflate_zlib_decompress(w->thread_decompressor,
                                     actual_data,
                                     actual_data_size,
                                     w->thread_buffer,
                                     actual_decompressed_size,
                                     &actual_decompressed_size);

        if (res != libdeflate_result::LIBDEFLATE_SUCCESS)
        {
          return arg;
        }

        actual_data = w->thread_buffer;
        actual_data_size = actual_decompressed_size;
      }

      // we now have a PrimitiveBlock inside buffer_out
      decode_primitive_block(
        { actual_data, actual_data_size },
        [w](cosm::span<const char> primitive_groups)
        {
          // we have a repeated PrimitiveGroup inside
          // primitive_groups
          decode_primitive_groups(
            primitive_groups,
            [w](cosm::entity_state es, cosm::span<const char> data)
            {
              // data contains either dense nodes, ways or
              // relations
              switch (es)
              {
                case cosm::entity_state::nodes:
                  // nodes are in a dense array
                  count_dense_nodes(&w->thread_counter.nodes, data);
                  break;
                  // ways and relations are in length delimited
                case cosm::entity_state::ways:
                  ++(w->thread_counter.ways);
                  break;
                  // ways and relations are in length delimited
                case cosm::entity_state::relations:
                  ++(w->thread_counter.relations);
                  break;
              }
            });
        });
    }
  }

  return arg;
}

static constexpr int k_threads_max = 128;

static read_data_status
read_data(osm_counter* counter,
          cosm::span<char> file,
          int number_of_threads,
          cosm::bump_allocator allocator) noexcept
{
  const char* p = file.data();
  [[maybe_unused]] const char* end = file.data() + file.size();

  [[maybe_unused]] uint32_t header_size =
    cosm::read_network_uint32_unchecked(p);
  p += 4;

  assert(p <= end && "There's at least one header in a file");
  assert(header_size > 0 && "Headers can't be empty");

  cosm::span<const char> next_blob_type;
  // the first file header has to contain data about `OSMHeader` (one per file,
  // at start)
  int32_t next_blob_size = 0;
  if (!decode_file_header({ p, header_size }, &next_blob_type, &next_blob_size))
  {
    return read_data_status::e_decode_file_header;
  }

  cosm::pbf_blob blob;
  if (!decode_header_blob({ p, static_cast<uint64_t>(next_blob_size) }, &blob))
  {
    return read_data_status::e_decode_header_blob;
  }

  p += (header_size + next_blob_size);

  const int number_of_threads_adjusted =
    number_of_threads < 1
      ? 1
      : (number_of_threads > k_threads_max ? k_threads_max : number_of_threads);

  // we allocate max threads, even if we don't use that much
  worker* workers = allocator.allocate<worker>(k_threads_max, 64);
  if (!workers)
  {
    return read_data_status::e_not_enough_memory;
  }

  // TODO: find a way to use mmaped area for libdeflate also
  // libdeflate_set_memory_allocator(void *(*malloc_func)(size_t),
  // 			void (*free_func)(void *));

  // TODO: make sure we cleanup
  for (int i = 0; i < number_of_threads_adjusted; ++i)
  {
    workers[i].thread_index = i;
    workers[i].thread_buffer =
      static_cast<char*>(allocator.allocate_raw(decompress_buffer_size, 64));
    if (!workers[i].thread_buffer)
    {
      return read_data_status::e_buffer_allocation;
    }
    if (workers[i].thread.start(&do_work, &workers[i]))
    {
      return read_data_status::e_thread_creation_failed;
    }
    workers[i].thread_decompressor = libdeflate_alloc_decompressor();
    if (!workers[i].thread_decompressor)
    {
      // each thread owns its own decompressor and frees it when done
      return read_data_status::e_decompress_create;
    }
  }

  // after the first `OSMHeader` we will get only `OSMData`
  // every entity is inside a so called `blob` which is also
  // pbf encoded (see fileformat.proto

  while (p < end)
  {
    header_size = cosm::read_network_uint32_unchecked(p);
    p += 4;

    if (!decode_file_header(
          { p, header_size }, &next_blob_type, &next_blob_size))
    {
      return read_data_status::e_decode_data_file_header;
    }

    if (!decode_header_blob({ p, static_cast<uint64_t>(next_blob_size) },
                            &blob))
    {
      return read_data_status::e_decode_data_header_blob;
    }

    p += (header_size + next_blob_size);

    {
      cosm::lock_guard<cosm::mutex> lk(thread_mutex);
      thread_queue.push(work_item{
        .data = reinterpret_cast<const char*>(blob.data.data()),
        .size = static_cast<uint32_t>(blob.data.size()),
        .compression_type = static_cast<uint32_t>(blob.compression_type) });
    }
  }

  {
    cosm::lock_guard<cosm::mutex> lck(thread_mutex);
    for (int i = 0; i < number_of_threads_adjusted * 4; ++i)
    {
      thread_queue.push(work_item{ .data = nullptr });
    }
  }

  for (int i = 0; i < number_of_threads_adjusted; ++i)
  {
    if (workers[i].thread.joinable())
    {
      if (workers[i].thread.join())
      {
        return read_data_status::e_thread_join_failed;
        // should abort?
      }
    }
    counter->nodes += workers[i].thread_counter.nodes;
    counter->ways += workers[i].thread_counter.ways;
    counter->relations += workers[i].thread_counter.relations;
  }

  return read_data_status::ok;
}

// we only allocate memory once, for the whole program, then we use this
// pointer to get chunks
cosm::span<char> global_memory_pool;
cosm::span<char> global_mapped_file_memory;
char* global_scratch_memory_start = nullptr;

// let's try to use huge pages first (linux says 2MB)
constexpr size_t huge_page_size = 2ul * 1'024ul * 1'024ul;
// request at least 4GB of space for our own use
constexpr size_t page_aligned_scratch_space =
  (4ul * 1'024ul * 1'024ul * 1'024ul);
static_assert(page_aligned_scratch_space % huge_page_size == 0);

int
main(int argc, char** argv)
{
  // TODO add some sort of measurement tool to see how much memory we consume

  if (argc < 2)
  {
    usage(argv[0]);
    return 1;
  }

  int user_defined_number_of_threads = cosm::thread_get_hardware_concurrency();
  if (argc >= 3)
  {
    int temporary = cosm::atoi(argv[2]);
    if (temporary >= 1)
    {
      user_defined_number_of_threads = temporary;
      printf("Using %d/%ld threads, as specified by the user\n",
             user_defined_number_of_threads,
             cosm::thread_get_hardware_concurrency());
    }
  }
  else
  {
    printf("Using %d/%d threads\n",
           user_defined_number_of_threads,
           user_defined_number_of_threads);
  }

  auto defer_unmap = cosm::make_defer(
    []() noexcept
    {
      if (global_memory_pool.data())
      {
        munmap(global_memory_pool.data(), global_memory_pool.m_size);
      }
    });

  size_t mapped_file_size = 0;

  void* scratch_memory_start = nullptr;
  size_t scratch_memory_size = 0;

  {
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

    // we can align at 2MB, instead of the usual 4KB, not much of an issue
    // if we overuse 1.9MB :)
    const size_t page_aligned_file_size =
      ((sb.st_size + 2ul * huge_page_size) / huge_page_size) * huge_page_size;

    // total bytes, including our scratch space
    size_t page_aligned_total_size =
      page_aligned_file_size + page_aligned_scratch_space;

    // try to map with huge pages
    void* memory = mmap(nullptr,
                        page_aligned_total_size,
                        PROT_READ | PROT_WRITE,
                        MAP_PRIVATE | MAP_ANONYMOUS,
                        -1,
                        0);
    if (memory == MAP_FAILED)
    {
      return error("mmap global");
    }

    global_memory_pool =
      cosm::span<char>(static_cast<char*>(memory), page_aligned_total_size);

    scratch_memory_start = static_cast<char*>(memory) + page_aligned_file_size;
    scratch_memory_size = page_aligned_scratch_space;

    void* mapped_file = mmap(global_memory_pool.data(),
                             page_aligned_file_size,
                             PROT_READ | PROT_WRITE,
                             MAP_FIXED | MAP_PRIVATE,
                             fd,
                             0);
    if (mapped_file == MAP_FAILED)
    {
      return error("mmap file");
    }

    global_mapped_file_memory =
      cosm::span<char>(static_cast<char*>(mapped_file), mapped_file_size);

    madvise(mapped_file, page_aligned_file_size, MADV_SEQUENTIAL);

    // TODO: do we want to memset the extra padding bytes we added to the
    // end of the mapped file?
  }

  cosm::bump_allocator allocator(scratch_memory_start, scratch_memory_size);

  osm_counter counter;
  if (const read_data_status sts =
        read_data(&counter,
                  cosm::span<char>(
                    reinterpret_cast<char*>(global_mapped_file_memory.data()),
                    global_mapped_file_memory.m_size),
                  user_defined_number_of_threads,
                  cosm::move(allocator));
      sts != read_data_status::ok)
  {
    printf("E: %s\n", read_data_status_string(sts));
    return error("read_data");
  }

  puts("Stats:");
  // we now have a mmaped file
  printf("File size in bytes: %lu\n", mapped_file_size);
  printf("Nodes: %lu\nWays: %lu\nRelations: %lu\n",
         counter.nodes,
         counter.ways,
         counter.relations);
}
