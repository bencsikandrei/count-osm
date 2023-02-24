#include <count-osm/count-osm.h>

// src
#include <branch_hints.h>
#include <defer.h>
#include <numbers.h>
#include <protobuf.h>
#include <span.h>

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
#include <condition_variable>
#include <mutex>
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

struct pbf_field
{
  union VarintOrLenDelim
  {
    constexpr VarintOrLenDelim() noexcept
      : varint(0)
    {
    }
    uint64_t varint = 0;
    cosm::span<const char> length_delim;
  };

  uint32_t key = 0; // field and wt
  // pad 4 bytes
  VarintOrLenDelim data;

  constexpr cosm::pbf_wt wire_type() const noexcept
  {
    return static_cast<cosm::pbf_wt>(cosm::protobuf_wire_type(key));
  }

  constexpr uint32_t field_number() const noexcept
  {
    return cosm::protobuf_field_number(key);
  }
};

bool
read_field(pbf_field* __restrict__ field,
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

template<typename Callback>
static bool
iterate_fields(const char** begin, const char* end, Callback&& cb) noexcept
{
  pbf_field field;
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
  const char* e = header.end();
  iterate_fields(
    &b,
    e,
    [type, size](pbf_field& field) -> bool
    {
      constexpr uint32_t blob_header_type = 1;
      constexpr uint32_t blob_header_datasize = 3;
      switch (field.key)
      {
        case cosm::protobuf_key(blob_header_type, cosm::pbf_wt_length_delim):
          *type = field.data.length_delim;
          break;
        case cosm::protobuf_key(blob_header_datasize, cosm::pbf_wt_varint):
          *size = field.data.varint;
          break;
        default:
          break;
      }
      return true;
    });

  return true;
}

enum class entity_state
{
  nodes = 0,
  ways = 1,
  relations = 2
};

static constexpr const char*
entity_state_string(entity_state es) noexcept
{
  switch (es)
  {
    case entity_state::nodes:
      return "nodes";
    case entity_state::ways:
      return "ways";
    case entity_state::relations:
      return "relations";
  }

  return "error!";
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
count_relations(size_t* relations_count, cosm::span<const char> relations)
{
  const char* b = relations.begin();
  iterate_fields(&b,
                 relations.end(),
                 [relations_count](pbf_field& field) -> bool
                 {
                   static constexpr uint32_t relation_id = 1;
                   switch (field.key)
                   {
                     case cosm::protobuf_key(relation_id, cosm::pbf_wt_varint):
                       ++(*relations_count);
                       break;
                     default:
                       break;
                   }
                   return true;
                 });
  return true;
}

static bool
count_ways(size_t* ways_count, cosm::span<const char> ways)
{
  const char* b = ways.begin();
  iterate_fields(&b,
                 ways.end(),
                 [ways_count](pbf_field& field) -> bool
                 {
                   static constexpr uint32_t way_id = 1;
                   switch (field.key)
                   {
                     case cosm::protobuf_key(way_id, cosm::pbf_wt_varint):
                       ++(*ways_count);
                       break;
                     default:
                       break;
                   }
                   return true;
                 });
  return true;
}

static bool
count_dense_nodes(size_t* nodes_count, cosm::span<const char> dense_nodes)
{
  const char* b = dense_nodes.begin();
  iterate_fields(
    &b,
    dense_nodes.end(),
    [nodes_count](pbf_field& field) -> bool
    {
      // static constexpr uint32_t primitivegroup_nodes = 1; //
      // TODO
      static constexpr uint32_t dense_nodes_id = 1;
      switch (field.key)
      {
        case cosm::protobuf_key(dense_nodes_id, cosm::pbf_wt_length_delim):
          *nodes_count += fast_count_dense_ids(field.data.length_delim);
          break;
        default:
          break;
      }
      return true;
    });

  return true;
}

template<typename Callback>
static bool
decode_primitive_groups(cosm::span<const char> primitive_groups, Callback&& cb)
{
  const char* b = primitive_groups.begin();
  const char* e = primitive_groups.end();
  iterate_fields(&b,
                 e,
                 [&cb](pbf_field& field) -> bool
                 {
                   // static constexpr uint32_t primitivegroup_nodes = 1; //
                   // TODO
                   static constexpr uint32_t primitivegroup_dense_nodes = 2;
                   static constexpr uint32_t primitivegroup_ways = 3;
                   static constexpr uint32_t primitivegroup_relations = 4;
                   switch (field.key)
                   {
                     case cosm::protobuf_key(primitivegroup_dense_nodes,
                                             cosm::pbf_wt_length_delim):
                       cb(entity_state::nodes, field.data.length_delim);
                       break;
                     case cosm::protobuf_key(primitivegroup_ways,
                                             cosm::pbf_wt_length_delim):
                       cb(entity_state::ways, field.data.length_delim);
                       break;
                     case cosm::protobuf_key(primitivegroup_relations,
                                             cosm::pbf_wt_length_delim):
                       cb(entity_state::relations, field.data.length_delim);
                       break;
                     default:
                       break;
                   }
                   return true;
                 });

  return true;
}

template<typename Callback>
static bool
decode_primitive_block(cosm::span<const char> primitive_block,
                       Callback&& cb) noexcept
{
  const char* b = primitive_block.begin();
  const char* e = primitive_block.end();
  iterate_fields(&b,
                 e,
                 [&cb](pbf_field& field) -> bool
                 {
                   static constexpr uint32_t primitiveblock_primitivegroup = 2;
                   switch (field.key)
                   {
                     case cosm::protobuf_key(primitiveblock_primitivegroup,
                                             cosm::pbf_wt_length_delim):
                       cb(field.data.length_delim);
                       break;
                     default:
                       break;
                   }
                   return true;
                 });

  return true;
}

struct pbf_blob
{
  enum class compression : uint32_t
  {
    none,
    zlib
  };
  compression compression_type = compression::none;
  // when no compression, equal to data.size()
  uint32_t raw_size = 0;
  cosm::span<const char> data;
};

static bool
decode_header_blob(cosm::span<const char> blob_data,
                   pbf_blob* __restrict__ blob) noexcept
{
  const char* b = blob_data.begin();
  iterate_fields(
    &b,
    blob_data.end(),
    [blob](pbf_field& field) -> bool
    {
      constexpr uint32_t blob_raw = 1;
      constexpr uint32_t blob_raw_size = 2;
      constexpr uint32_t blob_zlib = 3;
      // do we care about the others? anyone implement those?
      switch (field.key)
      {
        case cosm::protobuf_key(blob_raw, cosm::pbf_wt_length_delim):
          blob->compression_type = pbf_blob::compression::none;
          blob->data = field.data.length_delim;
          break;
        case cosm::protobuf_key(blob_raw_size, cosm::pbf_wt_varint):
          blob->raw_size = static_cast<uint32_t>(field.data.varint);
          break;
        case cosm::protobuf_key(blob_zlib, cosm::pbf_wt_length_delim):
          blob->compression_type = pbf_blob::compression::zlib;
          blob->data = field.data.length_delim;
          break;
        default:
          break;
      }

      return true;
    });

  // let's unify the compressed and uncompressed data
  if (blob->compression_type == pbf_blob::compression::none)
  {
    blob->raw_size = static_cast<uint32_t>(blob->data.size());
  }

  return true;
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
  e_decompress_unuexpected_size
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
  }
  return "unknown error!";
}

struct alignas(64) worker
{
  ~worker() noexcept
  {
    free(buffer);
    buffer = nullptr;
  }

  char* buffer = nullptr;
  osm_counter counter;
};

struct work_item
{
  const char* data = nullptr;
  size_t size = 0;
  int32_t raw_size = 0;
  pbf_blob::compression compression_type;
};

alignas(64) worker thread_workers[12];
alignas(64) std::thread thread_threads[12];
alignas(64) std::queue<work_item> thread_queue;
alignas(64) std::mutex thread_mutex;
alignas(64) std::condition_variable thread_cv;
static constexpr size_t decompress_buffer_size = 8 * 1'024 * 1'024;

void
do_work(int thread_index)
{
  libdeflate_decompressor* dec = libdeflate_alloc_decompressor();
  if (!dec)
  {
    return;
  }
  auto free_decompressor = cosm::make_defer(
    [dec, thread_index]() noexcept { libdeflate_free_decompressor(dec); });

  constexpr int max_wi_per_pop = 1;
  while (true)
  {
    work_item wi[max_wi_per_pop];
    int actual_work_items = 0;
    {
      std::unique_lock<std::mutex> lck(thread_mutex);

      if (!thread_cv.wait_for(lck,
                              std::chrono::microseconds(10),
                              []() { return !thread_queue.empty(); }))
      {
        continue;
      }

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
        return;
      }

      const char* actual_data = reinterpret_cast<const char*>(wi[i].data);
      size_t actual_data_size = wi[i].size;

      if (wi[i].compression_type == pbf_blob::compression::zlib)
      {
        size_t actual_decompressed_size = decompress_buffer_size;
        libdeflate_result res =
          libdeflate_zlib_decompress(dec,
                                     actual_data,
                                     actual_data_size,
                                     thread_workers[thread_index].buffer,
                                     actual_decompressed_size,
                                     &actual_decompressed_size);

        if (res != libdeflate_result::LIBDEFLATE_SUCCESS)
        {
          return;
        }

        actual_data = thread_workers[thread_index].buffer;
        actual_data_size = actual_decompressed_size;
      }

      // we now have a PrimitiveBlock inside buffer_out
      decode_primitive_block(
        { actual_data, actual_data_size },
        [thread_index](cosm::span<const char> primitive_groups)
        {
          // we have a repeated PrimitiveGroup inside
          // primitive_groups
          decode_primitive_groups(
            primitive_groups,
            [thread_index](entity_state es, cosm::span<const char> data)
            {
              // data contains either dense nodes, ways or
              // relations
              switch (es)
              {
                case entity_state::nodes:
                  count_dense_nodes(&thread_workers[thread_index].counter.nodes,
                                    data);
                  break;
                case entity_state::ways:
                  count_ways(&thread_workers[thread_index].counter.ways, data);
                  break;
                case entity_state::relations:
                  count_relations(
                    &thread_workers[thread_index].counter.relations, data);
                  break;
              }
            });
        });
    }
  }
}

static read_data_status
read_data(osm_counter* counter, cosm::span<char> file) noexcept
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

  pbf_blob blob;
  if (!decode_header_blob({ p, static_cast<uint64_t>(next_blob_size) }, &blob))
  {
    return read_data_status::e_decode_header_blob;
  }

  p += (header_size + next_blob_size);

  // after the first `OSMHeader` we will get only `OSMData`
  // every entity is inside a so called `blob` which is also pbf encoded (see
  // fileformat.proto

  for (worker& w : thread_workers)
  {
    w.buffer = static_cast<char*>(aligned_alloc(64, decompress_buffer_size));
    if (!w.buffer)
    {
      return read_data_status::e_buffer_allocation;
    }
  }

  int thread_index = 0;
  for (std::thread& t : thread_threads)
  {
    t = std::thread(do_work, thread_index++);
  }

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
      std::lock_guard<std::mutex> l(thread_mutex);
      thread_queue.push(
        work_item{ .data = reinterpret_cast<const char*>(blob.data.data()),
                   .size = blob.data.size(),
                   .compression_type = blob.compression_type });
    }
    thread_cv.notify_one();
  }

  {
    std::lock_guard<std::mutex> lck(thread_mutex);
    for (int i = 0; i < 48; ++i)
    {
      // launch 12 stop tokens
      thread_queue.push(work_item{ .data = nullptr });
    }
  }

  for (int i = 0; i < 48; ++i)
  {
    thread_cv.notify_one();
  }

  for (auto& t : thread_threads)
  {
    if (t.joinable())
    {
      t.join();
    }
  }

  for (auto& w : thread_workers)
  {
    counter->nodes += w.counter.nodes;
    counter->ways += w.counter.ways;
    counter->relations += w.counter.relations;
  }

  return read_data_status::ok;
}

// we only allocate memory once, for the whole program, then we use this
// pointer to get chunks
cosm::span<char> global_memory_pool;
cosm::span<char> global_scratch_memory;
cosm::span<char> global_mapped_file_memory;
char* global_scratch_memory_start = nullptr;

template<size_t Alignment = 8>
[[nodiscard]] char*
global_alloc(size_t size) noexcept
{
  const size_t aligned_size = (size + Alignment - 1) & ~(Alignment - 1);
  char* chunk = global_memory_pool;
  global_scratch_memory_start += aligned_size;
  return chunk;
}

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

  auto defer_unmap = cosm::make_defer(
    []() noexcept
    {
      if (global_memory_pool.data())
      {
        munmap(global_memory_pool.data(), global_memory_pool.m_size);
      }
    });

  size_t mapped_file_size = 0;

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
                        MAP_PRIVATE | MAP_ANONYMOUS | MAP_HUGETLB,
                        -1,
                        0);
    if (memory == MAP_FAILED)
    {
      puts("W: could not allocate huge pages for mmap");

      memory = static_cast<char*>(mmap(nullptr,
                                       page_aligned_total_size,
                                       PROT_READ | PROT_WRITE,
                                       MAP_PRIVATE | MAP_ANONYMOUS,
                                       -1,
                                       0));
      if (memory == MAP_FAILED)
      {
        return error("mmap global");
      }

      global_memory_pool =
        cosm::span<char>(static_cast<char*>(memory), page_aligned_total_size);
      global_scratch_memory =
        cosm::span<char>(static_cast<char*>(memory) + page_aligned_file_size,
                         page_aligned_scratch_space);
      global_scratch_memory_start = global_scratch_memory.data();
    }

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

  osm_counter counter;
  if (const read_data_status sts =
        read_data(&counter,
                  cosm::span<char>(
                    reinterpret_cast<char*>(global_mapped_file_memory.data()),
                    global_mapped_file_memory.m_size));
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
