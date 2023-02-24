#include <count-osm/count-osm.h>

// src
#include <branch_hints.h>
#include <defer.h>
#include <numbers.h>
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

static constexpr uint32_t
protobuf_key(uint32_t field_number, uint8_t wire_type) noexcept
{
  constexpr uint8_t bits_for_wire_type = 3u;
  constexpr uint8_t mask_for_wire_type = ~(0xFFu << bits_for_wire_type) & 0xFFu;
  return (field_number << bits_for_wire_type) |
         (wire_type & mask_for_wire_type);
}

static constexpr uint32_t
protobuf_wire_type(uint32_t key) noexcept
{
  return key & 0x07u;
}

static constexpr uint32_t
protobuf_field_number(uint32_t key) noexcept
{
  return key >> 3;
}

enum pbf_wt : uint32_t
{
  pbf_wt_varint = 0,
  pbf_wt_64 = 1,
  pbf_wt_length_delim = 2,
  pbf_wt_unused1,
  pbf_wt_unused2,
  pbf_wt_32 = 5,
  pbf_wt_unused3,
  pbf_wt_unused4
};

struct pbf_field
{
  uint32_t key = 0; // field and wt
  // pad 4 bytes
  uint64_t varint = 0;
  const uint8_t* pointer = nullptr;
  uint64_t len = 0;

  constexpr pbf_wt wire_type() const noexcept
  {
    return static_cast<pbf_wt>(protobuf_wire_type(key));
  }

  constexpr uint32_t field_number() const noexcept
  {
    return protobuf_field_number(key);
  }
};

bool
read_field(pbf_field* __restrict__ field,
           const char** __restrict__ begin,
           const char* __restrict__ end)
{
  assert(field && "Pass valid values for field");

  cosm::varint_result decoded = cosm::decode_varint_u64(begin, end);
  if (decoded.sts != cosm::varint_status::ok)
  {
    return false;
  }

  field->key = decoded.value;
  field->pointer = reinterpret_cast<const uint8_t*>(*begin);

  switch (field->wire_type())
  {
    case pbf_wt::pbf_wt_varint:
      decoded = cosm::decode_varint_u64(begin, end);
      if (decoded.sts != cosm::varint_status::ok)
      {
        return false;
      }
      field->varint = decoded.value;
      break;
    case pbf_wt::pbf_wt_64:
      field->len = 8;
      *begin += field->len;
      break;
    case pbf_wt::pbf_wt_length_delim:
      decoded = cosm::decode_varint_u64(begin, end);
      if (decoded.sts != cosm::varint_status::ok)
      {
        return false;
      }
      field->pointer = reinterpret_cast<const uint8_t*>(*begin);
      field->len = decoded.value;
      *begin += field->len;
      break;
    case pbf_wt::pbf_wt_32:
      field->len = 4;
      *begin += field->len;
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

// static bool
// skip_length_delim(const char** __restrict__ begin, const char* __restrict__
// end)
// {
//   // read the length as varint
//   if (auto [len, sts] = cosm::decode_varint_u64(begin, end);
//       sts != cosm::varint_status::ok)
//   {
//     return false;
//   }
//   else
//   {
//     // advance by the len
//     begin += len;
//   }
//   return true;
// }

static constexpr uint32_t
KEY_F_WT(uint32_t field_number, uint8_t wire_type)
{
  constexpr uint8_t k_bits_for_wt = 3u;
  constexpr uint8_t k_mask_for_wt = 0b111u;

  return (field_number << k_bits_for_wt) | (wire_type & k_mask_for_wt);
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
        case KEY_F_WT(blob_header_type, pbf_wt_length_delim):
          *type = { reinterpret_cast<const char*>(field.pointer), field.len };
          break;
        case KEY_F_WT(blob_header_datasize, pbf_wt_varint):
          *size = field.varint;
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
                     case KEY_F_WT(relation_id, pbf_wt_varint):
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
  const char* e = ways.end();
  iterate_fields(&b,
                 e,
                 [ways_count](pbf_field& field) -> bool
                 {
                   static constexpr uint32_t way_id = 1;
                   switch (field.key)
                   {
                     case KEY_F_WT(way_id, pbf_wt_varint):
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
  const char* e = dense_nodes.end();
  iterate_fields(
    &b,
    e,
    [nodes_count](pbf_field& field) -> bool
    {
      // static constexpr uint32_t primitivegroup_nodes = 1; // TODO
      static constexpr uint32_t dense_nodes_id = 1;
      switch (field.key)
      {
        case KEY_F_WT(dense_nodes_id, pbf_wt_length_delim):

          *nodes_count += fast_count_dense_ids(
            { reinterpret_cast<const char*>(field.pointer), field.len });

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
  iterate_fields(
    &b,
    e,
    [&cb](pbf_field& field) -> bool
    {
      // static constexpr uint32_t primitivegroup_nodes = 1; // TODO
      static constexpr uint32_t primitivegroup_dense_nodes = 2;
      static constexpr uint32_t primitivegroup_ways = 3;
      static constexpr uint32_t primitivegroup_relations = 4;
      switch (field.key)
      {
        case KEY_F_WT(primitivegroup_dense_nodes, pbf_wt_length_delim):
          cb(entity_state::nodes,
             { reinterpret_cast<const char*>(field.pointer), field.len });
          break;
        case KEY_F_WT(primitivegroup_ways, pbf_wt_length_delim):
          cb(entity_state::ways,
             { reinterpret_cast<const char*>(field.pointer), field.len });
          break;
        case KEY_F_WT(primitivegroup_relations, pbf_wt_length_delim):
          cb(entity_state::relations,
             { reinterpret_cast<const char*>(field.pointer), field.len });
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
  iterate_fields(
    &b,
    e,
    [&cb](pbf_field& field) -> bool
    {
      static constexpr uint32_t primitiveblock_primitivegroup = 2;
      switch (field.key)
      {
        case KEY_F_WT(primitiveblock_primitivegroup, pbf_wt_length_delim):
          cb({ reinterpret_cast<const char*>(field.pointer), field.len });
          break;
        default:
          break;
      }
      return true;
    });

  return true;
}

struct pbf_metadata
{
  uint64_t replication_timestamp;
  uint64_t replication_sequence_no;
  char replication_base_url[256];
};

struct pbf_blob
{
  enum class compression : uint32_t
  {
    none,
    zlib
    // lzma,
    // lz4,
    // zstd
  };
  compression compression_type = compression::none;
  // when no compression, equal to data.size()
  uint32_t raw_size = 0;
  cosm::span<const uint8_t> data;
};

static bool
decode_header_blob(cosm::span<const char> blob_data,
                   pbf_blob* __restrict__ blob) noexcept
{
  const char* b = blob_data.begin();
  iterate_fields(&b,
                 blob_data.end(),
                 [blob](pbf_field& field) -> bool
                 {
                   constexpr uint32_t blob_raw = 1;
                   constexpr uint32_t blob_raw_size = 2;
                   constexpr uint32_t blob_zlib = 3;
                   // do we care about the others? anyone implement those?
                   switch (field.key)
                   {
                     case KEY_F_WT(blob_raw, pbf_wt_length_delim):
                       blob->compression_type = pbf_blob::compression::none;
                       blob->data = { field.pointer, field.len };
                       break;
                     case KEY_F_WT(blob_raw_size, pbf_wt_varint):
                       blob->raw_size = static_cast<uint32_t>(field.varint);
                       break;
                     case KEY_F_WT(blob_zlib, pbf_wt_length_delim):
                       blob->compression_type = pbf_blob::compression::zlib;
                       blob->data = { field.pointer, field.len };
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

struct Counter
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

static read_data_status
read_data(Counter* counter, cosm::span<char> file) noexcept
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

  libdeflate_decompressor* dec = libdeflate_alloc_decompressor();
  if (!dec)
  {
    return read_data_status::e_decompress_create;
  }
  auto free_decompressor =
    cosm::make_defer([dec]() noexcept { libdeflate_free_decompressor(dec); });

  char* buffer_out = static_cast<char*>(aligned_alloc(64, 8 * 1'024 * 1'024));
  if (!buffer_out)
  {
    return read_data_status::e_buffer_allocation;
  }
  auto free_buffer_out =
    cosm::make_defer([buffer_out]() noexcept { free(buffer_out); });

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

    const char* actual_data = reinterpret_cast<const char*>(blob.data.data());
    size_t actual_data_size = blob.data.size();

    if (blob.compression_type == pbf_blob::compression::zlib)
    {
      size_t actual_decompressed_size = blob.raw_size;
      libdeflate_result res =
        libdeflate_zlib_decompress(dec,
                                   blob.data.data(),
                                   blob.data.size(),
                                   buffer_out,
                                   blob.raw_size,
                                   &actual_decompressed_size);

      if (res != libdeflate_result::LIBDEFLATE_SUCCESS)
      {
        return read_data_status::e_decompress_data_blob;
      }

      actual_data = buffer_out;
      actual_data_size = actual_decompressed_size;
    }

    // we now have a PrimitiveBlock inside buffer_out
    decode_primitive_block(
      { actual_data, actual_data_size },
      [counter](cosm::span<const char> primitive_groups)
      {
        // we have a repeated PrimitiveGroup inside
        // primitive_groups
        decode_primitive_groups(
          primitive_groups,
          [counter](entity_state es, cosm::span<const char> data)
          {
            // data contains either dense nodes, ways or
            // relations
            switch (es)
            {
              case entity_state::nodes:
                count_dense_nodes(&counter->nodes, data);
                break;
              case entity_state::ways:
                count_ways(&counter->ways, data);
                break;
              case entity_state::relations:
                count_relations(&counter->relations, data);
                break;
            }
          });
      });
  }

  return read_data_status::ok;
}

// we only allocate memory once, for the whole program, then we use this
// pointer to get chunks
cosm::span<uint8_t> global_memory_pool;
cosm::span<uint8_t> global_scratch_memory;
cosm::span<uint8_t> global_mapped_file_memory;
uint8_t* global_scratch_memory_start = nullptr;

template<size_t Alignment = 8>
[[nodiscard]] uint8_t*
global_alloc(size_t size) noexcept
{
  const size_t aligned_size = (size + Alignment - 1) & ~(Alignment - 1);
  uint8_t* chunk = global_memory_pool;
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

      memory = static_cast<uint8_t*>(mmap(nullptr,
                                          page_aligned_total_size,
                                          PROT_READ | PROT_WRITE,
                                          MAP_PRIVATE | MAP_ANONYMOUS,
                                          -1,
                                          0));
      if (memory == MAP_FAILED)
      {
        return error("mmap global");
      }

      global_memory_pool = cosm::span<uint8_t>(static_cast<uint8_t*>(memory),
                                               page_aligned_total_size);
      global_scratch_memory = cosm::span<uint8_t>(
        static_cast<uint8_t*>(memory) + page_aligned_file_size,
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
      cosm::span<uint8_t>(static_cast<uint8_t*>(mapped_file), mapped_file_size);

    madvise(mapped_file, page_aligned_file_size, MADV_SEQUENTIAL);

    // TODO: do we want to memset the extra padding bytes we added to the
    // end of the mapped file?
  }

  Counter counter;
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