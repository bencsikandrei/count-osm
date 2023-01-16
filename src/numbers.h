#ifndef COSM_NUMBERS_H__
#define COSM_NUMBERS_H__

#include "branch_hints.h"
#include "span.h"

// TODO get rid of cstdint and use something we make
#include <cassert>
#include <cstdint>

namespace cosm
{

enum class varint_status : uint8_t
{
  ok = 0,
  too_many_bytes,
  too_few_bytes
};

template<typename T = uint64_t>
struct varint_result
{
  T value;
  varint_status sts;
};

inline varint_result<uint64_t>
decode_varint_u64(const char** __restrict__ begin, const char* __restrict__ end)
{
  static constexpr int32_t k_max_varint_length = 10;
  static constexpr uint8_t varint_last_bit_mask = 0x7FU;

  assert(begin && "Don't pass null as the begin address");
  assert(*begin && "Don't pass null as begin");
  assert(end && "Don't pass null as end");

  varint_result<uint64_t> result{ .value = 0, .sts = varint_status::ok };

  const int8_t* b = reinterpret_cast<const int8_t*>(*begin);
  const int8_t* e = reinterpret_cast<const int8_t*>(end);
  const int8_t* p = b;

  // taken from https://github.com/facebook/folly/blob/main/folly/Varint.h
  // and here:
  // https://github.com/mapbox/protozero/blob/master/include/protozero/varint.hpp
  if (COSM_LIKELY((e - b) >= k_max_varint_length))
  {
    do
    {
      int64_t b = *p++;
      result.value = ((uint64_t(b) & varint_last_bit_mask));
      if (b >= 0)
      {
        break;
      }
      b = *p++;
      result.value |= ((uint64_t(b) & varint_last_bit_mask) << 7U);
      if (b >= 0)
      {
        break;
      }
      b = *p++;
      result.value |= ((uint64_t(b) & varint_last_bit_mask) << 14U);
      if (b >= 0)
      {
        break;
      }
      b = *p++;
      result.value |= ((uint64_t(b) & varint_last_bit_mask) << 21U);
      if (b >= 0)
      {
        break;
      }
      b = *p++;
      result.value |= ((uint64_t(b) & varint_last_bit_mask) << 28U);
      if (b >= 0)
      {
        break;
      }
      b = *p++;
      result.value |= ((uint64_t(b) & varint_last_bit_mask) << 35U);
      if (b >= 0)
      {
        break;
      }
      b = *p++;
      result.value |= ((uint64_t(b) & varint_last_bit_mask) << 42U);
      if (b >= 0)
      {
        break;
      }
      b = *p++;
      result.value |= ((uint64_t(b) & varint_last_bit_mask) << 49U);
      if (b >= 0)
      {
        break;
      }
      b = *p++;
      result.value |= ((uint64_t(b) & varint_last_bit_mask) << 56U);
      if (b >= 0)
      {
        break;
      }
      b = *p++;
      result.value |= ((uint64_t(b) & 0x01U) << 63U);
      if (b >= 0)
      {
        break;
      }
      result.sts = varint_status::too_many_bytes;
      goto varint_done_parsing;
    } while (false);
  }
  else
  {
    int shift = 0;
    while (p != e && *p < 0)
    {
      result.value |= static_cast<uint64_t>(*p++ & varint_last_bit_mask)
                      << shift;
      shift += 7;
    }
    if (p == e)
    {
      result.sts = varint_status::too_few_bytes;
      goto varint_done_parsing;
    }
    result.value |= static_cast<uint64_t>(*p++) << shift;
  }

varint_done_parsing:
  *begin = reinterpret_cast<const char*>(p);
  return result;
}

varint_result<int64_t>
decode_varint_i64(const char** __restrict__ begin, const char* __restrict__ end)
{
  auto [vi_u64, sts] = decode_varint_u64(begin, end);
  // static cast is enough
  return { static_cast<int64_t>(vi_u64), sts };
}

varint_result<int32_t>
decode_varint_i32(const char** __restrict__ begin, const char* __restrict__ end)
{
  auto [vi_u64, sts] = decode_varint_u64(begin, end);
  // static cast is enough
  return { static_cast<int32_t>(vi_u64), sts };
}

varint_result<int64_t>
decode_varint_si64(const char** __restrict__ begin,
                   const char* __restrict__ end)
{
  auto [vi_u64, sts] = decode_varint_u64(begin, end);
  // see protobuf docs for more info:
  // https://developers.google.com/protocol-buffers/docs/encoding#signed-ints
  return { static_cast<int64_t>(
             (vi_u64 >> 1u) ^
             static_cast<uint64_t>(-static_cast<int64_t>(vi_u64 & 1u))),
           sts };
}

constexpr uint32_t
read_network_uint32_unchecked(span_fixed_size<const char, 4> buf) noexcept
{
  // we don't care about anything but little endian
  return (uint32_t(buf[0]) << 24u) | (uint32_t(buf[1]) << 16u) |
         (uint32_t(buf[2]) << 8u) | (uint32_t(buf[3]));
}

} // namespace cosm

#endif // COSM_NUMBERS_H__
