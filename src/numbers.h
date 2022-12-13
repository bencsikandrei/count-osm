#ifndef COSM_NUMBERS_H__
#define COSM_NUMBERS_H__

#include "span.h"

// TODO get rid of cstdint and use something we make
#include <cstdint>

namespace cosm
{

constexpr uint32_t
unchecked_read_network_uint32(span_fixed_size<const char, 4> buf) noexcept
{
  // we don't care about anything but little endian
  return (uint32_t(buf[0]) << 24u) | (uint32_t(buf[1]) << 16u) |
         (uint32_t(buf[2]) << 8u) | (uint32_t(buf[3]));
}

} // namespace cosm

#endif // COSM_NUMBERS_H__
