#ifndef COSM_PROTOBUF_H__
#define COSM_PROTOBUF_H__

#include <cstdint>

namespace cosm
{

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

/**
 * @brief Merge field_number and wire_type to form a pbf key (wt == last 3 bits)
 * @param field_number varint (not more than 5 bits for OSM pbfs)
 * @param wire_type max 3 bits
 * @return key based on inputs
 */
static constexpr uint32_t
protobuf_key(uint32_t field_number, uint8_t wire_type) noexcept
{
  constexpr uint8_t bits_for_wire_type = 3u;
  constexpr uint8_t mask_for_wire_type = ~(0xFFu << bits_for_wire_type) & 0xFFu;
  return (field_number << bits_for_wire_type) |
         (wire_type & mask_for_wire_type);
}

/**
 * @brief Get the wire type of a key
 * @param key input key
 * @return wiere type (fits in 3 bits)
 */
static constexpr uint32_t
protobuf_wire_type(uint32_t key) noexcept
{
  return key & 0x07u;
}

/**
 * @brief Get field number of a key (not more than 5 bits in OSM pbfs)
 * @param key input key
 * @return field number
 */
static constexpr uint32_t
protobuf_field_number(uint32_t key) noexcept
{
  return key >> 3;
}

} // namespace cosm

#endif // COSM_PROTOBUF_H__
