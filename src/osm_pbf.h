#ifndef COSM_OSM_PBF_H__
#define COSM_OSM_PBF_H__

#include "protobuf.h"
#include "span.h"

#include <cstdint>

namespace cosm
{

/**
 * @brief We care only about these kinds of entities in the pbf
 */
enum class entity_state : uint32_t
{
  nodes = 0,
  ways = 1,
  relations = 2
};

static_assert(sizeof(entity_state) == 4);

/**
 * @brief Convert state to string
 * @param es input state
 * @return string for state
 */
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

/**
 * @brief Represents a blob from the OSM pbf format (see docs)
 */
struct pbf_blob
{
  /**
   * @brief We only support zlib for now
   */
  enum class compression : uint32_t
  {
    none = 0,
    zlib
  };
  compression compression_type = compression::none;
  // when no compression, equal to data.size()
  uint32_t raw_size = 0;
  cosm::span<const char> data;
};

static_assert(sizeof(pbf_blob) == 24);

/**
 * @brief Represents a pbf field (we only have varints and length delims)
 */
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

  // field and wt
  uint32_t key = 0;
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

static_assert(sizeof(pbf_field) == 24);

} // namespace cosm

#endif // COSM_OSM_PBF_H__
