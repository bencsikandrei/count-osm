#include <numbers.h>

#include <gtest/gtest.h>

TEST(TestReadNwU32, different_values_ok)
{
  char data[4]{ 0, 0, 0, 0 };

  EXPECT_EQ(cosm::read_network_uint32_unchecked(data), 0);

  data[0] = 0x7F;
  data[1] = 0x69;

  EXPECT_EQ(cosm::read_network_uint32_unchecked(data), 0x7F690000);

  data[2] = 0x01;
  data[3] = 0x42;

  EXPECT_EQ(cosm::read_network_uint32_unchecked(data), 0x7F690142);
}

TEST(TestDecodeVarintu64, small_values_ok)
{
  uint8_t data[1]{ 0x07 };
  const char* cursor = reinterpret_cast<const char*>(&data[0]);

  EXPECT_EQ(cosm::decode_varint_u64(&cursor, cursor + 1).value, 7);
}

TEST(TestDecodeVarintu64, medium_values_ok)
{
  {
    uint8_t data[2]{ 0x96, 0x01 };
    const char* cursor = reinterpret_cast<const char*>(&data[0]);

    // from https://developers.google.com/protocol-buffers/docs/encoding#varints
    EXPECT_EQ(cosm::decode_varint_u64(&cursor, cursor + 2).value, 150);
  }
  {
    uint8_t data[3]{ 0x80, 0x80, 0x01 };
    const char* cursor = reinterpret_cast<const char*>(&data[0]);

    EXPECT_EQ(cosm::decode_varint_u64(&cursor, cursor + 3).value, 16384);
  }
}

TEST(TestDecodeVarintu64, large_values_ok)
{
  uint8_t data[10]{
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x01
  };
  const char* cursor = reinterpret_cast<const char*>(&data[0]);

  EXPECT_EQ(cosm::decode_varint_u64(&cursor, cursor + 10).value, ~(0ull));
}

TEST(TestDecodeVarintu64, large_values_too_many_nok)
{
  // ends at the 11th pos, which is more than the max for varint
  uint8_t data[11]{ 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
                    0xff, 0xff, 0xff, 0xff, 0x01 };
  //                                     end ^ is too far
  const char* cursor = reinterpret_cast<const char*>(&data[0]);

  EXPECT_EQ(cosm::decode_varint_u64(&cursor, cursor + 11).sts,
            cosm::varint_status::too_many_bytes);
}

TEST(TestDecodeVarintu64, small_values_too_few_nok)
{
  // does not end
  uint8_t data[3]{ 0xff, 0xff, 0xff /* end missing ! */ };
  const char* cursor = reinterpret_cast<const char*>(&data[0]);

  EXPECT_EQ(cosm::decode_varint_u64(&cursor, cursor + 3).sts,
            cosm::varint_status::too_few_bytes);
}

TEST(TestDecodeVarintu64, small_zig_zag_ok)
{
  // https://developers.google.com/protocol-buffers/docs/encoding#signed-ints
  {
    uint8_t data[1]{ 0x03 };
    const char* cursor = reinterpret_cast<const char*>(&data[0]);

    EXPECT_EQ(cosm::decode_varint_si64(&cursor, cursor + 1).value, -2);
  }
  {
    uint8_t data[1]{ 0x02 };
    const char* cursor = reinterpret_cast<const char*>(&data[0]);

    EXPECT_EQ(cosm::decode_varint_si64(&cursor, cursor + 1).value, 1);
  }
  {
    uint8_t data[1]{ 0x04 };
    const char* cursor = reinterpret_cast<const char*>(&data[0]);

    EXPECT_EQ(cosm::decode_varint_si64(&cursor, cursor + 1).value, 2);
  }
}
