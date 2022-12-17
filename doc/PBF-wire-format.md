## Protobuf wire format

Wire formats:

    0	VARINT	int32, int64, uint32, uint64, sint32, sint64, bool, enum
    1	I64	fixed64, sfixed64, double
    2	LEN	string, bytes, embedded messages, packed repeated fields
    3	SGROUP	group start (deprecated)
    4	EGROUP	group end (deprecated)
    5	I32	fixed32, sfixed32, float

Any field will be a varint of the form:

    (field_number << 3) | wire_type

wire_type fits in 3 bits.

**NOTE** we should never have more than 2^5 fields in the OSM format, but who knows

## Varints

Variable-width integers, or varints, are at the core of the wire format. They allow encoding unsigned 64-bit integers using anywhere between one and ten bytes, with small values using fewer bytes.

Each byte in the varint has a **continuation** bit that indicates if the byte that follows it is part of the varint. This is the most significant bit (MSB) of the byte (sometimes also called the sign bit). **The lower 7 bits are a payload**; the resulting integer is built by appending together the 7-bit payloads of its constituent bytes.

So, for example, here is the number 1, encoded as `01` -- it's a single byte, so the MSB is not set.

here's 150:
    
    10010110 00000001
    ^ msb    ^ msb
     0010110  0000001
     it's little endian, swap
     0000001  0010110
             10010110 = 150