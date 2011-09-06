#ifndef FLETCHER_H
#define FLETCHER_H

struct fletcher_checksum_t
{
uint8_t first;
uint8_t second;
};

bool operator==( const fletcher_checksum_t & lhs, const fletcher_checksum_t & rhs )
{
return lhs.first==rhs.first && lhs.second==rhs.second;
}

static fletcher_checksum_t fletcher_checksum( const void * DATA, const unsigned int BYTES )
{
fletcher_checksum_t buffer;
buffer.first=0;
buffer.second=0;
const uint8_t * PTR = (uint8_t*)DATA;

for( unsigned int i = 0; i < BYTES; ++i )
    {
    buffer.first = buffer.first + PTR[i];
    buffer.second = buffer.second + buffer.first;
    }
return buffer;
}

#endif
