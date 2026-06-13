// Copyright (c) 2026 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef ROS_BABEL_FISH_TOOLS_COMPRESSION_HPP
#define ROS_BABEL_FISH_TOOLS_COMPRESSION_HPP

#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include <lz4.h>
#include <zstd.h>

namespace ros_babel_fish_tools
{

enum class CompressionAlgorithm { None, Lz4, Zstd };

//! Maps the CLI value ("lz4"/"zstd") to an algorithm.
//! @return False if the name is not a supported algorithm.
inline bool parse_compression_algorithm( const std::string &name, CompressionAlgorithm &out )
{
  if ( name == "lz4" ) {
    out = CompressionAlgorithm::Lz4;
    return true;
  }
  if ( name == "zstd" ) {
    out = CompressionAlgorithm::Zstd;
    return true;
  }
  return false;
}

inline const char *compression_name( CompressionAlgorithm algo )
{
  switch ( algo ) {
  case CompressionAlgorithm::Lz4:
    return "lz4";
  case CompressionAlgorithm::Zstd:
    return "zstd";
  default:
    return "none";
  }
}

//! Worst-case size of compressing @p size bytes with @p algo. Size the destination buffer passed to
//! compress() to at least this value; doing it up front keeps the (re)allocation out of a timed
//! region.
//! @throws std::runtime_error if @p size exceeds the algorithm's input limit.
inline size_t compress_bound( CompressionAlgorithm algo, size_t size )
{
  if ( algo == CompressionAlgorithm::Lz4 ) {
    // LZ4's simple API is limited to int-sized buffers; ROS messages are far below that limit.
    if ( size > static_cast<size_t>( LZ4_MAX_INPUT_SIZE ) )
      throw std::runtime_error( "message too large to compress with lz4" );
    return static_cast<size_t>( LZ4_compressBound( static_cast<int>( size ) ) );
  }
  return ZSTD_compressBound( size );
}

//! Compresses @p size bytes from @p data with @p algo into @p dst at the library default level. The
//! caller must size @p dst to at least compress_bound(algo, size) (so the allocation can be hoisted
//! out of a timed region); the compressed bytes are left in @p dst and are exactly what
//! decompress() expects as its input.
//! @return Number of bytes the input compresses to.
//! @throws std::runtime_error on a compression failure.
inline size_t compress( CompressionAlgorithm algo, const uint8_t *data, size_t size,
                        std::vector<char> &dst )
{
  if ( algo == CompressionAlgorithm::Lz4 ) {
    const int n = LZ4_compress_default( reinterpret_cast<const char *>( data ), dst.data(),
                                        static_cast<int>( size ), static_cast<int>( dst.size() ) );
    if ( n <= 0 )
      throw std::runtime_error( "LZ4_compress_default failed" );
    return static_cast<size_t>( n );
  }

  const size_t n = ZSTD_compress( dst.data(), dst.size(), data, size, ZSTD_CLEVEL_DEFAULT );
  if ( ZSTD_isError( n ) )
    throw std::runtime_error( std::string( "ZSTD_compress failed: " ) + ZSTD_getErrorName( n ) );
  return n;
}

//! Decompresses @p compressed bytes from @p data (produced by compress() with the same @p algo) into
//! @p dst. The caller must size @p dst to at least @p original_size. The decompressed output is
//! discarded; this exists only to measure decompression cost.
//! @throws std::runtime_error on a decompression failure.
inline void decompress( CompressionAlgorithm algo, const char *data, size_t compressed,
                        size_t original_size, std::vector<char> &dst )
{
  if ( algo == CompressionAlgorithm::Lz4 ) {
    const int n = LZ4_decompress_safe( data, dst.data(), static_cast<int>( compressed ),
                                       static_cast<int>( original_size ) );
    if ( n < 0 )
      throw std::runtime_error( "LZ4_decompress_safe failed" );
    return;
  }

  const size_t n = ZSTD_decompress( dst.data(), original_size, data, compressed );
  if ( ZSTD_isError( n ) )
    throw std::runtime_error( std::string( "ZSTD_decompress failed: " ) + ZSTD_getErrorName( n ) );
}

} // namespace ros_babel_fish_tools

#endif // ROS_BABEL_FISH_TOOLS_COMPRESSION_HPP
