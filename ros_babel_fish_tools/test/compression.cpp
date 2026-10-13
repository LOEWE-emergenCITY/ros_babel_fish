// Copyright (c) 2026 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include <compression.hpp>

#include <gtest/gtest.h>

#include <numeric>
#include <string>

using namespace ros_babel_fish_tools;

TEST( Compression, parseAlgorithm )
{
  CompressionAlgorithm algo = CompressionAlgorithm::None;
  EXPECT_TRUE( parse_compression_algorithm( "lz4", algo ) );
  EXPECT_EQ( algo, CompressionAlgorithm::Lz4 );
  EXPECT_TRUE( parse_compression_algorithm( "zstd", algo ) );
  EXPECT_EQ( algo, CompressionAlgorithm::Zstd );

  algo = CompressionAlgorithm::Lz4;
  EXPECT_FALSE( parse_compression_algorithm( "gzip", algo ) );
  EXPECT_FALSE( parse_compression_algorithm( "LZ4", algo ) );
  EXPECT_FALSE( parse_compression_algorithm( "", algo ) );
  EXPECT_EQ( algo, CompressionAlgorithm::Lz4 ) << "failed parse must not touch the output";
}

TEST( Compression, name )
{
  EXPECT_STREQ( compression_name( CompressionAlgorithm::Lz4 ), "lz4" );
  EXPECT_STREQ( compression_name( CompressionAlgorithm::Zstd ), "zstd" );
  EXPECT_STREQ( compression_name( CompressionAlgorithm::None ), "none" );
}

TEST( Compression, boundIsAtLeastInputSize )
{
  for ( size_t size : { size_t( 0 ), size_t( 1 ), size_t( 100 ), size_t( 65536 ) } ) {
    EXPECT_GE( compress_bound( CompressionAlgorithm::Lz4, size ), size );
    EXPECT_GE( compress_bound( CompressionAlgorithm::Zstd, size ), size );
  }
  EXPECT_THROW(
      compress_bound( CompressionAlgorithm::Lz4, static_cast<size_t>( LZ4_MAX_INPUT_SIZE ) + 1 ),
      std::runtime_error );
}

class CompressionRoundtrip : public ::testing::TestWithParam<CompressionAlgorithm>
{
};

TEST_P( CompressionRoundtrip, compressibleData )
{
  const CompressionAlgorithm algo = GetParam();
  // Highly redundant payload so we can also assert that compression actually shrinks it.
  std::string input;
  for ( int i = 0; i < 200; ++i ) input += "ros_babel_fish_tools ";
  const auto *data = reinterpret_cast<const uint8_t *>( input.data() );

  std::vector<char> compressed( compress_bound( algo, input.size() ) );
  const size_t n = compress( algo, data, input.size(), compressed );
  EXPECT_GT( n, 0u );
  EXPECT_LT( n, input.size() );

  std::vector<char> decompressed( input.size() );
  ASSERT_NO_THROW( decompress( algo, compressed.data(), n, input.size(), decompressed ) );
  EXPECT_EQ( std::string( decompressed.data(), input.size() ), input );
}

TEST_P( CompressionRoundtrip, incompressibleData )
{
  const CompressionAlgorithm algo = GetParam();
  // A pseudo-random payload must survive the roundtrip even though it does not shrink.
  std::vector<uint8_t> input( 4096 );
  uint32_t state = 12345;
  for ( auto &b : input ) {
    state = state * 1664525u + 1013904223u;
    b = static_cast<uint8_t>( state >> 24 );
  }

  std::vector<char> compressed( compress_bound( algo, input.size() ) );
  const size_t n = compress( algo, input.data(), input.size(), compressed );
  EXPECT_GT( n, 0u );
  EXPECT_LE( n, compressed.size() );

  std::vector<char> decompressed( input.size() );
  ASSERT_NO_THROW( decompress( algo, compressed.data(), n, input.size(), decompressed ) );
  EXPECT_TRUE( std::equal( input.begin(), input.end(),
                           reinterpret_cast<const uint8_t *>( decompressed.data() ) ) );
}

TEST_P( CompressionRoundtrip, emptyInput )
{
  const CompressionAlgorithm algo = GetParam();
  std::vector<char> compressed( compress_bound( algo, 0 ) );
  size_t n = 0;
  ASSERT_NO_THROW( n = compress( algo, nullptr, 0, compressed ) );
  std::vector<char> decompressed;
  EXPECT_NO_THROW( decompress( algo, compressed.data(), n, 0, decompressed ) );
}

TEST_P( CompressionRoundtrip, tooSmallDestinationThrows )
{
  const CompressionAlgorithm algo = GetParam();
  // Incompressible input into a 1-byte buffer cannot succeed and must be reported, not silently
  // truncated.
  std::vector<uint8_t> input( 1024 );
  std::iota( input.begin(), input.end(), 0 );
  std::vector<char> compressed( 1 );
  EXPECT_THROW( compress( algo, input.data(), input.size(), compressed ), std::runtime_error );
}

TEST_P( CompressionRoundtrip, corruptInputThrows )
{
  const CompressionAlgorithm algo = GetParam();
  const std::string garbage( 64, '\xff' );
  std::vector<char> decompressed( 1024 );
  EXPECT_THROW( decompress( algo, garbage.data(), garbage.size(), 1024, decompressed ),
                std::runtime_error );
}

INSTANTIATE_TEST_SUITE_P( Algorithms, CompressionRoundtrip,
                          ::testing::Values( CompressionAlgorithm::Lz4, CompressionAlgorithm::Zstd ),
                          []( const ::testing::TestParamInfo<CompressionAlgorithm> &info ) {
                            return std::string( compression_name( info.param ) );
                          } );
