// Copyright (c) 2026 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "test_helpers.hpp"

#include <stats_common.hpp>

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

using namespace ros_babel_fish_tools;
using namespace ros_babel_fish_tools_test;

TEST( Accumulator, empty )
{
  Accumulator acc;
  EXPECT_EQ( acc.count, 0u );
  EXPECT_EQ( acc.sum, 0 );
  EXPECT_DOUBLE_EQ( acc.avg(), 0.0 );
}

TEST( Accumulator, tracksMinMaxSumCount )
{
  Accumulator acc;
  acc.add( 5 );
  EXPECT_EQ( acc.count, 1u );
  EXPECT_EQ( acc.min, 5 );
  EXPECT_EQ( acc.max, 5 );
  EXPECT_EQ( acc.sum, 5 );
  EXPECT_DOUBLE_EQ( acc.avg(), 5.0 );

  acc.add( -3 );
  acc.add( 10 );
  EXPECT_EQ( acc.count, 3u );
  EXPECT_EQ( acc.min, -3 );
  EXPECT_EQ( acc.max, 10 );
  EXPECT_EQ( acc.sum, 12 );
  EXPECT_DOUBLE_EQ( acc.avg(), 4.0 );
}

TEST( Accumulator, firstSampleInitializesMinMaxEvenIfNegative )
{
  // min/max must be seeded from the first value, not from the zero-initialized members.
  Accumulator acc;
  acc.add( -7 );
  EXPECT_EQ( acc.min, -7 );
  EXPECT_EQ( acc.max, -7 );
  acc.add( 42 );
  acc.reset();
  EXPECT_EQ( acc.count, 0u );
  acc.add( 100 );
  EXPECT_EQ( acc.min, 100 );
  EXPECT_EQ( acc.max, 100 );
}

TEST( Formatting, minAvgMax )
{
  Accumulator acc;
  EXPECT_EQ( format_min_avg_max( acc, 1.0, 2 ), "n/a" );
  acc.add( 1'000'000 );
  acc.add( 3'000'000 );
  EXPECT_EQ( format_min_avg_max( acc, 1e6, 2 ), "1.00/2.00/3.00" );
  EXPECT_EQ( format_min_avg_max( acc, 1e3, 1 ), "1000.0/2000.0/3000.0" );
  EXPECT_EQ( format_min_avg_max( acc, 1.0, 0 ), "1000000/2000000/3000000" );
}

TEST( Formatting, bytes )
{
  EXPECT_EQ( format_bytes( 0 ), "0 B" );
  EXPECT_EQ( format_bytes( 512 ), "512 B" );
  EXPECT_EQ( format_bytes( 1023.4 ), "1023 B" );
  EXPECT_EQ( format_bytes( 1024 ), "1.00 KiB" );
  EXPECT_EQ( format_bytes( 1536 ), "1.50 KiB" );
  EXPECT_EQ( format_bytes( 1024.0 * 1024.0 ), "1.00 MiB" );
  EXPECT_EQ( format_bytes( 2.5 * 1024.0 * 1024.0 * 1024.0 ), "2.50 GiB" );
  // Never goes beyond GiB.
  EXPECT_EQ( format_bytes( 2048.0 * 1024.0 * 1024.0 * 1024.0 ), "2048.00 GiB" );
  EXPECT_EQ( format_bytes_per_sec( 1536 ), "1.50 KiB/s" );
  EXPECT_EQ( format_bytes_per_sec( 7 ), "7 B/s" );
}

TEST( Parsing, positiveNumber )
{
  double out = 0;
  EXPECT_TRUE( parse_positive_number( "5", out ) );
  EXPECT_DOUBLE_EQ( out, 5.0 );
  EXPECT_TRUE( parse_positive_number( "0.25", out ) );
  EXPECT_DOUBLE_EQ( out, 0.25 );
  EXPECT_TRUE( parse_positive_number( "1e3", out ) );
  EXPECT_DOUBLE_EQ( out, 1000.0 );

  EXPECT_FALSE( parse_positive_number( "", out ) );
  EXPECT_FALSE( parse_positive_number( "0", out ) );
  EXPECT_FALSE( parse_positive_number( "-1", out ) );
  EXPECT_FALSE( parse_positive_number( "5x", out ) );
  EXPECT_FALSE( parse_positive_number( "abc", out ) );
  EXPECT_FALSE( parse_positive_number( "inf", out ) );
  EXPECT_FALSE( parse_positive_number( "nan", out ) );
  // strtod semantics: leading whitespace is skipped, trailing junk is not.
  EXPECT_TRUE( parse_positive_number( " 5", out ) );
  EXPECT_DOUBLE_EQ( out, 5.0 );
  EXPECT_FALSE( parse_positive_number( "5 ", out ) );
}

TEST( Parsing, positiveSeconds )
{
  double out = 0;
  EXPECT_TRUE( parse_positive_seconds( "2.5", out ) );
  EXPECT_DOUBLE_EQ( out, 2.5 );
  EXPECT_FALSE( parse_positive_seconds( "0", out ) );
  EXPECT_FALSE( parse_positive_seconds( "-2", out ) );
  // Would overflow the int64 nanosecond representation used for timer periods.
  EXPECT_FALSE( parse_positive_seconds( "1e10", out ) );
  EXPECT_FALSE( parse_positive_seconds( "1e300", out ) );
  EXPECT_TRUE( parse_positive_seconds( "1e9", out ) );
}

TEST( Parsing, takeOptionValue )
{
  const std::vector<std::string> args = { "tool", "--window", "3", "--out" };
  size_t i = 1;
  std::string value;
  EXPECT_TRUE( take_option_value( args, i, value ) );
  EXPECT_EQ( value, "3" );
  EXPECT_EQ( i, 2u ) << "index must advance past the consumed value";

  i = 3;
  value.clear();
  EXPECT_FALSE( take_option_value( args, i, value ) ) << "option without value";
  EXPECT_EQ( i, 3u );
  EXPECT_TRUE( value.empty() );
}

TEST( CsvOutput, opensNewFile )
{
  TempDir dir;
  const std::string path = dir.file( "out.csv" );
  std::ofstream csv;
  ASSERT_TRUE( open_csv_output( path, csv ) );
  EXPECT_TRUE( csv.is_open() );
  csv << "a,b\n";
  csv.close();
  EXPECT_TRUE( std::filesystem::exists( path ) );
}

TEST( CsvOutput, failsForUnwritablePath )
{
  TempDir dir;
  std::ofstream csv;
  EXPECT_FALSE( open_csv_output( dir.file( "missing_dir/out.csv" ), csv ) );
  EXPECT_FALSE( csv.is_open() );
}

TEST( CsvOutput, existingFileRequiresConfirmation )
{
  TempDir dir;
  const std::string path = dir.file( "existing.csv" );
  {
    std::ofstream f( path );
    f << "old content\n";
  }

  // Redirect std::cin so the confirmation prompt reads our answer.
  auto with_stdin = [&]( const std::string &answer, std::ofstream &csv ) {
    std::istringstream input( answer );
    std::streambuf *old = std::cin.rdbuf( input.rdbuf() );
    std::cin.clear();
    const bool ok = open_csv_output( path, csv );
    std::cin.rdbuf( old );
    std::cin.clear();
    return ok;
  };

  std::ofstream csv;
  EXPECT_FALSE( with_stdin( "n\n", csv ) );
  EXPECT_FALSE( csv.is_open() );
  EXPECT_FALSE( with_stdin( "", csv ) ) << "EOF (non-interactive) must default to no";
  EXPECT_FALSE( with_stdin( "yes please\n", csv ) ) << "only exact yes answers are accepted";
  {
    std::ifstream f( path );
    std::string content;
    std::getline( f, content );
    EXPECT_EQ( content, "old content" ) << "declining must leave the file untouched";
  }

  EXPECT_TRUE( with_stdin( "y\n", csv ) );
  EXPECT_TRUE( csv.is_open() );
  csv.close();
  EXPECT_EQ( std::filesystem::file_size( path ), 0u ) << "confirming truncates the file";

  {
    std::ofstream f( path );
    f << "old content\n";
  }
  std::ofstream csv2;
  EXPECT_TRUE( with_stdin( "yes\n", csv2 ) );
  csv2.close();
  std::ofstream csv3;
  EXPECT_TRUE( with_stdin( "Y\n", csv3 ) );
}
