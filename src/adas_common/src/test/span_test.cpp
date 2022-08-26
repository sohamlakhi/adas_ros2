#include <gtest/gtest.h>

#include <adas_common/span.hpp>

#include <vector>
#include <array>
#include <algorithm>

using namespace adas_common;

namespace
{
constexpr size_t arrSize = 5;
}

TEST(Span, CtorDefault)
{
    Span<unsigned> span;

    EXPECT_EQ(span.size(), 0);
}

TEST(Span, CtorPointer)
{
    unsigned arr[arrSize] = {1, 2, 3, 4, 5};
    Span<unsigned> span{arr, arrSize};

    ASSERT_EQ(span.size(), arrSize);
    for (unsigned i = 0; i < span.size(); i++)
    { EXPECT_EQ(span[i], arr[i]); }
}

TEST(Span, CtorVector)
{
    unsigned arr[arrSize] = {1, 2, 3, 4, 5};
    std::vector<unsigned> v(arr, arr + arrSize);
    Span<unsigned> span{v};

    ASSERT_EQ(span.size(), v.size());
    for (unsigned i = 0; i < span.size(); i++)
    { EXPECT_EQ(span[i], v[i]); }
}

TEST(Span, CtorArray)
{
    unsigned arr[arrSize] = {1, 2, 3, 4, 5};
    std::array<unsigned, arrSize> a;
    std::copy(arr, arr + arrSize, a.begin());

    Span<unsigned> span{a};

    ASSERT_EQ(span.size(), a.size());
    for (unsigned i = 0; i < span.size(); i++)
    { EXPECT_EQ(span[i], a[i]); }
}

TEST(Span, VerifyView)
{
    unsigned arr[arrSize] = {1, 2, 3, 4, 5};
    Span<unsigned> span{arr, arrSize};

    ASSERT_EQ(span.size(), arrSize);
    for (unsigned i = 0; i < span.size(); i++)
    { EXPECT_EQ(&span[i], &arr[i]); }

    std::random_shuffle(arr, arr + arrSize);
    for (unsigned i = 0; i < span.size(); i++)
    { EXPECT_EQ(span[i], arr[i]); }
}

TEST(Span, Subspan)
{
    unsigned arr[arrSize] = {1, 2, 3, 4, 5};
    Span<unsigned> span{arr, arrSize};

    const std::array<std::pair<size_t, size_t>, 5> offsetCounts{
        std::make_pair(0, 0),
        std::make_pair(0, 2),
        std::make_pair(1, 4),
        std::make_pair(2, 1),
        std::make_pair(0, 5)};

    for (const auto &offsetCount: offsetCounts)
    {
        const auto offset = offsetCount.first;
        const auto count = offsetCount.second;
        auto subspan = span.makeSubspan(offset, count);
        ASSERT_EQ(subspan.size(), count);
        for (unsigned i = 0; i < subspan.size(); i++)
        { EXPECT_EQ(subspan[i], arr[offset + i]); }
    }
}

TEST(Span, SubspanEmpty)
{
    Span<unsigned> span;

    ASSERT_NO_THROW(span.makeSubspan(0, 0));
}

TEST(Span, SubspanError)
{
    unsigned arr[arrSize] = {1, 2, 3, 4, 5};
    Span<unsigned> span{arr, arrSize};

    EXPECT_THROW(span.makeSubspan(0, 6), std::runtime_error);
    EXPECT_THROW(span.makeSubspan(2, 4), std::runtime_error);
    EXPECT_THROW(span.makeSubspan(5, 0), std::runtime_error);
}

int main(int argc, char *argv[])
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
