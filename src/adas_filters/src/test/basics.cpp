#include <gtest/gtest.h>

#include "adas_filters/difference_equation.hpp"
#include "adas_filters/discrete_system.hpp"
#include "adas_filters/discrete_filter.hpp"

using namespace adas_filters;

TEST(DifferenceEquation, Trivial)
{
    using Eqn = DifferenceEquation<1, 0>;
    // Out[n] = In[n]
    Eqn eqn{Eqn::InCoeffs{1.0}, Eqn::OutCoeffs{}};

    for (unsigned i = 0; i < 10; i++)
    {
        const double in = i;
        EXPECT_DOUBLE_EQ(in, eqn.output(in));
    }
}

TEST(DifferenceEquation, RunningAverage)
{
    constexpr size_t windowSize = 5;
    using Eqn = DifferenceEquation<windowSize, 0>;
    // Out[n] = sum(In) / <Number of In>
    Eqn::InCoeffs avgCoeffs;
    avgCoeffs.fill(1.0 / Eqn::numInStates);
    Eqn eqn{std::move(avgCoeffs), Eqn::OutCoeffs{}};

    double inSum = 0.0;
    for (unsigned i = 0; i < windowSize; i++)
    {
        const double in = i + 1;
        inSum += in;
        const double expected = inSum  / windowSize;

        EXPECT_DOUBLE_EQ(expected, eqn.output(in));
    }
}

TEST(DifferenceEquation, Delay)
{
    using Eqn = DifferenceEquation<2, 0>;
    // Out[n] = In[n - 1]
    Eqn eqn{Eqn::InCoeffs{0.0, 1.0}, Eqn::OutCoeffs{}};

    for (unsigned i = 0; i < 10; i++)
    {
        const double prev = i;
        const double curr = i + 1;

        EXPECT_DOUBLE_EQ(prev, eqn.output(curr));
    }
}

TEST(DifferenceEquation, OutDelay)
{
    using Eqn = DifferenceEquation<1, 2>;
    // Out[n] = In[n] - Out[n - 2]
    Eqn eqn{Eqn::InCoeffs{1.0}, Eqn::OutCoeffs{0.0, 1.0}};

    constexpr size_t numIns = 10;
    const std::array<double, numIns> expected{0, 1, 2, 2, 2, 3, 4, 4, 4, 5};
    for (unsigned i = 0; i < numIns; i++)
    {
        const double in = i;
        EXPECT_DOUBLE_EQ(expected[i], eqn.output(in));
    }
}

TEST(DifferenceEquation, WithInitial)
{
    using Eqn = DifferenceEquation<2, 1>;
    // Out[n] = 0.1 * In[n] + 0.9 * In[n -1] - 0.8 * Out[n - 1]
    Eqn eqn{
        Eqn::InCoeffs{0.1, 0.9},
        Eqn::OutCoeffs{0.8},
        Eqn::InStates{-0.7, 1.4},
        Eqn::OutStates{-3.0}};

    const double in = 2.1;
    const double expected = 0.1 * in + 0.9 * -0.7 - (0.8 * -3.0);
    EXPECT_DOUBLE_EQ(expected, eqn.output(in));
}

TEST(DifferenceEquation, AssignInStateFromArrays)
{
    using Eqn = DifferenceEquation<2, 0>;
    Eqn eqn{
        Eqn::InCoeffs{0.2, 0.4},
        Eqn::OutCoeffs{}};

    const double in1 = -2.5;
    ASSERT_DOUBLE_EQ(0.2 * -2.5, eqn.output(in1));

    eqn.assignState(Eqn::InStates{1.2, 7.6}, Eqn::OutStates{});
    const double in2 = 3.1;
    EXPECT_DOUBLE_EQ(0.2 * 3.1 + 0.4 * 1.2, eqn.output(in2));
}

TEST(DifferenceEquation, AssignOutStateFromArrays)
{
    using Eqn = DifferenceEquation<1, 1>;
    Eqn eqn{
        Eqn::InCoeffs{-0.2},
        Eqn::OutCoeffs{0.7}};

    const double in1 = 3.2;
    ASSERT_DOUBLE_EQ(-0.2 * 3.2, eqn.output(in1));

    eqn.assignState(Eqn::InStates{1.6}, Eqn::OutStates{7.8});
    const double in2 = -0.1;
    EXPECT_DOUBLE_EQ(-0.2 * -0.1 - 0.7 * 7.8, eqn.output(in2));
}

TEST(DifferenceEquation, AssignInStateFromAnother)
{
    using Eqn = DifferenceEquation<2, 0>;
    Eqn eqn{
        Eqn::InCoeffs{0.1, 0.2},
        Eqn::OutCoeffs{},
        Eqn::InStates{0.3, 0.4},
        Eqn::OutStates{}};
    Eqn other{
        Eqn::InCoeffs{0.5, 0.6},
        Eqn::OutCoeffs{},
        Eqn::InStates{0.7, 0.8},
        Eqn::OutStates{}};

    const double inThis1 = 0.9;
    const double inOther = -1.3;
    ASSERT_DOUBLE_EQ(0.1 * 0.9 + 0.2 * 0.3, eqn.output(inThis1));
    ASSERT_DOUBLE_EQ(0.5 * -1.3 + 0.6 * 0.7, other.output(inOther));

    eqn.assignState(other);

    const double inThis2 = 5.1;
    EXPECT_DOUBLE_EQ(0.1 * 5.1 + 0.2 * -1.3, eqn.output(inThis2));
}

TEST(DifferenceEquation, AssignOutStateFromAnother)
{
    using Eqn = DifferenceEquation<1, 1>;
    Eqn eqn{
        Eqn::InCoeffs{-0.2},
        Eqn::OutCoeffs{0.7}};
    Eqn other{
        Eqn::InCoeffs{0.3},
        Eqn::OutCoeffs{-2.1}};

    const double inThis1 = 3.2;
    const double inOther = 1.7;
    const double otherExpected = 0.3 * inOther;
    ASSERT_DOUBLE_EQ(-0.2 * 3.2, eqn.output(inThis1));
    ASSERT_DOUBLE_EQ(otherExpected, other.output(inOther));

    eqn.assignState(other);
    const double inThis2 = 1.3;
    EXPECT_DOUBLE_EQ(-0.2 * inThis2 - (0.7 * otherExpected), eqn.output(inThis2));
}

TEST(DiscreteSystem, Initial)
{
    using Sys = DiscreteSystem<2, 1>;
    const double initialInput = 0.0;
    const double initialOutput = 4.0;
    Sys sys{Sys::Eqn::InCoeffs{1.0, 2.0}, Sys::Eqn::OutCoeffs{3.0}, initialOutput};

    constexpr size_t N = 5;

    std::array<double, N> ins{1.0, -2.0, 3.0, -4.0, 5.0};
    std::array<double, N> outs;

    outs[0] = 1.0 * ins[0] + 2.0 * initialInput - (3.0 * initialOutput);
    for (unsigned i = 1; i < N; i++)
    {
        outs[i] = 1.0 * ins[i] + 2.0 * ins[i - 1] - (3.0 * outs[i - 1]);
    }

    for (unsigned i = 0; i < N; i++)
    {
        EXPECT_DOUBLE_EQ(outs[i], sys.output(ins[i]));
    }
}

TEST(DiscreteSystem, Reset)
{
    using Sys = DiscreteSystem<2, 1>;
    const double initialInput = 0.0;
    const double initialOutput = 1.0;
    Sys sys{Sys::Eqn::InCoeffs{4.0, 3.0}, Sys::Eqn::OutCoeffs{2.0}, initialOutput};

    constexpr size_t N = 3;
    std::array<double, N> ins{3.0, -2.0, 1.0};
    std::array<double, N> outs{};
    outs[0] =  4.0 * ins[0] + 3.0 * initialInput - (2.0 * initialOutput);
    for (unsigned i = 1; i < N; i++)
    {
        outs[i] = 4.0 * ins[i] + 3.0 * ins[i - 1] - (2.0 * outs[i - 1]);
    }

    for (unsigned i = 0; i < N; i++)
    {
        ASSERT_DOUBLE_EQ(outs[i], sys.output(ins[i]));
    }

    sys.reset();

    for (unsigned i = 0; i < N; i++)
    {
        EXPECT_DOUBLE_EQ(outs[i], sys.output(ins[i]));
    }
}

TEST(DiscreteSystem, ResetWithNewInitial)
{
    using Sys = DiscreteSystem<2, 1>;
    const double initialInput = 0.0;
    const double initialOutput = 1.0;
    const double newInitialOutput = -1.0;
    Sys sys{Sys::Eqn::InCoeffs{4.0, 3.0}, Sys::Eqn::OutCoeffs{2.0}, initialOutput};

    constexpr size_t N = 3;
    std::array<double, N> ins{3.0, -2.0, 1.0};
    std::array<double, N> outs{};
    std::array<double, N> newOuts{};
    outs[0] =  4.0 * ins[0] + 3.0 * initialInput - (2.0 * initialOutput);
    newOuts[0] =  4.0 * ins[0] + 3.0 * initialInput - (2.0 * newInitialOutput);
    for (unsigned i = 1; i < N; i++)
    {
        outs[i] = 4.0 * ins[i] + 3.0 * ins[i - 1] - (2.0 * outs[i - 1]);
        newOuts[i] = 4.0 * ins[i] + 3.0 * ins[i - 1] - (2.0 * newOuts[i - 1]);
    }

    for (unsigned i = 0; i < N; i++)
    {
        ASSERT_DOUBLE_EQ(outs[i], sys.output(ins[i]));
    }

    sys.reset(newInitialOutput);

    for (unsigned i = 0; i < N; i++)
    {
        EXPECT_DOUBLE_EQ(newOuts[i], sys.output(ins[i]));
    }
}

TEST(DiscreteFilter, InitialCollectionStateEqual)
{
    using Filt = DiscreteFilter<2, 1>;
    Filt filt{Filt::Eqn::InCoeffs{0.4, 0.2}, Filt::Eqn::OutCoeffs{-0.4}};

    constexpr size_t N = 10;
    std::array<double, N> ins{0.1, 0.2, 0.0, 0.4, 0.5, 0.6, 0.85, 0.8, 0.6, 1.0};
    std::array<double, N> outs{};
    outs[0] = ins[0];

    for (unsigned i = 1; i < N; i++)
    {
        outs[i] = 0.4 * ins[i] + 0.2 * ins[i - 1] + 0.4 * outs[i - 1];
    }

    for (unsigned i = 0; i < N; i++)
    {
        EXPECT_DOUBLE_EQ(outs[i], filt.output(ins[i]));
    }
}

TEST(DiscreteFilter, InitialCollectionInOnly)
{
    using Filt = DiscreteFilter<3, 0>;
    Filt filt{Filt::Eqn::InCoeffs{0.3, 0.1, 0.05}, Filt::Eqn::OutCoeffs{}};
    constexpr size_t N = 10;
    std::array<double, N> ins{0.1, 0.2, 0.0, 0.4, 0.5, 0.6, 0.85, 0.8, 0.6, 1.0};
    std::array<double, N> outs{};
    outs[0] = ins[0];
    outs[1] = ins[1];

    for (unsigned i = 2; i < N; i++)
    {
        outs[i] = 0.3 * ins[i] + 0.1 * ins[i - 1] + 0.05 * ins[i - 2];
    }

    for (unsigned i = 0; i < N; i++)
    {
        EXPECT_DOUBLE_EQ(outs[i], filt.output(ins[i]));
    }
}

TEST(DiscreteFilter, InitialCollectionInBigger)
{
    using Filt = DiscreteFilter<3, 1>;
    Filt filt{Filt::Eqn::InCoeffs{0.3, 0.1, 0.05}, Filt::Eqn::OutCoeffs{-0.5}};
    constexpr size_t N = 10;
    std::array<double, N> ins{0.1, 0.2, 0.0, 0.4, 0.5, 0.6, 0.85, 0.8, 0.6, 1.0};
    std::array<double, N> outs{};
    outs[0] = ins[0];
    outs[1] = ins[1];

    for (unsigned i = 2; i < N; i++)
    {
        outs[i] = 0.3 * ins[i] + 0.1 * ins[i - 1] + 0.05 * ins[i - 2] + 0.5 * outs[i - 1];
    }

    for (unsigned i = 0; i < N; i++)
    {
        EXPECT_DOUBLE_EQ(outs[i], filt.output(ins[i]));
    }
}

TEST(DiscreteFilter, InitialCollectionOutBigger)
{
    using Filt = DiscreteFilter<1, 2>;
    Filt filt{Filt::Eqn::InCoeffs{0.5}, Filt::Eqn::OutCoeffs{-0.4, -0.1}};
    constexpr size_t N = 10;
    std::array<double, N> ins{0.1, 0.2, 0.0, 0.4, 0.5, 0.6, 0.85, 0.8, 0.6, 1.0};
    std::array<double, N> outs{};
    outs[0] = ins[0];
    outs[1] = ins[1];

    for (unsigned i = 2; i < N; i++)
    {
        outs[i] = 0.5 * ins[i] + 0.4 * outs[i - 1] + 0.1 * outs[i - 2];
    }

    for (unsigned i = 0; i < N; i++)
    {
        EXPECT_DOUBLE_EQ(outs[i], filt.output(ins[i]));
    }
}

TEST(DiscreteFilter, InheritSimple)
{
}

TEST(DiscreteFilter, InheritMidCollection)
{
}

TEST(DiscreteFilter, Reset)
{
    using Filt = DiscreteFilter<1, 1>;
    Filt filt{Filt::Eqn::InCoeffs{0.7}, Filt::Eqn::OutCoeffs{-0.3}};

    constexpr size_t N = 10;
    std::array<double, N> ins{0.3, 0.1, 0.5, 0.6, 0.9, 0.8, 0.9, 1.0, 1.7, 1.2};
    std::array<double, N> outs{};
    outs[0] = ins[0];

    for (unsigned i = 1; i < N; i++)
    {
        outs[i] = 0.7 * ins[i] + 0.3 * outs[i - 1];
    }

    for (unsigned i = 0; i < N; i++)
    {
        ASSERT_DOUBLE_EQ(outs[i], filt.output(ins[i]));
    }

    filt.reset();

    for (unsigned i = 0; i < N; i++)
    {
        EXPECT_DOUBLE_EQ(outs[i], filt.output(ins[i]));
    }
}

int main(int argc, char *argv[])
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
