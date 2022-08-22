#ifndef DIFFERENCE_EQUATION_HPP_
#define DIFFERENCE_EQUATION_HPP_

#include <array>
#include <algorithm>
#include <numeric>
#include <cassert>

#include <boost/circular_buffer.hpp>

/*
    TODO: @brief --> explain what this header does. 
    get size of difference equation coeffs and states, make circular buffers and find difference. Return difference and add to buffer
    As the name suggests - it models the difference equation
*/
namespace adas_filters
{

/*
 * Representation for difference equations of form
 * Out[n] + a1 * Out[n - 1] + a2 * Out[n - 2] + ... = b0 * In[n] + b1 * In[n - 1] + ...
 * Out coefficients: {a1, a2, ...}
 * In coefficients: {b0, b1, ...}
 */

//you can have non-typename template arguments (such as size_t that can be used as an array size). See: https://stackoverflow.com/questions/1951519/when-to-use-stdsize-t https://www.modernescpp.com/index.php/class-templates https://en.cppreference.com/w/cpp/language/template_parameters(see template argument types, etc.)
template <size_t numInAndPrevs, size_t numOutPrevs> class DifferenceEquation {
    public:
        static_assert(numInAndPrevs > 0, "numInAndPrevs must be larger than zero."); //assertion statement

        //aliases for arrays (type: double, size: passed suring template decleration)
        using InCoeffs = std::array<double, numInAndPrevs>; 
        using OutCoeffs = std::array<double, numOutPrevs>;
        using InStates = InCoeffs;
        using OutStates = OutCoeffs;


        static constexpr size_t numInStates = numInAndPrevs;
        static constexpr size_t numOutStates = numOutPrevs;


        //double && reference for rvalue/lvalue resolution in template (see https://stackoverflow.com/questions/5481539/what-does-t-double-ampersand-mean-in-c11 https://stackoverflow.com/questions/8526598/how-does-stdforward-work https://stackoverflow.com/questions/3582001/what-are-the-main-purposes-of-using-stdforward-and-which-problems-it-solves/3582313#3582313)
        // Ctor with set initial conditions
        DifferenceEquation(
            InCoeffs &&inCoeffs,
            OutCoeffs &&outCoeffs,
            const InStates &inInitials,
            const OutStates &outInitials) :
            ins{inInitials.begin(), inInitials.end()},
            inCoeffs{std::move(inCoeffs)},
            outs{outInitials.begin(), outInitials.end()},
            outCoeffs{std::move(outCoeffs)}
        {} //note: InCoeffs has &&, but InStates has & since InStates has const

        // Ctor with initial conditions of 0
        DifferenceEquation(InCoeffs &&inCoeffs, OutCoeffs &&outCoeffs) :
            DifferenceEquation{
                std::move(inCoeffs),
                std::move(outCoeffs),
                InStates{},
                OutStates{}}
        {}

        void assignState(const InStates &inStates, const OutStates &outStates)
        {
            assignState(inStates.begin(), inStates.end(), outStates.begin(), outStates.end());
        }

        void assignState(const DifferenceEquation<numInAndPrevs, numOutPrevs> &other)
        {
            /*
            * Because the type matches, numbers of previous values kept also match,
            * which in turn means that sizes of the circular buffers also match.
            */
            assignState(other.ins.begin(), other.ins.end(), other.outs.begin(), other.outs.end());
        }

        void assignCoeffs(const InCoeffs &newInCoeffs, const OutCoeffs &newOutCoeffs)
        {
            std::copy(newInCoeffs.begin(), newInCoeffs.end(), inCoeffs.begin());
            std::copy(newOutCoeffs.begin(), newOutCoeffs.end(), outCoeffs.begin());
        }

        double output(const double input)
        {
            ins.push_front(input);
            double newOut =
                std::inner_product(inCoeffs.begin(), inCoeffs.end(), ins.begin(), 0.0) -
                std::inner_product(outCoeffs.begin(), outCoeffs.end(), outs.begin(), 0.0);
            outs.push_front(newOut);
            return newOut;
        }

    private:
        void verify() const
        {
            // Must hold true at any time - Circular buffer is always "full."
            assert(ins.size() == ins.capacity());
            assert(outs.size() == outs.capacity());
        }

        template<typename InStatesIt, typename OutStatesIt>
        void assignState(
            InStatesIt inFirst,
            InStatesIt inLast,
            OutStatesIt outFirst,
            OutStatesIt outLast)
        {
            verify();
            std::copy(inFirst, inLast, ins.begin());
            std::copy(outFirst, outLast, outs.begin());
        }

        boost::circular_buffer<double> ins;
        InCoeffs inCoeffs;
        boost::circular_buffer<double> outs;
        OutCoeffs outCoeffs;
};

}

#endif
