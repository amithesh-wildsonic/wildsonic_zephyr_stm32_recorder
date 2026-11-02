#ifndef _HPF_HPP
#define _HPF_HPP

#include <climits>
#include <stdint.h>

/**
 * @brief
 * High pass DC offset filter
 *
 * @tparam D data type (can be int16_t or int32_t)
 * @tparam coeff coefficient used for filter
 */
template <typename D, unsigned long coeff>
class HPF
{
  public:
    HPF() = default;

    // Initialize DC offset filter member variables
    void initialize(void);

    // Do one step of high-pass filter on incoming sample
    D    filter(D input);

  private:
    D       x0, x1;
    int64_t y0, y1;
};

/**
 * @brief
 * Initialize DC offset filter member variables
 *
 * @tparam D data type (can be int16_t or int32_t)
 * @tparam coeff coefficient used for filter
 */
template <typename D, unsigned long coeff>
inline void HPF<D, coeff>::initialize(void)
{
    x0 = 0;
    y0 = 0;
    y1 = y0;
    x1 = x0;
}

/**
 * @brief
 * Do one step of high-pass filter on incoming sample
 *
 * @tparam D data type (can be int16_t or int32_t)
 * @tparam coeff coefficient used for filter
 * @param input last input sample
 * @return D filtered output sample
 */
template <typename D, unsigned long coeff>
inline D HPF<D, coeff>::filter(D input)
{
    const uint32_t sign_bit_pos = sizeof(D) * CHAR_BIT - 1;
    D              Acc, output;
    int64_t        tmp;

    /* a 1st order IIR high pass filter (100 Hz cutoff frequency)  */
    /* y(n)=x(n)-x(n-1)+A*y(n-1) and A =.995*2^sign_bit_pos */

    x0  = input;

    tmp = (coeff * y1);
    Acc = static_cast<D>((tmp + (1LL << (sign_bit_pos - 1))) >> sign_bit_pos);
    y0  = x0 - x1 + Acc;

    D lower_limit = static_cast<D>(1LL << sign_bit_pos);
    D upper_limit = lower_limit - 1;

    /* Clipping */
    if (y0 > upper_limit)
    {
        y0 = upper_limit;
    }

    if (y0 < lower_limit)
    {
        y0 = lower_limit;
    }

    /* Update filter state */
    y1     = y0;
    x1     = x0;

    output = static_cast<D>(y0);

    return (output);
}

#endif // _HPF_HPP