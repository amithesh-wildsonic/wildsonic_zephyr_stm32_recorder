#ifndef LIB_TRACE_INCLUDE_ASSERT_HPP
#define LIB_TRACE_INCLUDE_ASSERT_HPP

#include "w_hal_system.hpp"

#include <stdint.h>
#include <stdio.h>

#define SYSTEM_ASSERT_TRUE(expr)                                               \
    if (!(expr))                                                               \
    {                                                                          \
        _assert("==", #expr, "true", (expr), true, __FILE__, __LINE__);        \
    };

#define SYSTEM_ASSERT_FALSE(expr)                                              \
    if (expr)                                                                  \
    {                                                                          \
        _assert("==", #expr, "false", (expr), false, __FILE__, __LINE__);      \
    };

#define SYSTEM_ASSERT_EQUAL(expr1, expr2)                                      \
    if ((expr1) != (expr2))                                                    \
    {                                                                          \
        _assert("==", #expr1, #expr2, (expr1), (expr2), __FILE__, __LINE__);   \
    };

#define SYSTEM_ASSERT_IN_SET(expr, values, number_of_values)                   \
    _assert_in_set(#expr,                                                      \
                   (expr),                                                     \
                   (values),                                                   \
                   (number_of_values),                                         \
                   __FILE__,                                                   \
                   __LINE__);

#define SYSTEM_ASSERT_NOT_EQUAL(expr1, expr2)                                  \
    if ((expr1) == (expr2))                                                    \
    {                                                                          \
        _assert("!=", #expr1, #expr2, (expr1), (expr2), __FILE__, __LINE__);   \
    };

#define SYSTEM_ASSERT_LT(expr1, expr2)                                         \
    if ((expr1) >= (expr2))                                                    \
    {                                                                          \
        _assert("<", #expr1, #expr2, (expr1), (expr2), __FILE__, __LINE__);    \
    };

#define SYSTEM_ASSERT_LTEQ(expr1, expr2)                                       \
    if ((expr1) > (expr2))                                                     \
    {                                                                          \
        _assert("<=", #expr1, #expr2, (expr1), (expr2), __FILE__, __LINE__);   \
    };

#define SYSTEM_ASSERT_GT(expr1, expr2)                                         \
    if ((expr1) <= (expr2))                                                    \
    {                                                                          \
        _assert(">", #expr1, #expr2, (expr1), (expr2), __FILE__, __LINE__);    \
    };

#define SYSTEM_ASSERT_GTEQ(expr1, expr2)                                       \
    if ((expr1) < (expr2))                                                     \
    {                                                                          \
        _assert(">=", #expr1, #expr2, (expr1), (expr2), __FILE__, __LINE__);   \
    };

/**
 * @brief Updates the assert information in shared global memory
 * @param core core number in which assert is occurred
 * @param comparator compare operator expressed as a string
 * @param expr1 first operand of test condition expressed as string
 * @param expr2 second operand of test condition expressed as string
 * @param value1 value after evaluating first expression
 * @param value2 value after evaluating second expression
 * @param file file in which assert occurred
 * @param line line number in which assert occurred
 */
void        update_shared_assert_info(int                    core,
                                      const char *           comparator,
                                      const char *           expr1,
                                      const char *           expr2,
                                      unsigned long long int value1,
                                      unsigned long long int value2,
                                      const char *           file,
                                      int                    line);

/**
 * @brief Fatal system error function
 * @tparam T type of value
 * @param core core number in which assert is occurred
 * @param expr operand of test condition expressed as string
 * @param value value after evaluating expression
 * @param values set of values
 * @param number_of_values length of values set
 * @param file file in which assert occurred
 * @param line line number in which assert occurred
 */
void        update_shared_assert_in_set_info(int                    core,
                                             const char *           expr,
                                             unsigned long long int value,
                                             unsigned long long int number_of_values,
                                             const char *           file,
                                             int                    line);

/**
 * @brief Fatal system error function
 * @param comparator compare operator expressed as a string
 * @param expr1 first operand of test condition expressed as string
 * @param expr2 second operand of test condition expressed as string
 * @param value1 value after evaluating first expression
 * @param value2 value after evaluating second expression
 * @param file file in which assert occurred
 * @param line line number in which assert occurred
 */
inline void _assert(const char *           comparator,
                    const char *           expr1,
                    const char *           expr2,
                    unsigned long long int value1,
                    unsigned long long int value2,
                    const char *           file,
                    int                    line)
{
    w_hal_system_set_error_led(true);

    update_shared_assert_info(__CORE__,
                              comparator,
                              expr1,
                              expr2,
                              value1,
                              value2,
                              file,
                              line);

    printf("ASSERT %s #%d: %s (Value:0x%llx) %s %s (Value:0x%llx)\n",
           file,
           line,
           expr1,
           value1,
           comparator,
           expr2,
           value2);

    while (1)
        ;
}

/**
 * @brief Fatal system error function
 * @tparam T type of values to compare
 * @param comparator compare operator expressed as a string
 * @param expr1 first operand of test condition expressed as string
 * @param expr2 second operand of test condition expressed as string
 * @param value1 value after evaluating first expression
 * @param value2 value after evaluating second expression
 * @param file file in which assert occurred
 * @param line line number in which assert occurred
 */
template <typename T>
inline void _assert(const char *comparator,
                    const char *expr1,
                    const char *expr2,
                    T           value1,
                    T           value2,
                    const char *file,
                    int         line)
{
    _assert(comparator,
            expr1,
            expr2,
            static_cast<unsigned long long int>(value1),
            static_cast<unsigned long long int>(value2),
            file,
            line);
}

/**
 * @brief Fatal system error function
 * @tparam T type of value for which to compare the pointers
 * @param comparator compare operator expressed as a string
 * @param expr1 first operand of test condition expressed as string
 * @param expr2 second operand of test condition expressed as string
 * @param p_value1 pointer address of first expression
 * @param p_value2 pointer address of second expression
 * @param file file in which assert occurred
 * @param line line number in which assert occurred
 */
template <typename T>
inline void _assert(const char *comparator,
                    const char *expr1,
                    const char *expr2,
                    T *         p_value1,
                    T *         p_value2,
                    const char *file,
                    int         line)
{
    uint64_t long_value1, long_value2;

#if INTPTR_MAX == INT64_MAX
    // Cast the pointer to 64-bit type directly
    long_value1 = reinterpret_cast<uint64_t>(p_value1);
    long_value2 = reinterpret_cast<uint64_t>(p_value2);
#elif INTPTR_MAX == INT32_MAX
    // First cast pointer to 32-bit type, then set long value to result
    long_value1 = reinterpret_cast<uint32_t>(p_value1);
    long_value2 = reinterpret_cast<uint32_t>(p_value2);
#endif

    _assert(comparator, expr1, expr2, long_value1, long_value2, file, line);
}

/**
 * @brief Fatal system error function
 * @tparam T type of value
 * @param expr operand of test condition expressed as string
 * @param value value after evaluating expression
 * @param values set of values
 * @param number_of_values length of values set
 * @param file file in which assert occurred
 * @param line line number in which assert occurred
 */
template <typename T>
inline void _assert_in_set(const char * expr,
                           T            value,
                           const T      values[],
                           const size_t number_of_values,
                           const char * file,
                           int          line)
{
    for (uint32_t index = 0; index < number_of_values; index++)
    {
        if (value == values[index])
        {
            // No halt required
            return;
        }
    }

    w_hal_system_set_error_led(true);

    update_shared_assert_in_set_info(
        __CORE__,
        expr,
        static_cast<long long unsigned int>(value),
        static_cast<long long unsigned int>(number_of_values),
        file,
        line);

    printf("ASSERT %s #%d: %s (Value:0x%llx) not in set of %lu values\n",
           file,
           line,
           expr,
           static_cast<long long unsigned int>(value),
           number_of_values);

    while (1)
        ;
}

#endif // LIB_TRACE_INCLUDE_ASSERT_HPP
