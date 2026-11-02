#include "assert.hpp"
#include "shared.hpp"

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
void update_shared_assert_info(int                    core,
                               const char *           comparator,
                               const char *           expr1,
                               const char *           expr2,
                               unsigned long long int value1,
                               unsigned long long int value2,
                               const char *           file,
                               int                    line)
{
    // Set assert flag
    shared_glbl.shared_assert_info.asserted      = true;

    // Clear assert_in_set flag condition
    shared_glbl.shared_assert_info.assert_in_set = false;

    shared_glbl.shared_assert_info.core          = core;
    shared_glbl.shared_assert_info.comparator    = comparator;
    shared_glbl.shared_assert_info.expr1         = expr1;
    shared_glbl.shared_assert_info.expr2         = expr2;
    shared_glbl.shared_assert_info.value1        = value1;
    shared_glbl.shared_assert_info.value2        = value2;
    shared_glbl.shared_assert_info.file          = file;
    shared_glbl.shared_assert_info.line          = line;
}

/**
 * @brief Fatal system error function
 * @param core core number in which assert is occurred
 * @param expr operand of test condition expressed as string
 * @param value value after evaluating expression
 * @param number_of_values length of values set
 * @param file file in which assert occurred
 * @param line line number in which assert occurred
 */
void update_shared_assert_in_set_info(int                    core,
                                      const char *           expr,
                                      unsigned long long int value,
                                      unsigned long long int number_of_values,
                                      const char *           file,
                                      int                    line)
{
    // Set assert flag
    shared_glbl.shared_assert_info.asserted         = true;

    // Set assert_in_set flag condition
    shared_glbl.shared_assert_info.assert_in_set    = true;

    shared_glbl.shared_assert_info.core             = core;
    shared_glbl.shared_assert_info.expr1            = expr;
    shared_glbl.shared_assert_info.value1           = value;
    shared_glbl.shared_assert_info.number_of_values = number_of_values;
    shared_glbl.shared_assert_info.file             = file;
    shared_glbl.shared_assert_info.line             = line;
}