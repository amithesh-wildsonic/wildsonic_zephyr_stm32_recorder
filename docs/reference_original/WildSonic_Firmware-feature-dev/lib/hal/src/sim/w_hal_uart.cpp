#include "w_hal_uart.hpp"       //UART header file
#include "w_hal_error_code.hpp" //Error code header foe HAL

#include "assert.hpp"

#include <boost/thread.hpp>
#include <iostream>
#include <stdio.h>

using namespace std;

static boost::thread *            p_input_thread           = nullptr;

static whal_uart_new_character_cb uart_console_cb_function = nullptr;

static void                       input_task()
{
    while (true)
    {
        char c;
        while (!cin.get(c))
        {
        }

        if (uart_console_cb_function != nullptr)
        {
            uart_console_cb_function(c);
        }
    }
}

/**
 * @brief Initialize UART and register a new input character callback function
 * to pass along to app layer
 *
 * @param callback callback function in app layer
 * @return wHalErrorCode NO_ERROR
 */
wHalErrorCode w_hal_uart_console_init(whal_uart_new_character_cb callback)
{
    if (p_input_thread == nullptr)
    {
        p_input_thread = new boost::thread(input_task);
    }

    uart_console_cb_function = callback;

    // No need to call uart initialization function here since it has already
    // been called earlier
    return wHalErrorCode::NO_ERROR;
}

/**
 * @brief UART teardown after completing all outstanding function will shutdown
 * the UART
 *
 * @param channel UART channel, must be less than max channels
 */
void w_hal_uart_teardown(uint8_t channel)
{
    p_input_thread           = nullptr;
    uart_console_cb_function = nullptr;
}