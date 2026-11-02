#include "trace.hpp"

#include <fstream>
#include <iostream>
#include <new>

int main(int argc, char *argv[])
{
    if (argc != 2)
    {
        printf("Invalid number of arguments. Expected 1. Got:%d\n", argc - 1);
        return 1;
    }

    LOG_MSG(TraceFile::MAIN_CPP, "Main\n");
    LOG_MSG_WITH_ARGS(TraceFile::ADMIN_TASK_CPP, "Admin %d\n", 1);
    LOG_MSG_WITH_ARGS(TraceFile::AUDIO_TASK_CPP, "Audio %s\n", "Two");
    LOG_MSG_WITH_ARGS(TraceFile::COMM_TASK_CPP, "Comm %d\n", 3);
    LOG_MSG_WITH_ARGS(TraceFile::DETECT_TASK_CPP, "Detect %d\n", 4);
    LOG_MSG_WITH_ARGS(TraceFile::STORAGE_TASK_CPP,
                      "Storage %s\n",
                      "Five and some extra text here");
    LOG_MSG_WITH_ARGS(TraceFile::W_HAL_GSM_CPP, "WHAL GSM %s\n", "Six");
    LOG_MSG_WITH_ARGS(TraceFile::W_HAL_GSM_MAXIM_CPP,
                      "WHAL GSM MAXIM %s\n",
                      "Seven and some extra text here");
    LOG_MSG(TraceFile::W_HAL_I2S_CPP, "WHAL I2S\n");
    LOG_MSG_WITH_ARGS(TraceFile::W_HAL_RTC_CPP,
                      "WHAL RTC %s\n",
                      "Nine and some extra text here");
    LOG_MSG_WITH_ARGS(TraceFile::W_HAL_SD_CARD_CPP, "WHAL SD CARD %d\n", 10);
    LOG_MSG_WITH_ARGS(TraceFile::W_HAL_SYSTEM_CPP,
                      "WHAL SYSTEM %s\n",
                      "Eleven");
    LOG_MSG_WITH_ARGS(TraceFile::W_HAL_UART_CPP, "WHAL UART %d\n", 12);

    uint32_t buffer_size;
    uint8_t *p_buffer_size = Trace::get_write_trace_buffer(buffer_size);

    printf("Opening file %s\n", argv[1]);
    std::ofstream outfile(argv[1], std::ios::out | std::ios::binary);

    if (!outfile)
    {
        printf("Cannot open file!\n");
        return 1;
    }
    outfile.write(reinterpret_cast<char *>(p_buffer_size), buffer_size);
    outfile.close();

    return 0;
}