#ifndef _VERSION_HPP
#define _VERSION_HPP

#include <stdio.h>

extern const char *const fw_version;
extern const char *const git_branch;
extern const char *const git_sha;
extern const char *const build_datetime;

/**
 * @brief Print version information
 */
inline void              print_fw_version_information()
{
    printf("Bootloader SHA: %s\n", git_sha);
    printf("Bootloader Build Date: %s\n", build_datetime);
}

#endif // _VERSION_HPP