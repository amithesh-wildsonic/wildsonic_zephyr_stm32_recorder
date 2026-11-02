/**
 * @brief Read config file from SD card
 *
 * @return true if successfully read and applied the configuration settings
 * @return false if failed to read config file
 */
bool read_config_from_sd_card();

/**
 * @brief Create configuration status file
 */
void create_config_status_file();