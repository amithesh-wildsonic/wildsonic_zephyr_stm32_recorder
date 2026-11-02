#include "w_hal_spi_shared.hpp"

#include "assert.hpp"

#include "ff.h"

#include "board.h"
#include "diskio.h"
#include "spi.h"

/**
 * @brief SPI baton owner enum
 */
enum class spi_baton_owner_t : uint8_t
{
    NONE,
    OCM,
    SD_CARD,
    NUM_OWNERS
};

static spi_baton_owner_t
                spi_baton[MXC_SPI_INSTANCES]; // Designates the owner of spi
static uint32_t spi_request_baton_mask[MXC_SPI_INSTANCES]; // SPI Request mask
static bool     spi_initialized =
    false; // Indicates if SPI interface has already been initialized

/**
 * @brief Initialize spi shared variables
 *
 * @return wHalErrorCode NO_ERROR
 */
wHalErrorCode w_hal_spi_shared_initialize()
{
    if (!spi_initialized)
    {
        for (int spi = 0; spi < MXC_SPI_INSTANCES; spi++)
        {
            spi_baton[spi]              = spi_baton_owner_t::NONE;
            spi_request_baton_mask[spi] = 0;
        }

        spi_initialized = true;

        disk_set_spi_initialized();
    }

    return wHalErrorCode::NO_ERROR;
}

/**
 * @brief Request spi baton for use
 *
 * @param spi spi index
 * @param owner baton owner
 * @return wHalErrorCode BUSY if baton cannot be awarded, NO_ERROR otherwise
 */
wHalErrorCode w_hal_spi_shared_request_baton(uint32_t          spi,
                                             spi_baton_owner_t owner)
{
    SYSTEM_ASSERT_EQUAL(spi, 1);

    if (spi_baton[spi] == owner)
    {
        // Already have baton
        return wHalErrorCode::NO_ERROR;
    }
    else if (spi_baton[spi] == spi_baton_owner_t::NONE)
    {

        spi_request_baton_mask[spi] &= ~(1u << static_cast<uint32_t>(owner));
        spi_baton[spi] = owner;
        return wHalErrorCode::NO_ERROR;
    }
    else
    {
        spi_request_baton_mask[spi] |= 1u << static_cast<uint32_t>(owner);
        return wHalErrorCode::BUSY;
    }
}

/**
 * @brief Release spi baton after baton owner has finished using it
 *
 * @param spi spi index
 * @param owner baton owner
 * @return wHalErrorCode BUSY if baton cannot be awarded, NO_ERROR otherwise
 */
static wHalErrorCode w_hal_spi_shared_release_baton(uint32_t          spi,
                                                    spi_baton_owner_t owner)
{
    SYSTEM_ASSERT_EQUAL(spi, 1);

    if (spi_baton[spi] == owner)
    {
        spi_request_baton_mask[spi] &= ~(1u << static_cast<uint32_t>(owner));
        spi_baton[spi] = spi_baton_owner_t::NONE;
    }
    else
    {
        if (spi_baton[spi] == spi_baton_owner_t::NONE)
        {
            return wHalErrorCode::NO_ERROR;
        }
        else
        {
            return wHalErrorCode::BUSY;
        }
    }

    if (spi_request_baton_mask[spi] != 0)
    {
        for (owner = spi_baton_owner_t::NONE;
             owner != spi_baton_owner_t::NUM_OWNERS;
             owner = static_cast<spi_baton_owner_t>(
                 static_cast<uint32_t>(owner) + 1))
        {
            if (spi_request_baton_mask[spi] &
                (1u << static_cast<uint32_t>(owner)))
            {
                w_hal_spi_shared_request_baton(spi, owner);
                break;
            }
        }
    }

    return wHalErrorCode::NO_ERROR;
}

/**
 * @brief Request spi baton for OCM use
 *
 * @return wHalErrorCode BUSY if baton cannot be awarded, NO_ERROR otherwise
 */
wHalErrorCode w_hal_spi_shared_request_baton_for_ocm()
{
    wHalErrorCode error =
        w_hal_spi_shared_request_baton(1, spi_baton_owner_t::OCM);

    if (error == wHalErrorCode::NO_ERROR)
    {
        // Switch to quad mode
        mxc_spi_regs_t *spi = MXC_SPI_GET_SPI(1);
        MXC_SPI_SetWidth(spi, mxc_spi_width_t::SPI_WIDTH_QUAD);
        MXC_SPI_SetDefaultTXData(spi, 0xFF);
        MXC_SPI_SetFrequency(spi, SPI_SRAM_FREQ);
    }

    return error;
}

/**
 * @brief Request spi baton for SD card use
 *
 * @return wHalErrorCode BUSY if baton cannot be awarded, NO_ERROR otherwise
 */
wHalErrorCode w_hal_spi_shared_request_baton_for_sd_card()
{
    wHalErrorCode error =
        w_hal_spi_shared_request_baton(1, spi_baton_owner_t::SD_CARD);

    if (error == wHalErrorCode::NO_ERROR)
    {
        // Switch to standard mode
        mxc_spi_regs_t *spi = MXC_SPI_GET_SPI(1);
        MXC_SPI_SetWidth(spi, mxc_spi_width_t::SPI_WIDTH_STANDARD);
        MXC_SPI_SetDefaultTXData(spi, 0xFF);
        MXC_SPI_SetFrequency(spi, disk_get_spi_frequency());

        // Let lower api know baton has been awarded
        disk_award_spi_baton(true);
    }

    return error;
}

/**
 * @brief Release spi baton after OCM use
 *
 * @return wHalErrorCode BUSY if baton cannot be released or is not in
 * possession, NO_ERROR otherwise
 */
wHalErrorCode w_hal_spi_shared_release_baton_for_ocm()
{
    return w_hal_spi_shared_release_baton(1, spi_baton_owner_t::OCM);
}

/**
 * @brief Release spi baton after sd card use
 *
 * @return wHalErrorCode BUSY if baton cannot be released or is not in
 * possession, NO_ERROR otherwise
 */
wHalErrorCode w_hal_spi_shared_release_baton_for_sd_card()
{
    wHalErrorCode error =
        w_hal_spi_shared_release_baton(1, spi_baton_owner_t::SD_CARD);

    if (error == wHalErrorCode::NO_ERROR)
    {
        // Let lower api know baton has been revoked
        disk_award_spi_baton(false);
    }

    return error;
}