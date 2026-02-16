/**
 * @file serial_commands.h
 * @brief Serial command interface for GPS Lap Timer (ESP-IDF)
 * @version 2.0 - ESP-IDF native implementation
 *
 * Provides a simple serial CLI for debugging and control:
 * - Mode switching (GPS/Simulation)
 * - Lap reset
 * - Finish line configuration
 * - Status display
 * - Session management
 */

#ifndef SERIAL_COMMANDS_H
#define SERIAL_COMMANDS_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================
// PUBLIC INTERFACE
// ============================================================

/**
 * @brief Initialize serial command handler
 *
 * Initializes USB Serial JTAG for command input.
 */
void initSerialCommands(void);

/**
 * @brief Process incoming serial commands
 *
 * Should be called in main loop. Non-blocking - returns immediately
 * if no data is available.
 *
 * Supported commands:
 * - 'e' : Toggle between GPS and simulation mode
 * - 'r' : Reset current lap
 * - 'f' : Set finish line at current GPS position
 * - 'c' : Clear finish line
 * - 's' : Show system status
 * - 'l' : List saved laps in current session
 * - 'n' : Start new session
 * - 'd' : Display test
 * - 'h' : Show help message
 */
void handleSerialCommands(void);

/**
 * @brief Enable or disable command echo
 * @param enabled True to echo commands, false to disable
 */
void setCommandEcho(bool enabled);

/**
 * @brief Check if command echo is enabled
 * @return True if echo is enabled
 */
bool isCommandEchoEnabled(void);

#ifdef __cplusplus
}
#endif

#endif // SERIAL_COMMANDS_H
