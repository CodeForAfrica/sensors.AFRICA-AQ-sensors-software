#ifndef _DEBUG_HELPER_H
#define _DEBUG_HELPER_H

#include <WString.h>

// Function declarations
static void debug_out(const String &text, unsigned int level);
static void debug_out(const __FlashStringHelper *text, unsigned int level);
static void debug_outln(const String &text, unsigned int level);
static void debug_outln_info(const String &text);
static void debug_outln_verbose(const String &text);
static void debug_outln_error(const __FlashStringHelper *text);
static void debug_outln_info(const __FlashStringHelper *text);
static void debug_outln_verbose(const __FlashStringHelper *text);
static void debug_outln_info(const __FlashStringHelper *text, const String &option);
static void debug_outln_info(const __FlashStringHelper *text, float value);
static void debug_outln_verbose(const __FlashStringHelper *text, const String &option);
static void debug_outln_info_bool(const __FlashStringHelper *text, const bool option);

// Function definitions
/*****************************************************************
 * Debug output                                                  *
 *****************************************************************/

#define debug_level_check(level) \
    {                            \
        if (level > cfg::debug)  \
            return;              \
    }

static void debug_out(const String &text, unsigned int level)
{
    debug_level_check(level);
    Serial.print(text);
}

static void debug_out(const __FlashStringHelper *text, unsigned int level)
{
    debug_level_check(level);
    Serial.print(text);
}

static void debug_outln(const String &text, unsigned int level)
{
    debug_level_check(level);
    Serial.println(text);
}

static void debug_outln_info(const String &text)
{
    debug_level_check(DEBUG_MIN_INFO);
    Serial.println(text);
}

static void debug_outln_verbose(const String &text)
{
    debug_level_check(DEBUG_MED_INFO);
    Serial.println(text);
}

static void debug_outln_error(const __FlashStringHelper *text)
{
    debug_level_check(DEBUG_ERROR);
    Serial.println(text);
}

static void debug_outln_info(const __FlashStringHelper *text)
{
    debug_level_check(DEBUG_MIN_INFO);
    Serial.println(text);
}

static void debug_outln_verbose(const __FlashStringHelper *text)
{
    debug_level_check(DEBUG_MED_INFO);
    Serial.println(text);
}

static void debug_outln_info(const __FlashStringHelper *text, const String &option)
{
    debug_level_check(DEBUG_MIN_INFO);
    Serial.print(text);
    Serial.println(option);
}

static void debug_outln_info(const __FlashStringHelper *text, float value)
{
    debug_outln_info(text, String(value));
}

static void debug_outln_verbose(const __FlashStringHelper *text, const String &option)
{
    debug_level_check(DEBUG_MED_INFO);
    Serial.print(text);
    Serial.println(option);
}

static void debug_outln_info_bool(const __FlashStringHelper *text, const bool option)
{
    debug_level_check(DEBUG_MIN_INFO);
    Serial.print(text);
    Serial.println(String(option));
}

#undef debug_level_check
#endif