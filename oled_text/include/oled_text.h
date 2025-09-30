#pragma once
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void custom_display_show_message(const char* message);

bool display_is_ready(void);

/**
 * @brief Set the text to be displayed on the OLED screen.
 *
 * @param text The text to display.
 */
void oled_display_text(const char *text);
void oled_scroll_text(const char *text);
#ifdef __cplusplus
}
#endif