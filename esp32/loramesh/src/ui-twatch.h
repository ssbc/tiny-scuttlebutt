// ui-watch.h

#if defined(TINYSSB_BOARD_TWATCH)

#ifndef _INCLUDE_UI_TWATCH_H
#define _INCLUDE_UI_TWATCH_H

#include "lib/tinySSBlib.h"
#include <lvgl.h>

class UI_TWatch_Class: public UIClass {

  using UIClass::UIClass;

public:
  UI_TWatch_Class();

  void refresh();
  
  void loop() override; // for screen animations
  void spinner(bool show) override;

  lv_obj_t *scr;
  lv_style_t bg_style;
  lv_obj_t *spin;

  // void buzz() override;
  void to_next_screen();
};

#endif // _INCLUDE_UI_TWATCH_H
#endif // TINYSSB_BOARD_TWATCH
