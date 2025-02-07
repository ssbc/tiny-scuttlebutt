// ui-twatch.cpp

#include "ui-twatch.h"

#if defined(TINYSSB_BOARD_TWATCH)

#include "lib/tinySSBlib.h"

#include "hardware.h"
#include "lib/cmd.h"

#include <SPI.h>
#include <TFT_eSPI.h>
#include <Wire.h>
// #include "es7210.h"
// #include <Audio.h>
// #include <driver/i2s.h>

extern TFT_eSPI    tft;
extern void setBrightness(uint8_t level);

// ---------------------------------------------------------------------------

#include <TouchDrvFT6X36.hpp>
extern TouchDrvFT6X36 sensor_touch;

static void touchpad_read2(lv_indev_drv_t *indev_driver, lv_indev_data_t *data)
{
    int16_t x, y;
    bool touched = sensor_touch.getPoint(&x, &y);
    if ( !touched ) {
        data->state = LV_INDEV_STATE_REL;
    } else {
        switch (tft.getRotation()) {
        case 0:
            data->point.x = TFT_WIDTH - x;
            data->point.y = TFT_HEIGHT - y;
            break;
        case 1:
            data->point.x = TFT_WIDTH - y;
            data->point.y =  x;
            break;
        case 3:
            data->point.x = y;
            data->point.y = TFT_HEIGHT - x;
            break;
        case 2:
        default:
            data->point.x = x;
            data->point.y = y;
        }
        // Serial.printf("# touch X:%d Y:%d\r\n", data->point.x, data->point.y);
        data->state = LV_INDEV_STATE_PR;
    }
}

// ---------------------------------------------------------------------------

SemaphoreHandle_t xSemaphore = NULL;

static void disp_flush( lv_disp_drv_t *disp,
                        const lv_area_t *area, lv_color_t *color_p )
{
    uint32_t w = ( area->x2 - area->x1 + 1 );
    uint32_t h = ( area->y2 - area->y1 + 1 );
    if ( xSemaphoreTake( xSemaphore, portMAX_DELAY ) == pdTRUE ) {
        tft.startWrite();
        tft.setAddrWindow( area->x1, area->y1, w, h );
        tft.pushColors( ( uint16_t * )&color_p->full, w * h, true );
        tft.endWrite();
        lv_disp_flush_ready( disp );
        xSemaphoreGive( xSemaphore );
    }
}

static void setupLvgl()
{
  Serial.println("# setupLvgl()");
    static lv_disp_draw_buf_t draw_buf;

#ifndef BOARD_HAS_PSRAM
#define LVGL_BUFFER_SIZE    ( TFT_WIDTH * 50 )
    static lv_color_t buf1[ LVGL_BUFFER_SIZE ];
    static lv_color_t buf2[ LVGL_BUFFER_SIZE ];
#else
#define LVGL_BUFFER_SIZE    (TFT_WIDTH * TFT_HEIGHT * sizeof(lv_color_t))
    static lv_color_t *buf1 = (lv_color_t *) ps_malloc(LVGL_BUFFER_SIZE);
    static lv_color_t *buf2 = (lv_color_t *) ps_malloc(LVGL_BUFFER_SIZE);
    if (!buf1 || !buf2) {
      Serial.printf("PSRAM malloc for LVGL failed: free=%d %d %d %d, trying standard RAM\r\n",
                    ESP.getFreeHeap(),
                    LVGL_BUFFER_SIZE, TFT_WIDTH, TFT_HEIGHT);
      buf1 = (lv_color_t *) malloc(LVGL_BUFFER_SIZE);
      buf2 = (lv_color_t *) malloc(LVGL_BUFFER_SIZE);
      delay(5000);
      assert(buf1);
      assert(buf2);
    }
#endif
    // memset(buf, 0, LVGL_BUFFER_SIZE);

    String LVGL_Arduino = String("# LVLG Arduino ") + lv_version_major() +
                          "." + lv_version_minor() + "." + lv_version_patch();
    Serial.println( LVGL_Arduino );

    lv_init();
    lv_group_set_default(lv_group_create());

#ifndef BOARD_HAS_PSRAM
    lv_disp_draw_buf_init(&draw_buf, buf1, buf2, sizeof(buf1));
#else
    lv_disp_draw_buf_init(&draw_buf, buf1, buf2, LVGL_BUFFER_SIZE);
#endif

    // Initialize the display
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init( &disp_drv );

    disp_drv.hor_res = TFT_HEIGHT;
    disp_drv.ver_res = TFT_WIDTH;
    disp_drv.flush_cb = disp_flush;
    disp_drv.draw_buf = &draw_buf;
#ifdef BOARD_HAS_PSRAM
    disp_drv.full_refresh = 1;
#endif
    lv_disp_drv_register( &disp_drv );

    // Initialize the input device driver
    static lv_indev_drv_t indev_touchpad;
    lv_indev_drv_init( &indev_touchpad );
    indev_touchpad.type = LV_INDEV_TYPE_POINTER;
    indev_touchpad.read_cb = touchpad_read2;
    lv_indev_drv_register( &indev_touchpad );
}

// ---------------------------------------------------------------------------

UIClass::UIClass()
{
  Serial.println("# init of UIClass()");

  //Add mutex to allow multitasking access
  xSemaphore = xSemaphoreCreateBinary();
  assert(xSemaphore);
  xSemaphoreGive( xSemaphore );

  setupLvgl();
}

void UI_TWatch_Class::spinner(bool show)
{
  if (show)
    lv_obj_clear_flag(spin, LV_OBJ_FLAG_HIDDEN);
  else
    lv_obj_add_flag(spin, LV_OBJ_FLAG_HIDDEN);
  lv_task_handler();
}

void UI_TWatch_Class::loop()
{
  lv_task_handler();
}

// ---------------------------------------------------------------------------

extern UIClass *theUI;

#include "../assets/twatch/tremola.h"
#include "../assets/twatch/splash.h"

void UI_TWatch_Class::refresh()
{
  lv_obj_invalidate( lv_scr_act() );
  lv_task_handler();
}

// ---------------------------------------------------------------------------

#include "../assets/twatch/tinySSB_logo_240x132.h"

#include "../assets/twatch/clock_ticks3.h"
#include "../assets/twatch/clock_hour_hand.h"   //  20x150
#include "../assets/twatch/clock_minute_hand.h" //  20x235
#include "../assets/twatch/clock_second_hand.h" //  40x200

static uint8_t pageId = 0;

static lv_style_t bgTransparent;
static lv_style_t tile_title_style;
static lv_style_t tile_text_style;
static lv_style_t peers_title_style;
static lv_style_t peers_text_style;
static lv_style_t config_text_style;

static lv_obj_t *tileview;
static lv_obj_t *tile_peers_list;
static lv_obj_t *tile_config_list;

static lv_obj_t *logo_img;
static lv_obj_t *hour_img;
static lv_obj_t *min_img;
static lv_obj_t *sec_img;

// ---------------------------------------------------------------------------

extern void update_peers();
extern void update_config();

void update_hand()
{
    if (pageId != 0)
        return;

    time_t now;
    struct tm  timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);

    lv_img_set_angle(hour_img,
                  ((timeinfo.tm_hour) * 300 + ((timeinfo.tm_min) * 5)) % 3600);
    lv_img_set_angle(min_img, (timeinfo.tm_min) * 60);
    lv_img_set_angle(sec_img, (timeinfo.tm_sec) * 60);
}

void mk_tile_clock(lv_obj_t *parent)
{
    bool antialias = true;

    lv_obj_t *ticks_img = lv_img_create(parent);
    lv_img_set_src(ticks_img, &clock_ticks3);
    lv_obj_center(ticks_img);
    lv_img_set_antialias(ticks_img, antialias);

    hour_img = lv_img_create(parent);
    lv_img_set_src(hour_img, &clock_hour_hand);
    lv_obj_center(hour_img);
    lv_img_set_pivot(hour_img,
                     clock_hour_hand.header.w / 2,
                     clock_hour_hand.header.h / 2);
    lv_img_set_antialias(hour_img, antialias);

    min_img = lv_img_create(parent);
    lv_img_set_src(min_img,  &clock_minute_hand);
    lv_obj_center(min_img);
    lv_img_set_pivot(min_img,
                     clock_minute_hand.header.w / 2,
                     clock_minute_hand.header.h / 2);
    lv_img_set_antialias(min_img, antialias);

    sec_img = lv_img_create(parent);
    lv_img_set_src(sec_img,  &clock_second_hand);
    lv_obj_center(sec_img);
    lv_img_set_pivot(sec_img,
                     clock_second_hand.header.w / 2,
                     clock_second_hand.header.h / 2);
    lv_img_set_antialias(sec_img, antialias);

    update_hand();

    static lv_timer_t *clockTimer = lv_timer_create([](lv_timer_t *timer)
      {
        update_hand();
        update_peers();
        update_config();
      }, 1000, NULL
    );

}

// ---------------------------------------------------------------------------

lv_obj_t* append_peer(lv_obj_t *flex, char *nm, char *rssi, char *age)
{
  lv_obj_t *ln = lv_obj_create(flex);
  lv_obj_set_size(ln, 240, 35);
  lv_obj_set_style_pad_all(ln, 0, 0);
  lv_obj_add_style(ln, &peers_text_style, LV_PART_MAIN);

  lv_obj_t *label = lv_label_create(ln);
  lv_obj_set_width(label, 110);
  lv_label_set_text(label, nm);
  lv_obj_set_pos(label, 10, 5);

  label = lv_label_create(ln);
  lv_obj_set_width(label, 50);
  lv_label_set_text(label, rssi);
  lv_obj_set_pos(label, 130, 5);

  label = lv_label_create(ln);
  lv_obj_set_width(label, 40);
  lv_label_set_text(label, age);
  lv_obj_set_pos(label, 190, 5);

  return ln;
}

void update_peers(void)
{
    if (pageId != 2)
        return;

    while (lv_obj_get_child_cnt(tile_peers_list) > 4)
        lv_obj_del(lv_obj_get_child(tile_peers_list, 4));

    long now = millis();
    char buf1[10], buf2[10];
    for (int i = 0; i < MAX_HEARD_PEERS; i++) {
        struct peer_s *p = thePeers->heard_peers + i;
        if (p->id[0] == '\0')
            break;
        sprintf(buf1, "%d", int(p->rssi));
        sprintf(buf2, "%d", (now - p->when) / 1000);
        append_peer(tile_peers_list, p->id, buf1, buf2);
    }
}

void mk_tile_peers(lv_obj_t *tile)
{
    extern char ssid[];

    lv_style_init(&peers_title_style);
    // lv_style_set_text_decor(&peers_title_style, LV_TEXT_DECOR_UNDERLINE);
    lv_style_set_bg_color(&peers_title_style, lv_color_hex(0xc0c0c0));
    lv_style_init(&peers_text_style);
    lv_style_set_bg_color(&peers_text_style, lv_color_hex(0xe0e0e0));

    lv_obj_set_layout(tile, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(tile, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(tile, LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START);

    lv_obj_t *label = lv_label_create(tile);
    lv_obj_add_style(label, &tile_title_style, LV_PART_MAIN);
    lv_label_set_text(label, "LoRa Peers");

    label = lv_label_create(tile);
    lv_obj_add_style(label, &tile_text_style, LV_PART_MAIN);
    lv_label_set_text(label, ssid);

    label = lv_label_create(tile);
    // lv_obj_add_style(label, &tile_text_style, LV_PART_MAIN);
    lv_label_set_text(label, "");

    lv_obj_t *title = append_peer(tile, "Peer", "RSSI", "Age");
    lv_obj_add_style(title, &peers_title_style, LV_PART_MAIN);
    /* test
    append_peer(tile, "X1", "Y", "Z");
    append_peer(tile, "X2", "Y", "Z");
    append_peer(tile, "X3", "Y", "Z");
    append_peer(tile, "X4", "Y", "Z");
    append_peer(tile, "X5", "Y", "Z");
    append_peer(tile, "X6", "Y", "Z");
    */
}

// ---------------------------------------------------------------------------

lv_obj_t* append_kv(lv_obj_t *flex, char *key, char *val)
{
  lv_obj_t *ln = lv_obj_create(flex);
  lv_obj_set_size(ln, 240, 30);
  lv_obj_set_style_pad_all(ln, 0, 0);
  lv_obj_add_style(ln, &tile_text_style, LV_PART_MAIN);
  lv_obj_set_style_border_width(ln, 0, LV_PART_MAIN);

  lv_obj_t *label = lv_label_create(ln);
  lv_obj_set_width(label, 105);
  lv_obj_set_pos(label, 5, 5);
  lv_label_set_long_mode(label, LV_LABEL_LONG_CLIP);
  lv_label_set_text(label, key);

  label = lv_label_create(ln);
  lv_obj_set_width(label, 115);
  lv_obj_set_pos(label, 120, 5);
  lv_obj_add_style(label, &config_text_style, LV_PART_MAIN);
  lv_label_set_long_mode(label, LV_LABEL_LONG_CLIP);
  lv_label_set_text(label, val);

  return ln;
}

void update_config(void)
{
    if (pageId != 3)
        return;

    while (lv_obj_get_child_cnt(tile_config_list) > 1)
        lv_obj_del(lv_obj_get_child(tile_config_list, 1));

    extern char *utc_compile_time;
    extern char ssid[];
    char buf[50];

    append_kv(tile_config_list, "this node", ssid);
    append_kv(tile_config_list, "compiled", utc_compile_time);

    append_kv(tile_config_list, "LoRa plan", the_lora_config->plan);
    int f = the_lora_config->fr / 10000;
    sprintf(buf, "%d.%02d MHz", f/100, f%100);
    append_kv(tile_config_list, "LoRa freq", buf);
    sprintf(buf, "%dkHz, %d",
            (int)(the_lora_config->bw/1000), the_lora_config->sf);
    append_kv(tile_config_list, "LoRa BW,SF", buf);

    sprintf(buf, "%d/%d (%d)", theGOset->goset_len,
            theGOset->largest_claim_span, theRepo->rplca_cnt);
    append_kv(tile_config_list, "# feeds", buf);
    sprintf(buf, "%d", theRepo->entry_cnt);
    append_kv(tile_config_list, "# entries", buf);
    sprintf(buf, "%d", theRepo->chunk_cnt);
    append_kv(tile_config_list, "# chunks", buf);

    int total = MyFS.totalBytes();
    int avail = total - MyFS.usedBytes();
    sprintf(buf, "%2d%% (%d MB)", avail / (total/100), avail/1024/1024);
    append_kv(tile_config_list, "avail. mem", buf);
}

void mk_tile_config(lv_obj_t *tile)
{
    lv_style_init(&config_text_style);
    lv_style_set_text_color(&config_text_style, lv_color_hex(0xD51B10));
    lv_style_set_bg_opa(&config_text_style, LV_OPA_TRANSP);

    lv_obj_set_layout(tile, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(tile, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(tile, LV_FLEX_ALIGN_SPACE_EVENLY,
                          LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START);

    lv_obj_t *label = lv_label_create(tile);
    lv_obj_add_style(label, &tile_title_style, LV_PART_MAIN);
    lv_label_set_text(label, "Config");
    lv_obj_set_style_pad_bottom(label, 15, 0);

    /* test
    append_kv(tile, "Key", "Val");
    append_kv(tile, "a1", "b");
    append_kv(tile, "a2", "b");
    append_kv(tile, "a3", "b");
    append_kv(tile, "a4", "b");
    append_kv(tile, "a5", "b");
    append_kv(tile, "a6", "b");
    */
}

// ---------------------------------------------------------------------------

void mk_tile_about(lv_obj_t *tile)
{
    lv_obj_set_layout(tile, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(tile, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(tile, LV_FLEX_ALIGN_SPACE_EVENLY,
                          LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START);

    lv_obj_t *label = lv_label_create(tile);
    lv_obj_add_style(label, &tile_title_style, LV_PART_MAIN);
    lv_label_set_text(label, "About tinySSB");
    // lv_obj_center(label);

    label = lv_label_create(tile);
    lv_obj_add_style(label, &tile_text_style, LV_PART_MAIN);
    lv_label_set_text(label,
                      "Feb 2025\n"
                      "https://github.com/\nssbc/tinySSB");
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);

    logo_img = lv_img_create(tile);
    lv_img_set_src(logo_img, &tinySSB_logo_240x132);
    lv_img_set_zoom(logo_img, 210);
}

// ---------------------------------------------------------------------------

void tileview_change_cb(lv_event_t *e)
{
    lv_obj_t *tileview = lv_event_get_target(e);
    pageId = lv_obj_get_index(lv_tileview_get_tile_act(tileview));
    /*
    lv_event_code_t c = lv_event_get_code(e);
    Serial.print("# tile CB -- code=");
    Serial.print(c);
    uint32_t count =  lv_obj_get_child_cnt(tileview);
    Serial.print(" count=");
    Serial.print(count);
    */
    Serial.print("# tile CB -- pageId=");
    Serial.println(pageId);
}


UI_TWatch_Class::UI_TWatch_Class()
{
    Serial.println("# init of UI_TWatch_Class()");

    lv_obj_t *label = 0;

    lv_style_init(&bg_style);
    lv_style_set_bg_color(&bg_style, lv_color_white());
    lv_style_set_text_color(&bg_style, lv_color_hex(0xff4040));

    scr = lv_scr_act();
    lv_obj_t *tmp;
    
    // background

    lv_obj_t *img = lv_img_create(scr);
    lv_img_set_src(img, &tremola);
    lv_obj_set_pos(img, 0, 0);

    logo_img = lv_img_create(scr); // transient logo
    lv_img_set_src(logo_img, &tinySSB_logo_240x132);
    lv_obj_center(logo_img);

    lv_task_handler();
    lv_obj_del(logo_img); // will be replaced by std background
 
    // spinner

    spin = lv_spinner_create(lv_scr_act(), 1500, 60);
    lv_obj_add_style(spin, &bg_style, LV_PART_ITEMS); // FIXME: no effect ...
    lv_obj_set_size(spin, 50, 50);
    lv_obj_center(spin);
    lv_obj_add_flag(spin, LV_OBJ_FLAG_HIDDEN);

    // tiles

    lv_style_init(&bgTransparent);
    lv_style_set_bg_opa(&bgTransparent, LV_OPA_TRANSP);

    lv_style_init(&tile_title_style);
    lv_style_set_text_color(&tile_title_style, lv_color_hex(0xD51B10));
    lv_style_set_text_font(&tile_title_style, &lv_font_montserrat_28);
    lv_style_set_bg_opa(&tile_title_style, LV_OPA_TRANSP);

    lv_style_init(&tile_text_style);
    lv_style_set_text_color(&tile_text_style, lv_color_black());
    lv_style_set_text_font(&tile_text_style, &lv_font_montserrat_18);
    lv_style_set_bg_opa(&tile_text_style, LV_OPA_TRANSP);

    tileview = lv_tileview_create(lv_scr_act());
    lv_obj_add_style(tileview, &bgTransparent, LV_PART_MAIN);
    lv_obj_set_size(tileview, lv_disp_get_hor_res(NULL), lv_disp_get_ver_res(NULL));
    lv_obj_add_event_cb(tileview, tileview_change_cb, LV_EVENT_VALUE_CHANGED, NULL);

    // clock
    lv_obj_t *t0_0 = lv_tileview_add_tile(tileview, 0, 0, LV_DIR_HOR);
    mk_tile_clock(t0_0);

    // chat
    lv_obj_t *t1_0 = lv_tileview_add_tile(tileview, 1, 0,
                                          LV_DIR_HOR | LV_DIR_BOTTOM);
    label = lv_label_create(t1_0);
    lv_obj_add_style(label, &tile_title_style, LV_PART_MAIN);
    lv_label_set_text(label, "Chat"); /*_fmt*/
    lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 0);

    // peers (name/RSSI/age)
    lv_obj_t *t2_0 = lv_tileview_add_tile(tileview, 2, 0, LV_DIR_HOR);
    tile_peers_list = t2_0;
    mk_tile_peers(t2_0);

    // settings
    lv_obj_t *t3_0 = lv_tileview_add_tile(tileview, 3, 0, LV_DIR_HOR);
    tile_config_list = t3_0;
    mk_tile_config(t3_0);

    // about
    lv_obj_t *t4_0 = lv_tileview_add_tile(tileview, 4, 0, LV_DIR_HOR);
    mk_tile_about(t4_0);
}

#endif // TINYSSB_BOARD_TWATCH

// eof
